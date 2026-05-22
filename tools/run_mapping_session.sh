#!/usr/bin/env bash
# Mapping session — automatyzacja end-to-end:
#   1. odpal fastlio_mapping.launch.py w tle
#   2. czekaj aż user przejedzie sceną + naciśnie Enter
#   3. /map_save → PCD plik
#   4. konwersja PCD → wiele wariantów PGM (pcd_variant_grid)
#   5. otwórz map_picker.py do wyboru najlepszej
#
# Założenie: Livox driver odpalony niezależnie (msg_MID360_launch.py).
# Plus workspace zbudowany + source install/setup.bash zrobione.

set -euo pipefail

# Katalog skryptu (odporny na zmianę cwd w trakcie działania).
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"

# --- konfiguracja ---
PCD_OUT="${PCD_OUT:-$HOME/maps/last_session.pcd}"
SCENARIOS_DIR="${SCENARIOS_DIR:-$HOME/maps/scenarios_$(date +%Y%m%d_%H%M%S)}"
FLIP_Y="${FLIP_Y:-1}"     # 1 = pass --flip-y (Mid-360 upside-down)
KILL_DELAY="${KILL_DELAY:-3}"   # s po /map_save przed kill launchu

# --- helpers ---
log() { printf '\033[1;36m[mapping_session]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[mapping_session]\033[0m %s\n' "$*" >&2; }
err() { printf '\033[1;31m[mapping_session]\033[0m %s\n' "$*" >&2; }

trap 'cleanup' EXIT INT TERM

LAUNCH_PID=0
cleanup() {
    if [[ $LAUNCH_PID -gt 0 ]] && kill -0 $LAUNCH_PID 2>/dev/null; then
        log "killing fastlio_mapping (pid=$LAUNCH_PID)..."
        kill -INT $LAUNCH_PID 2>/dev/null || true
        sleep 2
        kill -9 $LAUNCH_PID 2>/dev/null || true
    fi
    pkill -9 -f 'fastlio_mapping' 2>/dev/null || true
    pkill -9 -f 'laserMapping' 2>/dev/null || true
}

# --- pre-flight ---
if ! command -v ros2 &>/dev/null; then
    err "ros2 nie znalezione w PATH — czy install/setup.bash sourced?"
    exit 1
fi

if ! ros2 pkg prefix fast_lio &>/dev/null; then
    err "pakiet fast_lio nie zbudowany — patrz docs/fast_lio_setup.md"
    exit 1
fi

# Sprawdź czy Livox publikuje (najwyżej krótkie czekanie).
log "sprawdzam /livox/lidar (10 s timeout)..."
if ! timeout 10 bash -c 'until ros2 topic hz /livox/lidar 2>&1 | grep -q "average rate"; do sleep 1; done'; then
    warn "/livox/lidar nie publikuje. Czy livox_ros_driver2 chodzi?"
    warn "    ros2 launch livox_ros_driver2 msg_MID360_launch.py"
    read -p 'Kontynuować mimo to (y/N)? ' yn
    [[ "$yn" =~ ^[yY]$ ]] || exit 1
fi

# --- start FAST-LIO ---
mkdir -p "$(dirname "$PCD_OUT")"
cd "$(dirname "$PCD_OUT")"

log "startuję fastlio_mapping.launch.py..."
log "  Map zapis: $PCD_OUT"
log "  Scenariusze: $SCENARIOS_DIR"

ros2 launch g1_courier_fastlio fastlio_mapping.launch.py &
LAUNCH_PID=$!

# Daj FAST-LIO sekundę na warm-up.
sleep 3

if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    err "fastlio_mapping launch padło natychmiast — sprawdź log powyżej"
    exit 1
fi

# --- user-driven mapping ---
cat <<'EOF'

╔═══════════════════════════════════════════════════════════════════════════╗
║  MAPPING SESSION ACTIVE                                                     ║
║                                                                             ║
║  1. Jeźdź robotem powoli (≤ 0.15 m/s) po całej scenie                       ║
║  2. W RViz Fixed Frame ustaw "camera_init" — patrz jak mapa rośnie          ║
║  3. Gdy mapa kompletna, **wróć do startu** (loop closure)                   ║
║                                                                             ║
║  Po zakończeniu mapowania → naciśnij Enter tutaj                            ║
╚═══════════════════════════════════════════════════════════════════════════╝

EOF

read -p 'Press Enter when mapping done, then PCD save + variant gen will run: '

# --- save PCD ---
log "wywołuję /map_save service..."
if ! timeout 15 ros2 service call /map_save std_srvs/srv/Trigger {} 2>&1 | tee /tmp/map_save.out; then
    warn "/map_save service call timeout — czy FAST-LIO wciąż chodzi?"
fi

if grep -q 'success=True\|success=true' /tmp/map_save.out; then
    log "/map_save: success"
else
    warn "/map_save nie potwierdził sukcesu — sprawdź output powyżej"
fi

# FAST-LIO zapisuje do cwd jako `scans.pcd` lub `g1_map.pcd` w zależności od configu.
# Daj sekundę na flush + szukaj pliku.
sleep "$KILL_DELAY"

PCD_FOUND=""
for candidate in "g1_map.pcd" "scans.pcd"; do
    full="$(pwd)/$candidate"
    if [[ -s "$full" ]]; then
        PCD_FOUND="$full"
        break
    fi
done

if [[ -z "$PCD_FOUND" ]]; then
    err "PCD plik nie znaleziony w $(pwd) (szukałem: g1_map.pcd, scans.pcd)"
    err "Lista plików .pcd w cwd:"
    ls -la *.pcd 2>/dev/null || echo '  (brak)'
    exit 1
fi

log "znaleziono PCD: $PCD_FOUND ($(du -h "$PCD_FOUND" | cut -f1))"

# Przenieś do PCD_OUT (lepsza nazwa).
if [[ "$PCD_FOUND" != "$PCD_OUT" ]]; then
    mv "$PCD_FOUND" "$PCD_OUT"
    log "renamed → $PCD_OUT"
fi

# --- kill launch ---
log "stopping FAST-LIO launch..."
kill -INT $LAUNCH_PID 2>/dev/null || true
sleep 2

# --- variant generation ---
log "generuję warianty (pcd_variant_grid)..."
FLIP_ARG=""
[[ "$FLIP_Y" == "1" ]] && FLIP_ARG="--flip-y"
python3 "$SCRIPT_DIR/pcd_variant_grid.py" "$PCD_OUT" "$SCENARIOS_DIR" $FLIP_ARG

# --- launch picker ---
log "uruchamiam map picker..."
log "  W picker wybierz najlepszy wariant → 'Save as production map'"
log "  Mapa zostanie zapisana jako ~/maps/lab.yaml + lab.pgm"
log "  Potem: ros2 launch g1_courier_bringup real.launch.py map:=\$HOME/maps/lab.yaml"

python3 "$SCRIPT_DIR/map_picker.py" "$SCENARIOS_DIR" || true

log "session done — PCD: $PCD_OUT, scenarios: $SCENARIOS_DIR"

#!/usr/bin/env python3
"""Pomiar CPU na Jetsonie z /proc (bez pidstat/htop — Jetson ich nie ma).

  python3 cpu_probe.py [sekundy]

Liczy delty utime+stime per proces, sortuje malejaco i pokazuje udzial w
rdzeniach. Dla najciezszego procesu dodatkowo rozbija na WATKI — nav2 jest
skomponowany (jeden component_container_isolated), wiec bez tego nie widac,
ktory serwer nav2 pali CPU.
"""
import os
import sys
import time

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
HZ = os.sysconf('SC_CLK_TCK')
NCPU = os.cpu_count()


def read_stat(path):
    try:
        with open(path) as f:
            d = f.read()
    except OSError:
        return None
    # comm moze zawierac spacje/nawiasy — bierz po ostatnim ')'
    tail = d[d.rindex(')') + 2:].split()
    return int(tail[11]) + int(tail[12])          # utime + stime [ticks]


def name_of(pid):
    try:
        with open(f'/proc/{pid}/cmdline') as f:
            parts = [p for p in f.read().split('\0') if p]
    except OSError:
        parts = []
    if not parts:
        try:
            with open(f'/proc/{pid}/comm') as f:
                return f.read().strip()
        except OSError:
            return f'pid{pid}'
    base = os.path.basename(parts[0])
    # dla pythonowych node'ow nazwa skryptu mowi wiecej niz "python3"
    if base.startswith('python'):
        for p in parts[1:]:
            if p.endswith('.py') or ('/lib/' in p and not p.startswith('-')):
                return f'{base}:{os.path.basename(p)}'
    for p in parts:
        if p.startswith('__node:='):
            return f'{base}({p[8:]})'
    return base


def snapshot():
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        t = read_stat(f'/proc/{pid}/stat')
        if t is not None:
            out[pid] = t
    return out


def snap_threads(pid):
    out = {}
    try:
        tids = os.listdir(f'/proc/{pid}/task')
    except OSError:
        return out
    for tid in tids:
        t = read_stat(f'/proc/{pid}/task/{tid}/stat')
        if t is None:
            continue
        try:
            with open(f'/proc/{pid}/task/{tid}/comm') as f:
                nm = f.read().strip()
        except OSError:
            nm = tid
        out[tid] = (t, nm)
    return out


def main():
    print(f'probkuje {DUR:.0f} s   (rdzeni: {NCPU})', flush=True)
    la0 = os.getloadavg()
    a = snapshot()
    t0 = time.time()
    time.sleep(DUR)
    b = snapshot()
    el = time.time() - t0
    la1 = os.getloadavg()

    rows = []
    for pid, t1 in b.items():
        if pid in a:
            dcpu = (t1 - a[pid]) / HZ / el * 100.0
            if dcpu >= 1.0:
                rows.append((dcpu, pid, name_of(pid)))
    rows.sort(reverse=True)

    total = sum(r[0] for r in rows)
    print('=' * 68)
    print(f'{"CPU%":>7}  {"rdzenie":>8}  PID      proces')
    print('=' * 68)
    for dcpu, pid, nm in rows[:18]:
        print(f'{dcpu:7.1f}  {dcpu/100:8.2f}  {pid:<8} {nm}')
    print('-' * 68)
    print(f'{total:7.1f}  {total/100:8.2f}  SUMA (z {NCPU} rdzeni '
          f'= {total/NCPU:.0f}% wysycenia)')
    print(f'loadavg: {la0[0]:.2f} -> {la1[0]:.2f}')
    print('=' * 68)

    if rows:
        top_pid = rows[0][1]
        print(f'\nROZBICIE NA WATKI najciezszego procesu ({rows[0][2]}, '
              f'pid {top_pid}):')
        ta = snap_threads(top_pid)
        time.sleep(min(8.0, DUR / 2))
        tb = snap_threads(top_pid)
        trows = []
        for tid, (t1, nm) in tb.items():
            if tid in ta:
                d = (t1 - ta[tid][0]) / HZ / min(8.0, DUR / 2) * 100.0
                if d >= 1.0:
                    trows.append((d, nm, tid))
        trows.sort(reverse=True)
        for d, nm, tid in trows[:12]:
            print(f'{d:7.1f}%  {nm:<20} tid {tid}')
        if not trows:
            print('   (zaden watek > 1% w tym okienku)')
    print(flush=True)


if __name__ == '__main__':
    main()

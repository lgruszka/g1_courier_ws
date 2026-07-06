"""Watchdog toru skanu — samoleczenie znanego zacięcia p2l.

PROBLEM (realny G1): pointcloud_to_laserscan uruchomiony z real.launch.py
potrafi przestać publikować /scan po kilkunastu sekundach od startu stacku
(źródło /livox/lidar dalej nadaje). Ta sama binarka odpalona ręcznie po
ustabilizowaniu stacku działa bez zarzutu — zespół obchodził to DRUGIM p2l
ze skryptu, co dawało przeplot dwóch skanów na jednym topiku (gorsze).
Przyczyna środowiskowa nieustalona (churn subskrybentów przy aktywacji
nav2 / discovery w oknie startowym); ręczny restart zawsze leczy.

Ten węzeł automatyzuje tego ręcznego restarta i domyka dwie rzeczy naraz:

1. KEEPALIVE: trzyma stałą subskrypcję /scan. p2l ma leniwą subskrypcję
   (chmurę czyta tylko gdy ktoś słucha skanu) — watchdog gwarantuje
   niezerowego odbiorcę od t=0, więc leniwość nigdy nie rozpina toru.
2. RESTART: gdy /scan milczy dłużej niż scan_timeout_s, zabija procesy
   toru (pointcloud_to_laserscan_node + filter_crop_box_node); launch ma
   na nich respawn=true, więc wstają czyste i (jak przy ręcznym
   restarcie) działają. Backoff restart_backoff_s chroni przed pętlą
   restartów gdy milczy samo źródło (wtedy restart jest nieszkodliwy,
   ale nie ma co mielić co kilka sekund). Każdy restart głośno w logu —
   liczba restartów w logach to jednocześnie pomiar częstości zacięcia.
"""
from __future__ import annotations

import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Wzorce procesów toru skanu (pkill -f). Celowo szerokie: na robocie tor
# jest jeden; ewentualny debugowy p2l (run_pointcloud_laserscan.sh) też
# zostanie ubity — i dobrze, nie powinien działać obok stacku.
# filter_crop_box_node zostaje na liście, żeby wymieść ewentualne zabłąkane
# instancje pcl_ros ze starszych checkoutów.
_PIPELINE_PATTERNS = (
    'pointcloud_to_laserscan_node',
    'parcel_cropbox',
    'filter_crop_box_node',
)


class ScanWatchdog(Node):
    def __init__(self) -> None:
        super().__init__('scan_watchdog')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_timeout_s', 8.0)
        self.declare_parameter('initial_grace_s', 30.0)
        self.declare_parameter('restart_backoff_s', 30.0)

        self._timeout = float(self.get_parameter('scan_timeout_s').value)
        self._backoff = float(self.get_parameter('restart_backoff_s').value)
        grace = float(self.get_parameter('initial_grace_s').value)

        now = time.monotonic()
        # Do pierwszego skanu obowiązuje grace (p2l startuje z opóźnieniem
        # pointcloud_start_delay i czeka na chmurę z firmware).
        self._last_scan = now + grace - self._timeout
        self._last_restart = 0.0
        self._n_restarts = 0

        topic = str(self.get_parameter('scan_topic').value)
        self.create_subscription(
            LaserScan, topic, self._on_scan, qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'scan_watchdog: pilnuję {topic} (timeout {self._timeout:.0f} s, '
            f'grace {grace:.0f} s, backoff {self._backoff:.0f} s)')

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        silent = now - self._last_scan
        if silent <= self._timeout:
            return
        if now - self._last_restart < self._backoff:
            return
        self._last_restart = now
        self._n_restarts += 1
        self.get_logger().error(
            f'/scan milczy od {silent:.0f} s — restartuję tor skanu '
            f'(restart #{self._n_restarts}; launch respawn wstawi procesy '
            f'z powrotem). Jeśli to się powtarza, sprawdź czy żyje '
            f'/livox/lidar (ros2 topic hz).')
        for pattern in _PIPELINE_PATTERNS:
            subprocess.run(['pkill', '-f', pattern], check=False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Lernprogramm (Objective 2.8b, erste Haelfte): nimmt am aktuellen Standort eine
Place-Recognition-Signatur auf und speichert sie unter dem angegebenen Ortsnamen.

WICHTIG: Vor dem Start den Roboter von Hand exakt in die jeweilige lokale Fahrtrichtung
an GENAU DIESEM Wegpunkt ausrichten (nicht eine einzige globale Richtung fuer alle vier
Orte -- die Route aendert im Labyrinth mehrfach die Richtung). Siehe
PLACE_RECOGNITION_TASK.md Objective 2.8b ("Klaerung zur Trainingsausrichtung") fuer die
Begruendung. Dieses Skript sendet KEINE Kommandos an den Teensy und bewegt den Roboter
nicht -- reines Aufnahmeprogramm.

Nutzung (mit gesourctem ROS2-Environment, RPLidar-Node muss laufen):
    python3 learn_location.py P2
    python3 learn_location.py P2 --n-bins 360 --n-scans 10 --out signatures/
"""

import argparse
import os
import sys
import warnings

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(__file__))
from place_signature import LocationSignature, save_signature  # noqa: E402

DEFAULT_N_BINS = 360
DEFAULT_N_SCANS = 10  # Anzahl gemittelter Scans zur Rauschunterdrueckung (offener Parameter,
                       # siehe PLACE_RECOGNITION_TASK.md Abschnitt 4.4)


class LocationLearner(Node):
    """Sammelt n_scans Lidar-Scans und mittelt sie zu einer Winkel-Signatur mit n_bins
    Bins (Bin 0 = aktuelle Roboter-Blickrichtung bei Start des Sammelns).
    Wird auch von recognize_location.py wiederverwendet."""

    def __init__(self, n_bins, n_scans, node_name='location_learner'):
        super().__init__(node_name)
        self.n_bins = n_bins
        self.n_scans = n_scans
        self._bin_width = 2 * np.pi / n_bins
        self._collected = []
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, QoSPresetProfiles.SENSOR_DATA.value)
        self.get_logger().info(f'Warte auf {n_scans} Scans (NO_BINS={n_bins})...')

    def _scan_cb(self, msg):
        if self.done:
            return
        ranges = np.array(msg.ranges, dtype=np.float64)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges[~valid] = np.inf
        self._collected.append(self._bin_scan(ranges, angles))
        self.get_logger().info(f'Scan {len(self._collected)}/{self.n_scans} erfasst')

    def _bin_scan(self, ranges, angles):
        """Vektorisierte Zuordnung der Rohmessungen zu den n_bins Winkel-Bins
        (Mittelwert je Bin ueber alle Rohpunkte, die in den Bin fallen). Analog zum
        vektorisierten Stil in mcl_nav/mcl_nav/mcl.py (keine Python-Schleife ueber Punkte)."""
        angles_norm = np.mod(angles, 2 * np.pi)
        bin_idx = np.round(angles_norm / self._bin_width).astype(int) % self.n_bins
        finite = np.isfinite(ranges)
        sums = np.bincount(bin_idx[finite], weights=ranges[finite], minlength=self.n_bins)
        counts = np.bincount(bin_idx[finite], minlength=self.n_bins)
        binned = np.full(self.n_bins, np.inf)
        has_data = counts > 0
        binned[has_data] = sums[has_data] / counts[has_data]
        return binned

    @property
    def done(self):
        return len(self._collected) >= self.n_scans

    def averaged_signature(self):
        """Mittelt ueber alle gesammelten (bereits gebinnten) Scans. Bins ohne
        gueltige Messung in irgendeinem Scan bleiben inf (kein Wandtreffer) --
        das ist der erwartete Fall fuer Bins ohne Wandtreffer im Messbereich,
        daher wird numpy's 'Mean of empty slice'-Warnung hier bewusst unterdrueckt."""
        stacked = np.array(self._collected)  # (n_scans, n_bins)
        stacked_nan = np.where(np.isfinite(stacked), stacked, np.nan)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', category=RuntimeWarning)
            mean = np.nanmean(stacked_nan, axis=0)
        return np.where(np.isnan(mean), np.inf, mean)


def main():
    parser = argparse.ArgumentParser(description='Lernt eine Place-Recognition-Signatur.')
    parser.add_argument('location_id', help='z.B. P2, P4, P6, P9')
    parser.add_argument('--n-bins', type=int, default=DEFAULT_N_BINS)
    parser.add_argument('--n-scans', type=int, default=DEFAULT_N_SCANS)
    parser.add_argument('--out', default=os.path.join(os.path.dirname(__file__), 'signatures'))
    args = parser.parse_args()

    rclpy.init()
    node = LocationLearner(args.n_bins, args.n_scans)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        node.get_logger().warn('Abgebrochen, keine Signatur gespeichert.')
        node.destroy_node()
        rclpy.shutdown()
        return

    ranges = node.averaged_signature()
    sig = LocationSignature(location_id=args.location_id, angle_ranges=ranges)

    os.makedirs(args.out, exist_ok=True)
    out_path = os.path.join(args.out, f'{args.location_id}.pkl')
    save_signature(sig, out_path)
    node.get_logger().info(f'Signatur gespeichert: {out_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

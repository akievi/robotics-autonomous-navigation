#!/usr/bin/env python3
"""
Erkennungsprogramm (Objective 2.8b zweite Haelfte / 2.8c): nimmt am aktuellen
Standort eine Signatur auf und vergleicht sie gegen alle gespeicherten
Trainingssignaturen. Meldet den erkannten Ort (oder "kein Match") sowie die
geschaetzte Winkelabweichung zur Trainingsausrichtung (jeweilige lokale
Fahrtrichtung an diesem Wegpunkt, siehe PLACE_RECOGNITION_TASK.md Objective 2.8b).

Sendet KEINE Kommandos an den Teensy und bewegt den Roboter nicht -- reines
Diagnoseprogramm. Die tatsaechliche Reorientierung + Weiterfahrt (Bewertungsstufe
3) ist bewusst nicht Teil dieses Skripts, siehe PLACE_RECOGNITION_TASK.md
Abschnitt 4.1 Punkt 6 (Integration in navigator.py).

Nutzung (mit gesourctem ROS2-Environment, RPLidar-Node muss laufen):
    python3 recognize_location.py
    python3 recognize_location.py --sig-dir signatures/ --threshold 2253 --n-scans 10
"""

import argparse
import glob
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(__file__))
from learn_location import LocationLearner  # noqa: E402  Scan-Aufnahme + Binning wiederverwenden
from place_signature import (DEFAULT_DEPTH_BIN_EDGES, dk, estimate_rotation_shift,  # noqa: E402
                              frequency_histogram, load_signature, shift_to_degrees)

# Aus validate_offline.py fuer NO_BINS=360 vorgeschlagener Wert (synthetisch kalibriert,
# durch echte Hardwaretests bestaetigt -- siehe README.md).
DEFAULT_THRESHOLD = 2253.0
DEFAULT_N_SCANS = 10
# Anzahl der Kandidaten (nach Frequenz-Histogramm-Vorfilter), die zusaetzlich per
# Kreuzkorrelation der Winkel-Signatur feinverglichen werden. 2 behebt die bei echten
# Hardwaretests beobachtete P2/P4-Verwechslung (siehe README.md); mit nur 4 gespeicherten
# Orten waere auch top_k=4 rechnerisch kein Problem, 2 bleibt aber naeher am
# Effizienzgedanken aus Abschnitt 2.6 der Aufgabenstellung.
DEFAULT_TOP_K = 2


def load_all_signatures(sig_dir):
    sigs = {}
    for path in sorted(glob.glob(os.path.join(sig_dir, '*.pkl'))):
        sig = load_signature(path)
        sigs[sig.location_id] = sig
    return sigs


def recognize(test_ranges, stored, threshold, n_bins, top_k=DEFAULT_TOP_K):
    """Kernvergleichslogik, getrennt von ROS2-I/O testbar. Zweistufig:
    (1) schneller Vorfilter per rotationsinvariantem Frequenz-Histogramm (Abschnitt 2.6)
        liefert die top_k wahrscheinlichsten Orte.
    (2) Shift-Dk-Feinvergleich (Abschnitt 2.5) wird NUR unter den Kandidaten durchgefuehrt,
        die im Frequenz-Histogramm unterhalb des Thresholds liegen. Kandidaten oberhalb des
        Thresholds werden nicht beruecksichtigt -- sie wuerden sonst durch einen zufaellig
        guenstigeren Shift-Dk den echten Match verdraengen und KEIN MATCH erzeugen
        (beobachtet bei P6 mit Rotation: P9 hatte niedrigeren Shift-Dk, aber Freq-Dk=10729).
    Gibt ein dict mit allen Zwischenergebnissen zurueck."""
    test_hist = frequency_histogram(test_ranges, DEFAULT_DEPTH_BIN_EDGES)
    stored_hists = {loc: frequency_histogram(sig.angle_ranges, DEFAULT_DEPTH_BIN_EDGES)
                     for loc, sig in stored.items()}
    hist_scores = {loc: dk(test_hist, h) for loc, h in stored_hists.items()}

    ranked = sorted(hist_scores, key=hist_scores.get)
    candidates = ranked[:min(top_k, len(ranked))]

    # Nur Kandidaten unterhalb des Thresholds fuer Shift-Dk-Vergleich verwenden.
    # Falls keiner unterhalb liegt: alle Kandidaten (fuer Debugging-Ausgabe), aber kein Match.
    below = [loc for loc in candidates if hist_scores[loc] <= threshold]
    compare_set = below if below else candidates

    shift_by_candidate = {}
    for loc in compare_set:
        shift, shift_dk, _ = estimate_rotation_shift(test_ranges, stored[loc].angle_ranges)
        shift_by_candidate[loc] = (shift, shift_dk)

    best_loc = min(compare_set, key=lambda loc: shift_by_candidate[loc][1])
    best_dk = hist_scores[best_loc]

    result = dict(scores=hist_scores, candidates=candidates,
                   shift_dk={loc: v[1] for loc, v in shift_by_candidate.items()},
                   best_loc=best_loc, best_dk=best_dk,
                   matched=bool(below), angle_deg=None)

    if result['matched']:
        shift, _ = shift_by_candidate[best_loc]
        result['angle_deg'] = shift_to_degrees(shift, n_bins)

    return result


def main():
    parser = argparse.ArgumentParser(
        description='Erkennt den aktuellen Ort per Place Recognition (nur Diagnose, keine Fahrbefehle).')
    parser.add_argument('--sig-dir', default=os.path.join(os.path.dirname(__file__), 'signatures'))
    parser.add_argument('--n-scans', type=int, default=DEFAULT_N_SCANS)
    parser.add_argument('--threshold', type=float, default=DEFAULT_THRESHOLD,
                         help='Dk-Schwellenwert; siehe validate_offline.py fuer Kalibrierung')
    parser.add_argument('--top-k', type=int, default=DEFAULT_TOP_K,
                         help='Anzahl Kandidaten fuer die Winkel-Signatur-Feinpruefung')
    args = parser.parse_args()

    stored = load_all_signatures(args.sig_dir)
    if not stored:
        print(f'Keine Signaturen in {args.sig_dir} gefunden -- zuerst learn_location.py ausfuehren.')
        return
    n_bins = next(iter(stored.values())).n_bins
    print(f'{len(stored)} gespeicherte Signaturen geladen: {sorted(stored.keys())} (NO_BINS={n_bins})')

    rclpy.init()
    node = LocationLearner(n_bins, args.n_scans, node_name='location_recognizer')
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        return

    test_ranges = node.averaged_signature()
    node.destroy_node()
    rclpy.shutdown()

    result = recognize(test_ranges, stored, args.threshold, n_bins, top_k=args.top_k)

    print('\nDk je gespeichertem Ort (Frequenz-Histogramm, Vorfilter):')
    for loc, score in sorted(result['scores'].items(), key=lambda kv: kv[1]):
        marker = ' <-- Kandidat' if loc in result['candidates'] else ''
        print(f'  {loc}: Dk={score:.2f}{marker}')

    print(f'\nFeinvergleich der Top-{len(result["candidates"])}-Kandidaten '
          f'(Winkel-Signatur, bester Shift):')
    for loc, shift_dk in sorted(result['shift_dk'].items(), key=lambda kv: kv[1]):
        marker = ' <-- best' if loc == result['best_loc'] else ''
        print(f'  {loc}: Shift-Dk={shift_dk:.2f}{marker}')

    if not result['matched']:
        print(f'\nKEIN MATCH (bester Dk={result["best_dk"]:.2f} > '
              f'Schwellenwert={args.threshold:.2f})')
        return

    angle = result['angle_deg']
    corr = angle / 1.03   # Vorzeichen: positiv=Roboter CW/rechts → Korrektur CCW → T:+angle
    print(f'\nErkannter Ort : {result["best_loc"]}  (Dk={result["best_dk"]:.2f})')
    print(f'Abweichung    : {angle:+.1f} deg  '
          f'(positiv = Roboter rechts/CW von Trainingsausrichtung)')
    print(f'T:-Korrektur  : T:{corr:+.2f}  '
          f'(dreht Roboter zurueck zur Trainingsausrichtung, 1.03-Faktor bereits eingerechnet)')


if __name__ == '__main__':
    main()

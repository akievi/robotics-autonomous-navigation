"""
Datenstruktur und Vergleichslogik fuer Place-Recognition-Signaturen.
Reine Python/Numpy-Logik ohne ROS2-Abhaengigkeit -- offline testbar.

Siehe PLACE_RECOGNITION_TASK.md Abschnitt 2.4-2.6 fuer die zugrunde liegende
Aufgabenstellung (Dk-Vergleichstest, Rotationsschaetzung per Verschiebung,
rotationsinvariantes Tiefen-Frequenz-Histogramm).
"""

import pickle
from dataclasses import dataclass

import numpy as np

# Gemeinsame Tiefen-Bin-Grenzen fuer das rotationsinvariante Frequenz-Histogramm
# (Abschnitt 2.6). Von validate_offline.py, learn_location.py und recognize_location.py
# genutzt, damit Kalibrierung und Betrieb dieselbe Aufloesung verwenden.
DEFAULT_DEPTH_BIN_EDGES = np.linspace(0.0, 5.0, 51)  # 50 Bins, 10cm Aufloesung, 0-5m


@dataclass
class LocationSignature:
    """Signatur eines Ortes: n_bins Tiefenwerte ueber volle 360 Grad.
    Bin 0 = Referenzrichtung bei der Aufnahme (beim Training: globale x-Achse/
    Fahrtrichtung, siehe Objective 2.8b; beim Testen: aktuelle Roboter-Blickrichtung)."""

    location_id: str
    angle_ranges: np.ndarray  # (n_bins,) Meter, np.inf wo kein Wandtreffer

    @property
    def n_bins(self):
        return len(self.angle_ranges)


def save_signature(sig: LocationSignature, path: str):
    with open(path, 'wb') as f:
        pickle.dump(sig, f)


def load_signature(path: str) -> LocationSignature:
    with open(path, 'rb') as f:
        return pickle.load(f)


def frequency_histogram(angle_ranges, depth_bin_edges):
    """Rotationsinvariantes Tiefen-Frequenz-Histogramm (Objective 2.6 / Abschnitt 2.6).
    Zaehlt Vorkommen von Tiefenwerten in Tiefen-Bins -- unabhaengig vom Winkel,
    daher invariant gegenueber der Roboter-Orientierung bei der Aufnahme."""
    finite = np.asarray(angle_ranges)
    finite = finite[np.isfinite(finite)]
    hist, _ = np.histogram(finite, bins=depth_bin_edges)
    return hist.astype(np.float64)


def dk(hist_a, hist_b):
    """Summe der quadratischen Abweichungen Dk (Abschnitt 2.4 der Aufgabenstellung)."""
    return float(np.sum((np.asarray(hist_a) - np.asarray(hist_b)) ** 2))


def find_best_match(test_hist, stored_hists, threshold):
    """stored_hists: dict location_id -> Histogramm.
    Gibt (location_id_oder_None, dk_bester_match, alle_dk_werte) zurueck.
    None bedeutet 'kein Match' (bester Dk liegt ueber dem Schwellenwert)."""
    scores = {loc_id: dk(test_hist, h) for loc_id, h in stored_hists.items()}
    best_id = min(scores, key=scores.get)
    best_dk = scores[best_id]
    if best_dk > threshold:
        return None, best_dk, scores
    return best_id, best_dk, scores


def estimate_rotation_shift(test_angle_ranges, stored_angle_ranges):
    """Zirkulaere Kreuzkorrelation ueber alle Bin-Verschiebungen (Abschnitt 2.5).
    Testet fuer jede moegliche Verschiebung 'shift' den Dk-Wert zwischen der
    Test-Winkel-Signatur und der um 'shift' Bins gedrehten gespeicherten Signatur.
    Gibt (bester_shift_in_bins, dk_bei_bestem_shift, alle_dk_je_shift) zurueck.

    Konvention: np.roll(stored, shift) verschiebt Bin i der gespeicherten Signatur
    auf Index (i+shift) mod n. Das Vorzeichen der resultierenden Winkelkorrektur
    muss beim ersten Hardwaretest gegen die tatsaechliche Drehrichtung des Roboters
    verifiziert werden (siehe README.md, Abschnitt 'Offene Punkte fuer Hardwaretest')."""
    test_r = np.asarray(test_angle_ranges)
    stored_r = np.asarray(stored_angle_ranges)
    n = len(test_r)
    assert len(stored_r) == n, 'Signaturen muessen gleiche Bin-Anzahl haben'

    # inf (kein Wandtreffer) durch einen grossen endlichen Wert ersetzen,
    # damit die quadratische Abweichung nicht explodiert/NaN wird.
    test_v = np.where(np.isfinite(test_r), test_r, 10.0)
    stored_v = np.where(np.isfinite(stored_r), stored_r, 10.0)

    dks = np.empty(n)
    for shift in range(n):
        shifted = np.roll(stored_v, shift)
        dks[shift] = np.sum((test_v - shifted) ** 2)

    best_shift = int(np.argmin(dks))
    return best_shift, float(dks[best_shift]), dks


def shift_to_degrees(shift_bins, n_bins):
    """Wandelt einen Bin-Shift in eine Gradzahl im Bereich (-180, 180] um."""
    deg = shift_bins * 360.0 / n_bins
    if deg > 180.0:
        deg -= 360.0
    return deg

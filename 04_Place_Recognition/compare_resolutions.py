"""
Vergleicht dieselbe (echte) Signatur bei mehreren Winkelaufloesungen nebeneinander --
Objective 2.8a: "Experimentiere mit der Abtastrate, um herauszufinden, wie gross die
Signatur wirklich sein muss, um einen Ort zu charakterisieren." Das war bisher nur
numerisch ueber Dk/Erkennungsgenauigkeit in validate_offline.py geprueft (synthetisch),
nicht visuell an echten Daten -- dieses Skript schliesst die Luecke.

Nimmt eine bereits gespeicherte Signatur (native Aufloesung, z.B. NO_BINS=360) und bildet
per Downsampling (Mittelung benachbarter Bins) niedrigere Aufloesungen nach -- eine
Naeherung an eine native Aufnahme mit weniger Bins, ohne erneut scannen zu muessen.

Nutzung:
    python3 compare_resolutions.py signatures/P2.pkl
    python3 compare_resolutions.py signatures/P2.pkl --bins 36 72 180 360
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from place_signature import LocationSignature, load_signature  # noqa: E402
from visualize_signature import plot_signature  # noqa: E402

DEFAULT_BINS = [36, 72, 180, 360]


def downsample(sig, n_bins_target):
    """Naehert eine groebere Winkelaufloesung an, durch Mittelung benachbarter Bins der
    Original-Signatur (analog zum vektorisierten Binning-Stil in learn_location.py).
    n_bins_target muss die Original-Bin-Anzahl teilen."""
    n_orig = sig.n_bins
    if n_bins_target == n_orig:
        return sig
    if n_orig % n_bins_target != 0:
        raise ValueError(f'{n_orig} Original-Bins nicht durch {n_bins_target} teilbar')

    group = n_orig // n_bins_target
    ranges = sig.angle_ranges.reshape(n_bins_target, group)
    finite = np.isfinite(ranges)
    counts = finite.sum(axis=1)
    sums = np.where(finite, ranges, 0.0).sum(axis=1)

    out = np.full(n_bins_target, np.inf)
    has_data = counts > 0
    out[has_data] = sums[has_data] / counts[has_data]
    return LocationSignature(location_id=f'{sig.location_id}', angle_ranges=out)


def main():
    parser = argparse.ArgumentParser(
        description='Vergleicht eine Signatur bei mehreren Winkelaufloesungen (Objective 2.8a).')
    parser.add_argument('path', help='Pfad zu einer gespeicherten .pkl-Signatur (native Aufloesung)')
    parser.add_argument('--bins', type=int, nargs='+', default=DEFAULT_BINS)
    parser.add_argument('--out', default=os.path.join(os.path.dirname(__file__),
                                                        'resolution_comparison.png'))
    args = parser.parse_args()

    sig = load_signature(args.path)
    bins_sorted = sorted(b for b in args.bins if b <= sig.n_bins)
    if not bins_sorted:
        raise SystemExit(f'Keiner der angegebenen --bins-Werte ist <= native Aufloesung '
                          f'({sig.n_bins} Bins) der Signatur.')

    variants = [downsample(sig, b) for b in bins_sorted]

    fig, axes = plt.subplots(1, len(variants), subplot_kw=dict(projection='polar'),
                              figsize=(5 * len(variants), 5))
    if len(variants) == 1:
        axes = [axes]
    for ax, v, n_bins in zip(axes, variants, bins_sorted):
        plot_signature(v, ax)
        ax.set_title(f'{v.location_id}  ({n_bins} Bins, downsampled)'
                     if n_bins != sig.n_bins else f'{v.location_id}  ({n_bins} Bins, original)')

    plt.tight_layout()
    plt.savefig(args.out, dpi=120)
    print(f'Vergleichsplot gespeichert: {args.out}')


if __name__ == '__main__':
    main()
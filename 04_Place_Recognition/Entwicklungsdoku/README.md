# Place Recognition — Offline-Vorarbeit & Hardware-Diagnoseprogramme

Dieser Ordner ist bewusst **komplett getrennt** vom produktiven `mcl_nav`-Package. Es wird
**keine bestehende Datei** (`mcl_nav/mcl.py`, `mcl_nav/navigator.py`, `config/map_config.yaml`,
`setup.py`, ...) verändert — alles hier ist neuer, eigenständiger Code, der die bestehenden
Dateien nur **lesend** nutzt (Wandmodell aus `map_config.yaml`, Raycast-Funktion aus `mcl.py`).

Kontext/Aufgabenstellung: siehe [`../PLACE_RECOGNITION_TASK.md`](../PLACE_RECOGNITION_TASK.md).
Verlauf/Historie (was wann warum geändert wurde, Hardware-Testergebnisse): siehe
[`CHANGELOG.md`](CHANGELOG.md).

## Aktueller Stand (Kurzfassung, Details im Changelog)

- Offline-Kalibrierung gegen das synthetische Wandmodell: **P2, P4, P6, P9 klar unterscheidbar**,
  Vorschlag `NO_BINS=360`, `Dk_threshold≈2253`.
- **Stufe 1 (feste Orientierung) und Stufe 2 (beliebige Orientierung) sind an echter Hardware
  validiert: P2/P4/P6/P9 jeweils 4/4 korrekt erkannt**, inkl. Vorzeichen-Verifikation der
  Winkelschätzung über alle vier Orte (West- und Ost-referenziert). Details zu den vier
  Testrunden (inkl. behobener Fehler: P6-Rotationsproblem, P2/P4-Verwechslung, Vorzeichen-Bug,
  Meldestörung bei P9) siehe `CHANGELOG.md`.
- Visualisierung (Objective 2.8a) vollständig: radiale Liniendarstellung, Live-Scan-Modus ohne
  vorheriges Speichern, und visueller Auflösungsvergleich (36–360 Bins) an echten Daten.
- **Nächster Schritt: Stufe 3** (Reorientierung + Weiterfahrt), Integration in `navigator.py`
  — siehe unten.

## Was hier drin ist

| Datei/Ordner | Zweck |
|---|---|
| `place_signature.py` | Kernlogik: `LocationSignature`-Klasse, Speichern/Laden, `Dk`-Vergleichstest (Abschnitt 2.4), rotationsinvariantes Tiefen-Frequenz-Histogramm (Abschnitt 2.6), zirkuläre Kreuzkorrelation zur Winkelschätzung (Abschnitt 2.5). Reine Numpy-Logik, kein ROS2 nötig. |
| `synthetic_scan.py` | Erzeugt synthetische 360°-Scans aus dem Wandmodell, per Wiederverwendung von `MCL._raycast_single_particle` aus `mcl_nav/mcl_nav/mcl.py` (nur Import, keine Änderung). Läuft auch ohne ROS2-Laufzeitumgebung (`mcl.py` hängt nur von `numpy` ab). |
| `validate_offline.py` | Offline-Kalibrierung: baut synthetische Trainingssignaturen für P2/P4/P6/P9 (theta=0, entlang x-Achse), simuliert 200 verrauschte Testfahrten pro Ort, testet mehrere `NO_BINS`-Werte, schlägt `NO_BINS` + `Dk_threshold` vor. Schreibt **ausschließlich** nach `signatures_synthetic/` und `report.txt` — fasst `signatures/` (echte Daten!) nicht an. |
| `visualize_signature.py` | Stellt eine oder mehrere Signaturen als radiale Linien dar (Objective 2.8a), speichert `signature_plot.png`. Mit `--live` direkt scannen ohne vorheriges Speichern (RPLidar-Node muss laufen). |
| `compare_resolutions.py` | Vergleicht eine gespeicherte Signatur bei mehreren `NO_BINS`-Werten nebeneinander (Downsampling durch Mittelung benachbarter Bins) — visueller Nachweis zu Objective 2.8a ("wie groß muss die Signatur sein"), ergänzt die rein numerische Prüfung in `validate_offline.py`. Speichert `resolution_comparison.png`. |
| `learn_location.py` | ROS2-Node/Programm (Objective 2.8b, erste Hälfte): sammelt `n_scans` echte `/scan`-Nachrichten, mittelt sie zu einer Signatur (`LocationLearner`-Klasse) und speichert sie nach `signatures/`. Sendet keine Teensy-Kommandos. |
| `recognize_location.py` | ROS2-Node/Programm (Objective 2.8b zweite Hälfte / 2.8c): nimmt eine Signatur auf (nutzt `LocationLearner` wieder), filtert per Frequenz-Histogramm die Top-`k` Kandidaten vor und verfeinert unter diesen per Winkel-Signatur-Kreuzkorrelation. Reines Diagnoseprogramm, sendet keine Teensy-Kommandos. |
| `lidar_only.launch.py` | Startet nur den RPLidar-Node (kein IMU, kein Navigator) — verhindert, dass der Roboter beim manuellen Positionieren autonom losfährt. |
| `signatures/` | **Echte**, mit `learn_location.py` an der Hardware aufgenommene Trainingssignaturen für P2, P4, P6, P9. Das ist die für den Betrieb relevante Quelle. |
| `signatures_synthetic/` | Rein synthetische Signaturen aus `validate_offline.py`, nur für Offline-Tests/Regressionstests dieses Codes. Getrennt von `signatures/`, damit ein erneuter `validate_offline.py`-Lauf nie echte Daten überschreibt. |
| `report.txt` | Textkopie der letzten `validate_offline.py`-Ausgabe. |
| `CHANGELOG.md` | Versionshistorie inkl. Hardware-Testergebnissen und warum welche Entscheidung getroffen wurde. |

## Ausführen

**Rein offline (kein ROS2-Sourcing nötig, numpy/yaml/matplotlib sind bereits installiert):**

```bash
cd place_recognition_dev
python3 validate_offline.py
python3 visualize_signature.py signatures/P2.pkl signatures/P4.pkl signatures/P6.pkl signatures/P9.pkl
python3 compare_resolutions.py signatures/P2.pkl --bins 36 72 180 360
```

**Mit echtem Roboter** (nur Lidar nötig, kein Navigator/Teensy):

```bash
source /opt/ros/iron/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ~/ros2_ws/src/mcl_nav/place_recognition_dev/lidar_only.launch.py
```

In einem zweiten Terminal:

```bash
source /opt/ros/iron/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/mcl_nav/place_recognition_dev

# Lernen: Roboter von Hand exakt in die jeweilige lokale Fahrtrichtung an diesem
# Wegpunkt ausrichten (nicht eine globale Richtung -- siehe PLACE_RECOGNITION_TASK.md 2.8b),
# dann:
python3 learn_location.py P2   # entsprechend P4, P6, P9

# Erkennen (Roboter an beliebigem gelerntem Ort, beliebige Orientierung):
python3 recognize_location.py
python3 recognize_location.py --top-k 2 --threshold 2253   # Standardwerte, explizit

# Live-Visualisierung ohne vorheriges Speichern:
python3 visualize_signature.py --live
```

`learn_location.py`/`recognize_location.py` senden **keine** Kommandos an den Teensy und bewegen
den Roboter nicht — reine Diagnoseprogramme, sicher zum Testen ohne Aufsicht über die Fahrlogik.

## Wichtige Einschränkungen (Stand nach Hardwaretest-Runde 4, siehe CHANGELOG.md für Details)

1. **`Dk`-Werte sind nicht über `NO_BINS` normiert** — beim Vergleich verschiedener `NO_BINS`-
   Werte in `validate_offline.py` ist "größere Trennmarge" teilweise ein Rechenartefakt.
2. **`NO_BINS=360`** entspricht ungefähr der realen Winkelauflösung des RPLidar A1 (~360–460
   Punkte/Umdrehung) — mehr bringt real keine zusätzliche Information. Visuell bestätigt mit
   `compare_resolutions.py` an echten P2-Daten: die charakteristische Kontur ist schon bei
   36 Bins klar erkennbar, 360 Bins fügt kaum mehr als einen einzelnen Rausch-Ausreißer hinzu.
3. **P2-Signatur ist empfindlich**: Die erste P2-Signatur war bei 90° rechts mit P4 verwechselbar
   (Freq-Dk P4=2236 < P2=4511). Nach Neu-Lernen: P2-Dk=717, P4=2169 → korrekte Erkennung
   dank Shift-Dk-Tiebreaker (P2=388 < P4=458). Signatur in `signatures/P2.pkl` = 2026-08-04.
4. **Vorzeichenkonvention der Winkelschätzung** — verifiziert (Runde 3/4):
   positiv = Roboter rechts/CW von Trainingsausrichtung; T-Korrektur = T:+angle/1.03 (CCW). ✓
5. **`--live`-Modus von `visualize_signature.py`** ist code-fertig und smoke-getestet
   (Import/Konstruktion), aber noch nicht gegen echte Lidar-Daten gelaufen (keine Hardware in
   der Session, die diesen Modus gebaut hat, aktiv angesteuert) — analog zu `learn_location.py`
   bei dessen erstem Einsatz.

## Nächster Schritt: Stufe 3 — Navigator-Integration

Integration in `mcl_nav/mcl_nav/navigator.py`:
- Neuer optionaler Startmodus: statt bei P1 zu starten, erst Place Recognition durchführen
- Reorientierung per T:-Kommando auf Fahrtrichtung des erkannten Waypoints
- Weiterfahrt ab erkanntem Waypoint bis P11 (wie normaler Betrieb)
- Siehe `PLACE_RECOGNITION_TASK.md` Abschnitt 4.1 Punkt 6

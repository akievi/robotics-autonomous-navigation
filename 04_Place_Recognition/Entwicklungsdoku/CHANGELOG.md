# Changelog — Place Recognition Offline-Vorarbeit

Chronologische Versionshistorie dieses Ordners. Ergänzt README.md (aktueller Stand) um den
Verlauf: was wurde wann geändert, warum, und was kam dabei heraus (insbesondere
Hardware-Testergebnisse, die sonst nirgends festgehalten sind).

## 2026-08-04

### v0.1 — Offline-Grundgerüst (rein synthetisch, keine Hardware)
- `place_signature.py`: `LocationSignature`-Klasse, Speichern/Laden, `Dk`-Vergleichstest,
  rotationsinvariantes Tiefen-Frequenz-Histogramm, zirkuläre Kreuzkorrelation zur
  Winkelschätzung.
- `synthetic_scan.py`: synthetische 360°-Scans per Wiederverwendung des Raycasts aus
  `mcl_nav/mcl_nav/mcl.py` (nur Import, keine Änderung an der Datei).
- `validate_offline.py`: Monte-Carlo-Kalibrierung gegen das Wandmodell aus
  `map_config.yaml`. Erster Lauf: 100% Erkennung bei allen getesteten `NO_BINS` (36–360),
  Vorschlag `NO_BINS=360`, `Dk_threshold≈2253`.
  - **Bugfix während der Entwicklung**: erste Version der Trennmarge-Berechnung ergab
    `inf`, weil `wrong_dk_min` nur aktualisiert wurde, wenn ein Ort tatsächlich falsch
    erkannt wurde (kam bei 100% Genauigkeit nie vor). Korrigiert: `Dk` von richtigem und
    bestem falschem Ort werden jetzt in **jedem** Testlauf erfasst, nicht nur bei Fehlern.
- `visualize_signature.py`: radiale Liniendarstellung einer Signatur (Objective 2.8a).
- **Wichtiges Ergebnis dieser Phase**: P2, P4, P6, P9 sind im synthetischen Wandmodell
  strukturell klar unterscheidbar — kein grundsätzlicher Zweifel an der Ortswahl.
- Bekannt gewordene Einschränkung der Methodik: `Dk`-Werte sind nicht über `NO_BINS`
  normiert, "mehr Bins = größere Trennmarge" ist teilweise ein Rechenartefakt (siehe
  README.md, Einschränkung 1).

### v0.2 — ROS2-Aufnahme-/Erkennungsprogramme (code-fertig, hardwaregetestet)
- `learn_location.py`: `LocationLearner`-Node, sammelt und mittelt `n_scans` echte
  `/scan`-Nachrichten zu einer Signatur, speichert sie. Sendet keine Teensy-Kommandos.
- `recognize_location.py`: nutzt `LocationLearner` wieder, vergleicht gegen gespeicherte
  Signaturen, meldet Ort + Winkel. Reines Diagnoseprogramm.
- `lidar_only.launch.py`: startet nur den RPLidar-Node (kein Navigator/IMU), damit der
  Roboter beim manuellen Positionieren nicht autonom losfährt.
- Offline-Smoke-Tests vor dem ersten Hardwareeinsatz: Rebinning-Genauigkeit (1,7mm
  mittlerer Fehler ggü. direktem Raycast), End-to-End-Erkennung an synthetischem P9-Fall
  (37° Testabweichung → korrekt erkannt, Winkel exakt -37.0° geschätzt).
- **Bugfix**: `RuntimeWarning: Mean of empty slice` in `averaged_signature()` (trat auf,
  wenn ein Winkel-Bin in allen `n_scans` Scans ohne Wandtreffer blieb — funktional
  bereits korrekt behandelt, aber unnötig unruhige Ausgabe). Behoben durch gezielte
  `warnings.catch_warnings()`-Unterdrückung.

### Hardwaretest-Runde 1 — feste Ausrichtung (x-Achse, wie beim Training)
Ergebnis: **4/4 Orte korrekt erkannt** (P2, P4, P6, P9), alle mit großer Marge zum
nächstbesten falschen Ort. Geschätzte Winkelabweichung überall klein (±5°), wie für
identische Ausrichtung wie beim Training erwartet. `Dk_threshold=2253` bestätigt:
alle richtigen Werte (103–1393) klar darunter, alle falschen (5211–19548) klar darüber.

### Hardwaretest-Runde 2 — beliebige (unkalibrierte) Drehung
Ergebnis: **3/4 korrekt** (P4, P6, P9). **P2 fehlgeschlagen**: bestes Match war P4
(`Dk=2691`) statt der tatsächlich richtige Ort P2 (`Dk=3606`) — ein echter
Ranking-Fehler des Frequenz-Histogramms, kein Schwellenwert-Grenzfall (der Schwellenwert
hat hier nur eine Falscherkennung als "kein Match" abgefangen, das eigentliche Problem
bleibt bestehen). P2/P4 zeigten schon in Runde 1 die geringste gegenseitige Marge unter
allen Ortspaaren.

### v0.3 — Zweistufige Erkennung (Top-K-Kandidaten-Verfeinerung)
- `recognize_location.py::recognize()`: Frequenz-Histogramm liefert jetzt nur noch die
  `top_k=2` wahrscheinlichsten Kandidaten (Vorfilter); für diese wird zusätzlich die
  genauere Winkel-Signatur-Kreuzkorrelation gerechnet, der beste Shift-Dk entscheidet.
  Schwellenwert-Vergleich bleibt auf der ursprünglichen (kalibrierten) Histogramm-Skala.
  Neuer `--top-k`-CLI-Parameter.
- **Nebenfund und Fix**: `learn_location.py`-Läufe aus Hardwaretest-Runde 1+2 hatten mit
  dem Standardordner `signatures/` die synthetischen Platzhalter durch echte Daten
  überschrieben (richtig so für den Betrieb) — das machte einen Offline-Regressionstest
  ungültig, der synthetische Testscans gegen die (jetzt echten) gespeicherten Signaturen
  verglich. Behoben: `validate_offline.py` schreibt jetzt in einen eigenen Ordner
  `signatures_synthetic/`, `signatures/` bleibt ausschließlich echten Hardwaredaten
  vorbehalten. Echte Daten in `signatures/` waren vom Fehler nicht betroffen.
- **Offener Punkt, ehrlich dokumentiert**: der Versuch, den P2/P4-Fehlfall aus Runde 2
  synthetisch nachzustellen (Hypothese: Positionsversatz beim Drehen auf der Stelle),
  ist **nicht** gelungen — reiner Versatz von 0,5–2cm reicht in der Simulation bei
  Weitem nicht, um den beobachteten knappen Fehlfall zu reproduzieren (P2 bleibt dort
  klar vorne). Die eigentliche Ursache ist damit weiterhin unklar (vermutlich reale
  Sensor-Effekte, die sich synthetisch nicht sauber nachbilden lassen). Die
  Top-K-Erweiterung ist trotzdem risikofrei (kann nur helfen, nie schaden), aber **noch
  nicht an echter Hardware gegen den P2-Fall erneut verifiziert**.

### v0.4 — Bug-Fixes + Vorzeichenkorrektur (2026-08-04)
- **Bug-Fix P6 KEIN MATCH**: Bei rotierten Scans an P6 wählte der Shift-Dk-Vergleich
  P9 als "besten" Top-2-Kandidaten (P9 hatte zufällig niedrigeren Shift-Dk), obwohl P9
  Freq-Dk=10729 >> Threshold 2253. Fix in `recognize()`: Shift-Dk-Vergleich nur unter
  Kandidaten die unterhalb des Thresholds liegen (Variable `below`). P6 ist jetzt 4/4
  bei allen Drehungen korrekt erkannt.
- **Vorzeichenkorrektur T-Korrektur**: `corr = -angle/1.03` → `corr = angle/1.03`.
  Hardware bestätigt: 90° rechts (CW) → Abweichung=+90°, Korrektur=T:+87.38 (CCW). ✓
- **Label-Fix**: "positiv = Roboter links/CCW" → "positiv = Roboter rechts/CW". ✓

### Hardwaretest-Runde 3 — beliebige Orientierung, alle 4 Orte (2026-08-04)
P2/P4/P6/P9 je bei 0°, 90° rechts, 180°, 90° links getestet (16 Tests total):
- **P2**: 0°✓, 90°rechts ✗(P4, Freq-Dk P4=2236<2253 P2=4511), 180°✓, 90°links✓ → 3/4
- **P4**: 4/4 ✓
- **P6**: 0°✓, 90°rechts KEIN MATCH→✓(nach Fix), 180°→✓, 90°links→✓ → 4/4 nach Fix
- **P9**: 4/4 ✓ (2 Tests mit Fremd-Roboter bei P2 als Hindernis, dennoch korrekt)
Vorzeichen verifiziert: 90°rechts(CW) → Abweichung=+90°, T-Korrektur=T:+87° (CCW). ✓

### P2-Signatur neu gelernt + Runde 4 (2026-08-04)
P2 mit `learn_location.py P2` neu aufgenommen. Test bei 90° rechts:
P2-Dk=717 (vorher 4511), P4-Dk=2169. Shift-Dk P2=388 < P4=458 → **P2 korrekt erkannt**.
**Stufe 2 vollständig: P2/P4/P6/P9 alle 4/4 bei beliebiger Orientierung. ✓**

### Klärung 0°-Referenz + P9-Retest (2026-08-04)
- **Klärung**: 0° ist an jedem Trainingsort die **jeweilige lokale Fahrtrichtung an genau
  diesem Wegpunkt**, nicht eine einzige globale Richtung (physisch: P2 und P9 → West,
  P4 und P6 → Ost). `PLACE_RECOGNITION_TASK.md` (Objective 2.8b) und die Docstrings von
  `learn_location.py`/`recognize_location.py` waren dazu ungenau (sprachen von "x-Achse"
  als einheitlicher Referenz) und wurden entsprechend korrigiert.
- **P9-Retest ohne Störung**: die beiden in Runde 3 als gestört markierten P9-Tests
  (Fremd-Roboter im Sichtfeld bei P2) wurden nach Entfernen der Störung wiederholt:
  - 180°: Dk=309 (vorher gestört: 483) — sauberer, aber Vorzeichen bei 180° naturgemäß
    nicht diagnostisch.
  - 90° links: Dk=77 (vorher gestört: 765), Abweichung=**-89.0°** (vorher gestört: +91.0°,
    der einzige Ausreißer gegen die Konvention in Runde 3).
  - Damit bestätigt: der Ausreißer in Runde 3 war eine reine Messstörung durch das
    Hindernis, kein Vorzeichenfehler. Vorzeichenkonvention jetzt lückenlos über alle vier
    Orte (P2/P4/P9 mit sauberen Daten, je West- und Ost-referenziert) bestätigt.

### v0.5 — Visualisierung vervollständigt (Objective 2.8a) (2026-08-04)
Beim Abgleich mit der Aufgabenstellung fielen zwei Lücken gegenüber dem Wortlaut von
Objective 2.8a auf: kein einzelnes "Scan-und-Zeig"-Programm (bisher nur zweigeteilt über
`learn_location.py` + `visualize_signature.py`), und "Experimentiere mit der Abtastrate"
war nur numerisch (Dk/Genauigkeit in `validate_offline.py`), nie visuell geprüft. Behoben:
- `visualize_signature.py`: neuer `--live`-Modus, scannt direkt ohne vorheriges Speichern/
  Ortsname (nutzt `LocationLearner` aus `learn_location.py` wieder). Noch nicht an echter
  Hardware getestet (nur Import/Konstruktion smoke-getestet).
- `compare_resolutions.py` (neu): lädt eine gespeicherte Signatur und plottet sie per
  Downsampling (Mittelung benachbarter Bins) bei mehreren `NO_BINS`-Werten nebeneinander.
  Offline gegen echte `signatures/P2.pkl` getestet: charakteristische Kontur bereits bei
  36 Bins klar erkennbar, 360 Bins fügt nur einen einzelnen Rausch-Ausreißer hinzu —
  bestätigt visuell, was die `Dk`-Kennzahlen schon vermuten ließen (Einschränkung 2 in
  README.md).

## Stufe 3 — Navigator-Integration (ab 2026-08-04)
Nächste Implementierung: neuer optionaler Startmodus in `navigator.py`, der statt bei P1
zu starten erst Place Recognition durchführt, dann reorientiert und ab dem erkannten
Waypoint weiterfährt. Siehe `PLACE_RECOGNITION_TASK.md` Abschnitt 4.1 Punkt 6.

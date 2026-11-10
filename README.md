# esphome-tmc22xx

ESPHome External Component für **TMC2208** und **TMC2209** an ESP32 (alle Varianten mit GPTimer).
Schritterzeugung in einer GPTimer-ISR mit Integer-Rampenplaner, optionale Treiberkonfiguration und
Diagnose über Single-Wire-UART, Homing über Endschalter (beide Chips) oder StallGuard4 (nur TMC2209).

Getestet mit ESPHome **2026.10.2** (ESP-IDF 5.5.6): Builds für `esp32dev` und `esp32-c3-devkitm-1`.
Die Komponente nutzt die Codegen-APIs `register_apply_action` / `build_callback_automations`; ältere
ESPHome-Versionen werden nicht unterstützt.

## Verzeichnisstruktur

```
components/tmc22xx/
  __init__.py            Schema, Validierung, Codegen, Actions/Conditions/Trigger
  sensor.py              position, stallguard_result, actual_current
  binary_sensor.py       moving, homed, homing, driver_error, overtemperature_warning, stall
  text_sensor.py         driver_status
  button.py              home, stop, emergency_stop
  number.py              Zielposition
  tmc22xx.h/.cpp         Komponente: Treiberkonfiguration, Homing, Diagnose, Entities
  tmc22xx_stepgen.h/.cpp GPTimer-ISR, Positionszähler, Trigger-Eingang (Endschalter/DIAG)
  tmc22xx_ramp.h         Integer-Rampenplaner (plattformunabhängig, host-testbar)
  tmc22xx_uart.h/.cpp    UART-Datagramme, CRC8, Echo-Behandlung, IFCNT-Prüfung, Retries
  tmc22xx_registers.h    Registeradressen und Bitmasken
  tmc22xx_button.h, tmc22xx_number.h
examples/                drei vollständige YAMLs
tests/test_ramp.cpp      Host-Test für Rampenplaner und CRC
```

## Einbindung

```yaml
external_components:
  - source:
      type: local
      path: components            # Pfad relativ zur YAML
  # oder:
  # - source: github://ShadyMCShadesen/esphome-tmc22xx@main
  #   components: [tmc22xx]
```

## Konfiguration

Alle Positionen, Wege, Geschwindigkeiten und Beschleunigungen sind in **Einheiten** angegeben.
Ohne `steps_per_unit` ist eine Einheit ein (Mikro-)Schritt; mit `steps_per_unit: 80` z. B. Millimeter.

| Schlüssel | Modus | Default | Bedeutung |
|---|---|---|---|
| `driver` | beide | `tmc2208` | `tmc2208` oder `tmc2209` |
| `mode` | beide | `uart` | `uart` oder `pins` |
| `step_pin`, `dir_pin` | beide | Pflicht | interne GPIOs |
| `enable_pin` | beide | – | EN ist low-aktiv: `inverted: true` setzen |
| `ms1_pin`, `ms2_pin`, `microsteps` | pins | – | TMC2208: 2/4/8/16, TMC2209: 8/16/32/64 |
| `max_speed` | beide | Pflicht | Einheiten/s (max. 100 000 Schritte/s) |
| `acceleration` / `deceleration` | beide | 1000 / = acceleration | Einheiten/s², Verhältnis max. 1:256 |
| `steps_per_unit` | beide | 1 | Schritte je Einheit |
| `min_position`, `max_position` | beide | – | Soft-Limits |
| `invert_direction` | beide | false | UART: GCONF.shaft, pins: DIR-Pegel |
| `enable_on_boot` | beide | true | |
| `idle_disable_delay` | beide | – | Endstufe nach Stillstandszeit abschalten |
| `report_interval` | beide | 500ms | Positionssensor während der Fahrt |
| `uart_id`, `address` | uart | – / 0 | Adresse 0–3 nur TMC2209 |
| `rsense` | uart | 110mOhm | Messwiderstand des Moduls |
| `run_current`, `hold_current` | uart | 500mA / halber Laufstrom | Effektivwerte |
| `standstill_delay` | uart | 2s | TPOWERDOWN bis zur Absenkung auf `hold_current` |
| `microsteps`, `interpolation` | uart | 16 / true | MRES, intpol |
| `chopper` | uart | `stealthchop` | oder `spreadcycle` |
| `stealthchop_threshold` | uart | – | darüber SpreadCycle (TPWMTHRS) |
| `poll_interval` | uart | 2s | Diagnoseabfrage (GSTAT, DRV_STATUS) |
| `diag_pin`, `stallguard_threshold`, `stop_on_stall` | uart, TMC2209 | – | StallGuard4 |
| `homing:` | beide | – | siehe unten |

`homing:` – `source` (`endstop`/`stallguard`), `endstop_pin` (NO/NC über `inverted`), `direction`
(`negative`/`positive`), `speed`, `slow_speed` (optional, zweite Anfahrt), `backoff`, `max_travel`,
`timeout` (60s), `debounce` (1ms), `home_position` (Positionswert am Schaltpunkt, 0),
`park_position` (optional, Anfahrt nach dem Homing), `on_boot`.

Ablauf: schnelle Suche → (bei `slow_speed`: Freifahren um `backoff`, langsame Anfahrt) → Position :=
`home_position` → Freifahren um `backoff` → optional `park_position`. Die ISR stoppt innerhalb eines
Schritts nach Ablauf der Entprellzeit. Fehler (Timeout, kein Trigger innerhalb `max_travel`, Schalter
nach dem Freifahren noch aktiv, Treiber nicht erreichbar) lösen `on_homing_failed` aus.

### Actions, Conditions, Trigger

| Action | Parameter (alle templatable) |
|---|---|
| `tmc22xx.move_to` | `position` |
| `tmc22xx.move_by` | `distance` |
| `tmc22xx.run` | `direction` (`forward`/`backward` oder Lambda → bool), optional `speed` |
| `tmc22xx.stop`, `tmc22xx.emergency_stop`, `tmc22xx.home`, `tmc22xx.enable`, `tmc22xx.disable` | – |
| `tmc22xx.set_position` | `position` |
| `tmc22xx.set_speed` | `max_speed`, `acceleration`, `deceleration` |
| `tmc22xx.set_current` | `run_current`, `hold_current` (Lambda liefert mA) |
| `tmc22xx.set_microsteps` | `microsteps` |
| `tmc22xx.set_stallguard_threshold` | `threshold` (TMC2209) |

Conditions: `tmc22xx.is_moving`, `tmc22xx.is_homed`, `tmc22xx.is_homing`.
Trigger: `on_move_complete`, `on_homing_complete`, `on_homing_failed`, `on_stall` (TMC2209),
`on_driver_error` (`x` = Fehlerbits: Bit 1 ot, 2/3 s2ga/s2gb, 4/5 s2vsa/s2vsb, 24 uv_cp, 25 UART-Ausfall).
In Lambdas: `id(motor).get_position()`, `get_target()`, `is_moving()`, `get_driver_errors()`.

`set_speed` und `run: speed:` ändern die Maximalgeschwindigkeit dauerhaft. `set_microsteps` skaliert
den Positionszähler nicht um.

## Verdrahtung ESP32 ↔ TMC2208-Modul

| TMC2208 | ESP32 | Hinweis |
|---|---|---|
| VM, GND | 4,75–36 V Motorversorgung | 100 µF Elko direkt am Modul; Motor nie unter Spannung stecken |
| VIO, GND | 3V3, GND | gemeinsame Masse mit VM |
| STEP | GPIO25 | |
| DIR | GPIO26 | |
| EN | GPIO27 | low = Endstufe aktiv → `inverted: true`; alternativ fest auf GND |
| PDN_UART | GPIO16 (RX) direkt, GPIO17 (TX) über **1 kΩ** | Single-Wire; je nach Modul Lötbrücke für UART schließen |
| MS1, MS2 | optional GPIO (nur `mode: pins`) | TMC2208 haben interne Pull-downs (offen = 1/8) |
| DIAG / INDEX | – | beim TMC2208 nur Treiberfehler, kein Stall-Signal |
| Endschalter | GPIO32 gegen GND | interner Pull-up, `inverted: true` (Schließer) |

TMC2209 zusätzlich: DIAG → Eingangs-GPIO (z. B. GPIO34); MS1/MS2 legen die UART-Adresse fest
(MS1 = Bit 0, MS2 = Bit 1) und müssen im UART-Betrieb fest verdrahtet sein. Mehrere TMC2209 teilen
sich einen Bus (TX über 1 kΩ, RX direkt auf die gemeinsame PDN_UART-Leitung). Ein TMC2208 hat keine
Adresse und braucht je Treiber einen eigenen UART; der ESP32 hat drei (C3: zwei) Hardware-UARTs,
einer davon wird meist vom Logger belegt.

**Strom:** Im UART-Modus setzt die Komponente `I_scale_analog = 0`; das Vref-Poti ist dann wirkungslos
und der Strom ergibt sich aus `run_current`/`hold_current`:
`I_RMS = (CS+1)/32 · V_FS/(R_sense + 20 mΩ) / √2`, V_FS = 325 mV (vsense = 0) bzw. 180 mV (vsense = 1).
Im Pin-Modus gilt das Poti; für Module mit 110 mΩ: `I_RMS ≈ 0,71 · Vref`. Die Stillstandsabsenkung
übernimmt dort der Chip, solange PDN_UART auf GND liegt.

## Inbetriebnahme / Testplan

1. **Ohne Motor, Logger auf DEBUG.** Erwartet beim Start:
   `IOIN=0x20…… (version 0x20, …)` (TMC2209: `0x21`) und
   `Driver configured (IRUN=…, IHOLD=…, vsense=…)`.
   Jeder Schreibzugriff wird über IFCNT geprüft; `dump_config` zeigt CRC-Fehler und Timeouts.
   - `Driver not responding on UART`: VM fehlt (die Logik des Chips wird aus VM versorgt), 1-kΩ-Widerstand,
     RX/TX vertauscht, UART-Lötbrücke, falsche Adresse. Die Komponente versucht es bei jedem Poll erneut.
   - `IOIN reports version 0x21, expected 0x20`: falscher `driver`.
   - `Driver reset detected, restoring configuration`: VM war kurz weg, Register wurden neu geschrieben.
2. **Stromtest:** Motor anschließen, `run_current` niedrig beginnen (z. B. 400 mA). Sensor `actual_current`
   zeigt bei Fahrt den Lauf-, nach `standstill_delay` den Haltestrom. `tmc22xx.set_current` zur Laufzeit testen.
3. **Rampentest:** `tmc22xx.move_by` mit einer vollen Umdrehung (200 · Mikroschritte), Log: `Move to …`,
   `Move complete at …`. Weg mit Markierung an der Welle prüfen, dann `max_speed`/`acceleration` steigern,
   bis Schritte verloren gehen, und 30 % Reserve lassen. `stop` (Rampe) und `emergency_stop` vergleichen.
4. **Homing:** Endschalter zuerst von Hand betätigen und `tmc22xx.home` auslösen (kleines `max_travel`).
   Log: `Homing started`, `Homing phase 1/2/3/4`, `Homing complete, position …`. Fehlerfälle provozieren:
   Schalter abziehen → `Homing failed: no trigger within max_travel`.
5. **TMC2209 sensorlos:** Sensor `stallguard_result` bei Homing-Geschwindigkeit im Leerlauf ablesen;
   `stallguard_threshold` ≈ ein Viertel dieses Werts (Stall bei `SG_RESULT ≤ 2 · SGTHRS`).
6. **Fehlerpfad:** UART im Betrieb abziehen → Status-Warnung, `driver_status = no_uart`, `on_driver_error`
   mit Bit 25; nach dem Wiederanstecken wird der Treiber automatisch neu konfiguriert.

Host-Test des Planers: `g++ -std=c++17 -O2 -Icomponents/tmc22xx tests/test_ramp.cpp -o test_ramp && ./test_ramp`

## Grenzen

- Nur ESP32-Familie. Ein GPTimer je Treiber (ESP32/S3: 4, C3: 2).
- Schrittfrequenz hart auf 100 kHz begrenzt. Jeder Schritt ist ein Interrupt (geschätzt 3–5 µs auf einem
  ESP32 mit 240 MHz); praxistauglich sind etwa 40–50 kHz, auf einem C3 etwa 25–30 kHz. Diese Werte sind
  Schätzungen und nicht an Hardware gemessen. Kritische Abschnitte des WLAN-Stacks verzögern einzelne
  Impulse um einige 10 µs.
- Trapezrampe, keine S-Kurve. Geschwindigkeit mindestens 1 Schritt/s, Beschleunigung mindestens 1 Schritt/s².
- UART-Zugriffe sind synchron: ein Poll kostet etwa 2–3 ms Loop-Zeit, bei ausgefallenem Treiber bis etwa
  20 ms. Die Schritterzeugung läuft davon unabhängig. Während des Homings wird nicht gepollt.
- Baudrate frei wählbar (Treiber erkennt sie automatisch); empfohlen 115200.
- Position wird nicht über einen Neustart gespeichert; nach einem Stall bleibt `homed` gesetzt.
- Open-Load-Flags (ola/olb) erscheinen nur im Text-Sensor, sie sind bei Stillstand und hoher
  Geschwindigkeit nicht aussagekräftig.
- StallGuard wird nur über den DIAG-Pin ausgewertet (bei konstanter Geschwindigkeit), nicht über
  zyklisches Lesen von SG_RESULT. CoolStep wird nicht konfiguriert.
- Arduino-Framework: nur ESP-IDF-APIs verwendet, hier aber nicht gebaut.

Mögliche Erweiterungen: S-Kurve, Encoder-Rückführung (PCNT) zur Schrittverlust-Erkennung, Position in
NVS sichern, `dedge` (Schritt auf beiden Flanken) für doppelte Schrittfrequenz, CoolStep, Endschalter
als Sicherheitsstopp im Normalbetrieb, Cover-Plattform.

# Trajectory Data Logging für FFNN Training

## Übersicht

Das `data_logger.py` Modul ermöglicht das Aufzeichnen von Finger-Trajektorien während der Grip-Ausführung, um Trainingsdaten für das FFNN (Feed-Forward Neural Network) zu sammeln.

## Aktivierung

### Via ROS2 Launch Parameter

```python
# launch/launch.py
Node(
    package="motors",
    executable="hand_controller",
    parameters=[
        {"dev": True},
        {"enable_logging": True},                    # Aktiviert Logging
        {"log_path": "/data/pib_hand_trajectories"}  # Optional: Custom Pfad
    ],
)
```

### Default-Verhalten

- **Logging deaktiviert** per Default
- **Default-Pfad:** `/tmp/pib_hand_trajectories/`
- Dateien werden automatisch pro Grip erstellt

## CSV-Format

Jede Grip-Ausführung erzeugt eine CSV-Datei:

**Spalten:**

| Spalte | Typ | Einheit | Beschreibung |
|--------|-----|---------|--------------|
| `timestamp` | float | seconds | Relative Zeit seit Grip-Start (ms-Präzision) |
| `axis_id` | string | - | Finger-Name (z.B. "daumen", "zeigefinger") |
| `q_cmd` | float | TF units | Kommandierte Position (nach Admittance) |
| `dq_cmd` | float | TF units/s | Geschwindigkeit (berechnet aus Position-Diff) |
| `direction` | int | - | Bewegungsrichtung: 1 (schließen), -1 (öffnen), 0 (stopp) |
| `measured_current` | int | mA | Gemessener Motorstrom |


## Verwendung

### Im Controller (automatisch)

Wenn `enable_logging=True`, wird automatisch geloggt:

```python
# Bei jedem Grip-Start
logger.start_recording(grip_name)

# Bei jedem 50Hz Tick (für jeden Finger)
logger.log_point(
    axis_id="daumen",
    q_cmd=state.current_cmd_pos,
    dq_cmd=velocity,
    direction=direction,
    measured_current=measured_current
)

# Bei Grip-Ende
logger.stop_recording()
```


## Implementierungs-Details

### Buffering

- **Buffer-Größe:** 100 Datenpunkte (default)
- **Schreiben:** Nur wenn Buffer voll (reduziert I/O)
- **Flush:** Automatisch bei `stop_recording()`

### Velocity-Berechnung

```python
dq_cmd = (q_cmd_current - q_cmd_previous) / dt
```

- Benötigt Historie der letzten Position + Timestamp
- Erste Messung: `dq_cmd = 0.0` (keine Historie)

### Direction-Berechnung

```python
direction = calculate_direction(target, current)
# 1 wenn target > current (schließen)
# -1 wenn target < current (öffnen)
# 0 wenn |target - current| < tolerance (stopp)
```


## Troubleshooting

### Keine Daten im CSV

**Check 1:** Logging aktiviert?
```bash
ros2 param get /hand_controller enable_logging
# Sollte: true
```

**Check 2:** Log-Pfad existiert?
```bash
ls -la /tmp/pib_hand_trajectories/
```

**Check 3:** Permissions?
```bash
chmod 777 /tmp/pib_hand_trajectories/
```

## Performance

**Overhead pro 50Hz Tick:**
- Velocity-Berechnung: ~0.01ms
- Buffering: ~0.001ms
- Disk-Write (nur bei vollem Buffer): ~1-5ms


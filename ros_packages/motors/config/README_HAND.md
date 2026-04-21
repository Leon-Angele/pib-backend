# PIB Hand Controller - Dokumentation

## Übersicht

Der **Hand Controller** ist ein ROS2 Action Server für intelligente Grip-Steuerung mit **Admittance Control** (Nachgiebigkeits-Regelung). Er läuft als vierter Node im `motors` Package und ermöglicht die Ausführung vordefinierter Griffe mit automatischer, weicher Compliance.

**Schlüssel-Features:**
- ✅ **Grip-basierte Steuerung**: 7 vordefinierte Griffe (OPEN, SPITZGRIFF, DREIPUNKTGRIFF, ...)
- ✅ **Admittance Control**: Weiche Nachgiebigkeit statt hartem Stopp bei Widerstand
- ✅ **50Hz Control Loop**: Präzise Trajektorien-Interpolation mit Compliance-Detection
- ✅ **Action Server**: Async Grip-Ausführung mit Progress-Feedback
- ✅ **FFNN-Ready**: Vorbereitet für ML-basierte Schwellenwert-Vorhersage

---

## Architektur

### ROS2 Nodes

```
motors Container:
├── motor_control      (Node 1: JointTrajectory Service → Motor Positions)
├── motor_current      (Node 2: Motor Current Publisher, 4Hz)
├── relay_control      (Node 3: Relay Management)
└── hand_controller    (Node 4: Grip Execution mit Admittance Control)
```

### Admittance Control Prinzip

Der `hand_controller` nutzt eine **Trennung von Reference und Command Position**:

```
Bei jedem 50Hz Tick:
1. q_ref schreitet Richtung Ziel (ideale Trajektorie)
2. Messe aktuellen Motorstrom
3. Berechne q_cmd = q_ref - K_a * (current - threshold)
   → Bei Überstrom: q_cmd weicht zurück (Finger wird "weich")
   → Kein Überstrom: q_cmd = q_ref (normale Bewegung)
4. Sende q_cmd an Motor
5. Wenn q_ref am Ziel: Achse fertig
```

**Vorteile gegenüber hartem Compliance-Stopp:**
- Finger geben bei Widerstand nach statt zu stoppen
- Automatische Fortsetzung wenn Widerstand nachlässt
- Flüssigere Reaktion durch 50Hz Loop
- Weniger Vibrationen

---

## Build & Deploy

### 1. Docker Build

```bash
cd /home/leona/pib-backend

# Build motors Container (baut datatypes + motors packages)
docker-compose build ros-motors

# Erwartete Ausgabe:
# Summary: 2 packages finished [~17s]
#   datatypes
#   motors
```

**Was wird gebaut:**
- `datatypes` Package: ExecuteGrip.action wird zu Python/C++ Interfaces kompiliert
- `motors` Package: hand_controller + hand_core.py werden installiert, hand_config.yaml wird nach share/ kopiert

### 2. Start Container

```bash
# Starten (automatisch alle 4 Nodes)
docker-compose up -d ros-motors

# Logs prüfen
docker-compose logs --tail=50 ros-motors

# Erwartete Ausgabe:
# [motor_control-1] ... Now Running MOTOR_CONTROL
# [motor_current-2] ... Now Running MOTOR CURRENT
# [relay_control-3] ... Now Running RELAY_CONTROL
# [hand_controller-4] ... Hand Controller started [namespace=hand, rate=50Hz, axes=6, grips=7]
```

### 3. Dev-Modus vs. Production

**Dev-Modus** (Windows, ohne Hardware):
- In `launch/launch.py` ist `dev=True` gesetzt
- Hand Controller loggt: `[WARN] Running in DEV mode - motor access may be limited`
- Keine echten Motorbewegungen, nur Logik-Test

**Production** (auf Roboter mit Tinkerforge):
- In `launch/launch.py` `dev=True` zu `dev=False` ändern
- Hand Controller greift direkt auf `pib_motors.motor` zu
- Echte Hardware-Bewegungen mit Current-Messung

---

## ROS2 Interface

### Action Server: `/hand/execute_grip`

**Type:** `datatypes/action/ExecuteGrip`

**Action Definition:**
```
# Goal - Request to execute a predefined grip
string grip_name

---

# Result - Final outcome of the grip execution
bool success
string message

---

# Feedback - Continuous updates during grip execution
string current_axis
float32 progress_percent
int32[] motor_currents
```

### Verfügbare Griffe (grip_name)

Definiert in `hand_config.yaml`:

| Grip Name | Beschreibung | Finger-Positionen |
|-----------|--------------|-------------------|
| `OPEN` | Hand öffnen (alle Finger gestreckt) | -9000 (stretched) |
| `SPITZGRIFF` | Pinch Grip (Daumen + Zeigefinger) | Daumen -2000, Zeigefinger -2000 |
| `DREIPUNKTGRIFF` | Three-point Grip | Daumen -2000, Zeigefinger -2000, Mittelfinger -2000 |
| `SCHLUESSELGRIFF` | Key Grip | Daumen -3000, Zeigefinger -6000 |
| `ZYLINDERGRIFF` | Cylinder/Power Grip (alle Finger) | Alle -1000 (closed) |
| `HAKENGRIFF` | Hook Grip (ohne Daumen) | Finger -1000, Daumen -9000 |
| `SPHAERISCHER_GRIFF` | Spherical Grip | Alle -3000 (mittel) |

**Position Range:** `-9000` (gestreckt/offen) bis `9000` (geschlossen)

### Beispiel-Nutzung

**1. Action Liste anzeigen:**
```bash
docker exec -it multirepo-ros-motors-1 bash -c \
  "cd /app/ros2_ws && source install/setup.bash && ros2 action list"

# Ausgabe:
# /hand/execute_grip
```

**2. Grip ausführen (mit Feedback):**
```bash
docker exec -it multirepo-ros-motors-1 bash -c \
  "cd /app/ros2_ws && source install/setup.bash && \
   ros2 action send_goal /hand/execute_grip datatypes/action/ExecuteGrip \
   '{grip_name: \"SPITZGRIFF\"}' --feedback"
```

**Feedback während Ausführung:**
```
Feedback:
    current_axis: daumen
    progress_percent: 25.5
    motor_currents: [120, 135, 0, 0, 0, 450]
```

**3. Aus Python-Code (ROS2 Client):**
```python
import rclpy
from rclpy.action import ActionClient
from datatypes.action import ExecuteGrip

class HandClient:
    def __init__(self):
        self.node = rclpy.create_node('hand_client')
        self._action_client = ActionClient(
            self.node, 
            ExecuteGrip, 
            '/hand/execute_grip'
        )
    
    def send_grip(self, grip_name):
        goal_msg = ExecuteGrip.Goal()
        goal_msg.grip_name = grip_name
        
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        return future
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"Achse: {feedback.current_axis}, "
              f"Progress: {feedback.progress_percent:.1f}%, "
              f"Currents: {feedback.motor_currents}")

# Nutzung:
client = HandClient()
future = client.send_grip("SPITZGRIFF")
```

---

## Konfiguration

### hand_config.yaml

**Pfad:** `ros_packages/motors/config/hand_config.yaml`

**Struktur:**

```yaml
axes:
  daumen:
    motor_name: "thumb_right_stretch"
    min_pos: -9000
    max_pos: 9000
    max_speed: 800        # Tinkerforge units/second
    max_current: 1500     # mA
  # ... weitere Achsen

grips:
  SPITZGRIFF:
    daumen: -2000
    zeigefinger: -2000
    mittelfinger: -9000
    # ... weitere Finger
```

**Wichtige Parameter:**

- **motor_name**: Name in `pib_motors.motor.name_to_motors` Dictionary
- **min_pos/max_pos**: Position Range (-9000 bis 9000 Tinkerforge units)
- **max_speed**: Maximale Geschwindigkeit für Trajektorien-Interpolation
- **max_current**: Schwellenwert für Admittance Control (70% davon wird genutzt)

### Admittance Parameter

**In `hand_core.py`:**

```python
CONTROL_LOOP_RATE_HZ = 50   # Control loop frequency
ADMITTANCE_GAIN = 1.5       # K_a: Units yielded per mA excess current
```

**In `hand_controller.py`:**

```python
COMPLIANCE_THRESHOLD = 0.7  # 70% of max_current triggers admittance
```

**Tuning-Tipps:**
- **ADMITTANCE_GAIN** erhöhen → Finger weicher, mehr Nachgiebigkeit
- **ADMITTANCE_GAIN** verringern → Finger härter, weniger Nachgiebigkeit
- **COMPLIANCE_THRESHOLD** erhöhen → Admittance startet später (höherer Strom erlaubt)
- **COMPLIANCE_THRESHOLD** verringern → Admittance startet früher (sensibler)

---

## Funktion der Handsteuerung

### Control Loop (50Hz)

Der `hand_controller` führt für jede Achse folgende Schritte aus:

**1. Initialisierung (beim Action Goal):**
```python
# Für jede Achse (daumen, zeigefinger, ...):
- Lese current_position vom Motor
- Setze reference_position = current_position (q_ref)
- Setze current_cmd_pos = current_position (q_cmd)
- Setze target_position aus Grip-Config
```

**2. Control Loop Tick (50Hz):**
```python
for axis in axes:
    # Schritt 1: Update Reference (ideale Trajektorie)
    q_ref = calculate_next_reference(q_ref, target, max_speed)
    
    # Schritt 2: Messe Strom
    current_measured = motor.get_current()
    
    # Schritt 3: Admittance Logic
    threshold = max_current * 0.7
    excess = max(0, current_measured - threshold)
    q_cmd = q_ref - (1.5 * excess)  # Weicht zurück bei Überstrom
    
    # Schritt 4: Sende Position
    motor.set_position(q_cmd)
    
    # Schritt 5: Check ob fertig
    if q_ref erreicht target:
        axis.is_finished = True
```

**3. Completion:**
```python
if all axes finished:
    return Result(success=True, message="Grip completed")
```

### Beispiel-Ablauf: SPITZGRIFF

```
t=0ms:   Goal empfangen: SPITZGRIFF
         daumen: current=-9000 → target=-2000
         zeigefinger: current=-9000 → target=-2000
         
t=20ms:  [50Hz Tick 1]
         daumen: q_ref=-9000+16=-8984, current=0mA, q_cmd=-8984
         zeigefinger: q_ref=-9000+16=-8984, current=0mA, q_cmd=-8984
         
t=40ms:  [50Hz Tick 2]
         daumen: q_ref=-8968, current=0mA, q_cmd=-8968
         ...
         
t=500ms: Daumen berührt Objekt
         daumen: q_ref=-5000, current=1200mA (>1050mA threshold!)
         excess = 1200-1050 = 150
         q_cmd = -5000 - (1.5*150) = -5225  ← weicht zurück!
         
t=520ms: Objekt gibt nach
         daumen: q_ref=-4800, current=800mA (<threshold)
         q_cmd = -4800  ← normale Bewegung
         
t=8000ms: Beide Finger am Ziel
         daumen: q_ref=-2000 (fertig), q_cmd=-2100 (leicht zurück wegen Objekt)
         zeigefinger: q_ref=-2000 (fertig), q_cmd=-2000
         → Result: success=True
```

---

## Integration mit bestehenden Systemen

### Mit motor_control Node

`hand_controller` greift **direkt** auf `pib_motors.motor` zu, **nicht** über den `ApplyJointTrajectory` Service. Das ermöglicht:
- Präzise 50Hz Kontrolle ohne Service-Overhead
- Direkte Current-Messung für Admittance
- Keine Interferenz zwischen verschiedenen Motion-Controllern

### Mit motor_current Node

`motor_current` published weiterhin alle Motorströme auf `/motor_current` Topic (4Hz). Das ist **unabhängig** vom `hand_controller`, der seine eigenen Current-Messungen macht (50Hz).

### Mit cerebra/rosbridge

Aus der Web-UI kann der Hand Controller über rosbridge angesprochen werden:

```javascript
// JavaScript Client (cerebra)
const client = new ROSLIB.ActionClient({
    ros: ros,
    serverName: '/hand/execute_grip',
    actionName: 'datatypes/action/ExecuteGrip'
});

const goal = new ROSLIB.Goal({
    actionClient: client,
    goalMessage: { grip_name: 'SPITZGRIFF' }
});

goal.on('feedback', (feedback) => {
    console.log('Progress:', feedback.progress_percent);
});

goal.on('result', (result) => {
    console.log('Success:', result.success);
});

goal.send();
```

---

## Troubleshooting

### Container startet nicht

```bash
# Check Logs
docker-compose logs ros-motors

# Häufige Fehler:
# - YAML Syntax Error in hand_config.yaml
# - Missing datatypes package (ExecuteGrip.action nicht gebaut)
```

### Hand Controller startet nicht

```bash
# Check ob ExecuteGrip.action existiert
docker exec -it multirepo-ros-motors-1 bash -c \
  "cd /app/ros2_ws && source install/setup.bash && \
   ros2 interface show datatypes/action/ExecuteGrip"

# Sollte Action Definition anzeigen
```

### Grip wird nicht accepted

```bash
# Check verfügbare Griffe
docker exec -it multirepo-ros-motors-1 bash -c \
  "cd /app/ros2_ws && cat install/motors/share/motors/config/hand_config.yaml | grep -A 1 'grips:'"

# Grip Name muss exakt matchen (case-sensitive!)
```

### Motor Currents sind -1

Das bedeutet: Motor nicht verbunden oder dev-Modus.

- **Dev-Modus**: Normal, `motor.get_current()` gibt `NO_CURRENT=-1` zurück
- **Production**: Check Tinkerforge Verbindung, Motor-Namen in PIB-API

### Finger bewegen sich nicht weiter nach Widerstand

Das ist das **gewünschte Verhalten** mit Admittance Control:
- `q_ref` läuft weiter zum Ziel
- `q_cmd` bleibt zurück wegen hohem Strom
- Wenn Widerstand nachlässt → `q_cmd` folgt `q_ref` wieder

Grip ist **completed** wenn `q_ref` am Ziel ist, auch wenn `q_cmd` durch Admittance weiter zurück ist.

---

## FFNN Integration (Future)

Der Code ist vorbereitet für ML-basierte Schwellenwert-Vorhersage:

**Aktuell (statisch):**
```python
threshold = max_current * 0.7  # Fixer Schwellenwert
```

**Zukünftig (FFNN):**
```python
# Predict expected current based on position and velocity
expected_current = ffnn_model.predict(q_ref, velocity, finger_id)
threshold = expected_current + safety_margin

# Only currents above prediction trigger admittance
excess = max(0, current_measured - threshold)
```

**Vorteile:**
- Dynamische Schwellenwerte je nach Bewegungsphase
- Weniger False-Positives (Admittance bei normaler Bewegung)
- Bessere Objekterkennung (unerwarteter Strom = Kontakt)

---

## Zusammenfassung

**Was passiert bei einem Grip-Request:**

1. **Action Goal** wird gesendet mit `grip_name`
2. **hand_controller** liest Ziel-Positionen aus `hand_config.yaml`
3. **50Hz Loop** startet für alle 6 Achsen:
   - Berechne nächsten `q_ref` Schritt
   - Messe Motor-Strom
   - Wende Admittance an: `q_cmd = q_ref - K_a * excess_current`
   - Sende `q_cmd` an Hardware
4. **Feedback** wird kontinuierlich gesendet (current_axis, progress, currents)
5. **Result** wird zurückgegeben wenn alle `q_ref` am Ziel

**Bei Widerstand:**
- Finger wird "weich" (q_cmd weicht zurück)
- Trajektorie läuft weiter (q_ref → Ziel)
- Automatische Fortsetzung wenn Widerstand nachlässt

**Vorteile:**
- ✅ Sichere, schonende Griffe
- ✅ Robustheit gegen unerwartete Objekte
- ✅ Keine "stuck" States
- ✅ Einfache High-Level API (nur Grip-Name nötig)

---

**Autoren:** PIB Robotics Team  
**Letzte Aktualisierung:** 2026-04-22  
**Version:** 1.0 (Admittance Control)

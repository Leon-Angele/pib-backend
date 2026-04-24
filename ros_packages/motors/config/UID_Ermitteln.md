
# Bricklet-UIDs ermitteln und zuweisen

Kurz: So findest du die physikalischen Tinkerforge-UIDs, weist sie in der API den Brick‑Nummern zu (1–3) und lässt die `ros-motors` neu laden.

1) Flask-API starten (damit die DB/Endpoints erreichbar sind)

```bash
# aus Repo-Root
docker compose up -d flask-app
```

2) aktuelle Bricklet-Einträge in der API ansehen

```bash
curl http://localhost:5000/bricklet
curl http://localhost:5000/bricklet/1
```

3) physische UIDs auslesen

- Option A — Brick Viewer (GUI): öffnet alle angeschlossenen Bricklets und zeigt ihre UIDs.
- Option B — kurzes Python-Enumerate-Skript (anpassbar):

```python
from tinkerforge.ip_connection import IPConnection
from tinkerforge.ip_connection import IPConnection as IPC
import time

def cb(uid, connected_uid, position, hw_ver, fw_ver, device_identifier, enumeration_type):
		print(f"UID={uid}, device_identifier={device_identifier}, enum_type={enumeration_type}")

ipcon = IPConnection()
ipcon.register_callback(IPC.CALLBACK_ENUMERATE, cb)
ipcon.connect('localhost', 4223)  # ggf. Host/Port anpassen
ipcon.enumerate()
time.sleep(1)
ipcon.disconnect()
```

Hinweis: Wenn du Docker verwendest, führe das Script auf dem Host aus oder im Container mit Zugang zum Tinkerforge-IPConnection (siehe `TINKERFORGE_HOST` in `docker-compose.yaml`).

4) UID der Brick‑Nummer zuweisen (per API)

```bash
curl -X PUT -H "Content-Type: application/json" \
	-d '{"uid":"DEINE_UID_HIER"}' \
	http://localhost:5000/bricklet/1
```

5) Alternative: interaktives Update-Skript

```bash
python3 scripts/update-servo-ids.py
```

Das Skript liest `pib_api/flask/pibdata.db` und schreibt die `uid`-Felder in der Tabelle `bricklet`.

6) `ros-motors` neu starten, damit die UIDs eingelesen werden

```bash
docker compose restart ros-motors
docker compose logs --tail=200 ros-motors
```

In den Logs solltest du Meldungen wie
`Servo Bricklet <UID> is connected.`
sehen — das bestätigt, dass `pib_motors/pib_motors/bricklet.py` die UID verwendet und das `uid_to_servo_bricklet`-Mapping aufgebaut wurde.




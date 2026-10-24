"""Watch PX4 SITL vehicles on the GCS link during the link-loss test, then land them.

    python scripts/px4/link_loss_watch.py [N_VEHICLES] [SECONDS]

Listens on UDP 14550 (the link PX4 SITL sends to a ground station) as its own GCS
(system id 246), so it does not share a link with the bridge. Prints a line with
wall-clock time (time.time()) whenever a vehicle's flight mode or arming state changes,
a one-line snapshot per second, and every PX4 status text. After SECONDS it lands every
armed vehicle and waits until all are disarmed. Run it next to
`utm-px4-demo headon2 --stop-after S` (docs/integration/PX4_SETUP.md, "Link loss").
"""

import math
import sys
import time

from mavsdk import Configuration, Mavsdk
from mavsdk.plugins.action import Action
from mavsdk.plugins.telemetry import Telemetry

n = int(sys.argv[1]) if len(sys.argv) > 1 else 2
duration_s = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0

mav = Mavsdk(Configuration.create_manual(246, 190, True))
mav.add_any_connection("udpin://0.0.0.0:14550")
t0 = time.monotonic()
while len({s.get_system_id() for s in mav.get_systems() if s.has_autopilot()}) < n:
    if time.monotonic() - t0 > 60:
        sys.exit(f"found {sorted(s.get_system_id() for s in mav.get_systems())}, need {n} autopilots")
    time.sleep(0.5)
systems = {s.get_system_id(): s for s in mav.get_systems() if s.has_autopilot()}
tel = {sid: Telemetry(s) for sid, s in systems.items()}
for sid, t in tel.items():
    t.subscribe_status_text(lambda st, sid: print(f"{time.time():.3f} sys{sid} text {st.type.name}: {st.text}",
                                                  flush=True), sid)

last, next_snap = {}, time.monotonic()
end = time.monotonic() + duration_s
while time.monotonic() < end:
    snap = time.monotonic() >= next_snap
    for sid, t in sorted(tel.items()):
        state = (t.flight_mode().name, t.armed())
        if state != last.get(sid) or snap:
            pv = t.position_velocity_ned()
            hspeed = math.hypot(pv.velocity.north_m_s, pv.velocity.east_m_s)
            tag = "change" if state != last.get(sid) else "snap"
            print(f"{time.time():.3f} sys{sid} {tag} mode={state[0]} armed={state[1]} "
                  f"height={-pv.position.down_m:.2f} m hspeed={hspeed:.2f} m/s", flush=True)
            last[sid] = state
    if snap:
        next_snap += 1.0
    time.sleep(0.05)

print(f"{time.time():.3f} watch done: landing every armed vehicle", flush=True)
for sid, s in systems.items():
    if tel[sid].armed():
        Action(s).land()
t1 = time.monotonic()
while any(t.armed() for t in tel.values()):
    if time.monotonic() - t1 > 120:
        sys.exit("vehicles still armed 120 s after land")
    time.sleep(0.5)
print(f"{time.time():.3f} all disarmed", flush=True)

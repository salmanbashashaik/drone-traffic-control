"""Setup check: connect to PX4 SITL instance 0, take off to 10 m, hover, land.

Run inside WSL with the venv from docs/integration/PX4_SETUP.md, after
`scripts/px4/sitl.sh start "0,0"`. Exits non-zero if any step fails.
"""

import sys
import time

from mavsdk import ComponentType, Configuration, Mavsdk
from mavsdk.exceptions import MavsdkError
from mavsdk.plugins.action import Action
from mavsdk.plugins.telemetry import Telemetry


def wait(cond, timeout_s, what):
    t0 = time.monotonic()
    while not cond():
        if time.monotonic() - t0 > timeout_s:
            sys.exit(f"timeout after {timeout_s} s waiting for {what}")
        time.sleep(0.2)
    print(f"[{time.monotonic() - T0:5.1f} s] {what}")


T0 = time.monotonic()
mav = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
mav.add_any_connection("udpin://0.0.0.0:14540")
drone = mav.first_autopilot(timeout_s=20.0)
if drone is None:
    sys.exit("no autopilot on udp 14540")
tel, act = Telemetry(drone), Action(drone)
wait(tel.health_all_ok, 60, "health checks ok")
act.set_takeoff_altitude(10.0)
for attempt in range(20):  # PX4 denies arming until it has seen our GCS heartbeat for a moment
    try:
        act.arm()
        break
    except MavsdkError as exc:
        print(f"arm attempt {attempt + 1}: {exc}")
        time.sleep(1)
else:
    sys.exit("arming denied for 20 s")
act.takeoff()
wait(lambda: -tel.position_velocity_ned().position.down_m > 9.5, 30, "above 9.5 m")
time.sleep(3)
p = tel.position_velocity_ned().position
print(f"hover NED: {p.north_m:.2f} {p.east_m:.2f} {p.down_m:.2f}")
act.land()
wait(lambda: not tel.in_air(), 60, "landed")
wait(lambda: not tel.armed(), 30, "disarmed")
print("takeoff check passed")

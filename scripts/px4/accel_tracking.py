"""Measure how well PX4 offboard setpoints realise a commanded acceleration (one SITL drone).

Flies the same 10 Hz profile with each setpoint mapping: +3.5 m/s^2 east for 2 s,
-3.5 m/s^2 for 2 s, then hold zero velocity. Prints, per mapping and phase, the
mean achieved acceleration over the whole 2 s and over its last 1 s (the
difference is the response lag), from PX4's velocity estimate.

    scripts/px4/sitl.sh start "0,0"
    python scripts/px4/accel_tracking.py [--set MPC_ACC_HOR=5.0 ...]

The numbers decide the mapping in px4_bridge.py (docs/integration/PX4_SETUP.md, "Setpoint mapping").
"""

import argparse
import math
import sys
import time

from mavsdk import ComponentType, Configuration, Mavsdk
from mavsdk.exceptions import MavsdkError
from mavsdk.plugins.action import Action
from mavsdk.plugins.offboard import AccelerationNed, Offboard, PositionNedYaw, VelocityNedYaw
from mavsdk.plugins.param import Param
from mavsdk.plugins.telemetry import Telemetry

DT = 0.1
A = 3.5
PARAMS = ["MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC", "MPC_ACC_HOR", "MPC_ACC_HOR_MAX", "MPC_XY_VEL_MAX",
          "MPC_TILTMAX_AIR", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN", "COM_OF_LOSS_T"]
MAPPINGS = ("velocity", "velocity_integrated", "velocity+accel_ff")


def send(off, mapping, v_meas, v_ref, a_east):
    """One setpoint; velocities are (north, east, down). Returns the commanded east velocity."""
    if mapping == "velocity_integrated":   # v_ref[k+1] = v_ref[k] + a dt, independent of the measurement
        ve = v_ref + a_east * DT
    else:                                  # v_cmd = v_meas + a dt
        ve = v_meas[1] + a_east * DT
    v = VelocityNedYaw(v_meas[0] if mapping != "velocity_integrated" else 0.0, ve, 0.0, 0.0)
    if mapping == "velocity+accel_ff":     # plus a as acceleration feedforward; NaN position = not controlled
        off.set_position_velocity_acceleration_ned(PositionNedYaw(math.nan, math.nan, math.nan, 0.0), v,
                                                   AccelerationNed(0.0, a_east, 0.0))
    else:
        off.set_velocity_ned(v)
    return ve


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="set a float PX4 parameter first")
    args = ap.parse_args()
    mav = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    mav.add_any_connection("udpin://0.0.0.0:14540")
    drone = mav.first_autopilot(timeout_s=20.0)
    if drone is None:
        sys.exit("no autopilot on udp 14540")
    tel, act, off, par = Telemetry(drone), Action(drone), Offboard(drone), Param(drone)
    tel.set_rate_position_velocity_ned(50.0)
    for item in args.set:
        name, value = item.split("=")
        par.set_param_float(name, float(value))
    print({p: round(par.get_param_float(p), 3) for p in PARAMS})
    act.set_takeoff_altitude(15.0)
    for _ in range(20):
        try:
            act.arm()
            break
        except MavsdkError:
            time.sleep(1)
    else:
        sys.exit("arming denied for 20 s")
    act.takeoff()
    while -tel.position_velocity_ned().position.down_m < 14.5:
        time.sleep(0.2)
    time.sleep(2)

    def vel():
        v = tel.position_velocity_ned().velocity
        return v.north_m_s, v.east_m_s, v.down_m_s

    off.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    off.start()
    for mapping in MAPPINGS:
        for _ in range(40):                       # settle at zero velocity
            off.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            time.sleep(DT)
        out = []
        v_ref = vel()[1]
        for a_east in (A, -A):
            samples = []
            nxt = time.monotonic()
            for _ in range(int(2.0 / DT)):
                samples.append((time.monotonic(), vel()[1]))
                v_ref = send(off, mapping, vel(), v_ref, a_east)
                nxt += DT
                time.sleep(max(0.0, nxt - time.monotonic()))
            samples.append((time.monotonic(), vel()[1]))
            whole = (samples[-1][1] - samples[0][1]) / (samples[-1][0] - samples[0][0])
            half = len(samples) // 2
            last = (samples[-1][1] - samples[half][1]) / (samples[-1][0] - samples[half][0])
            out.append(f"{a_east:+.1f}: whole {whole:+.2f} ({100 * whole / a_east:.0f}%), last 1 s {last:+.2f} "
                       f"({100 * last / a_east:.0f}%)")
        print(f"{mapping:20s} " + " | ".join(out))
    for _ in range(40):
        off.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        time.sleep(DT)
    off.stop()
    act.land()
    while tel.in_air():
        time.sleep(0.5)


if __name__ == "__main__":
    main()

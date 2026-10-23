"""Real-world operating conditions, from the sources in P3-SA-03 §2.

Every number here is a published value or a conversion of one; the section of
P3-SA-03 that cites it is given next to it. Accuracy figures are converted to
a per-axis standard deviation with the factors of P3-SA-03 §2.1:

- CEP (2-D, 50 %):          sigma = CEP / 1.1774
- R95 (2-D, 95 %):          sigma = R95 / 2.4477
- 1-D 95 %:                 sigma = V95 / 1.9600
- 1-D 50 %:                 sigma = value / 0.6745
"""

from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.80665
FT = 0.3048
KT = 0.514444

# --- GNSS (P3-SA-03 §2.1) -------------------------------------------------


@dataclass(frozen=True)
class Gnss:
    name: str
    sigma_h_m: float    # per horizontal axis
    sigma_v_m: float    # vertical
    sigma_vel_mps: float  # per axis
    source: str


def _cep(x: float) -> float:
    return x / 1.1774


def _r95(x: float) -> float:
    return x / 2.4477


def _v95(x: float) -> float:
    return x / 1.96


def _p50(x: float) -> float:
    return x / 0.6745


# ZED-F9P RTK 0.01 m + 1 ppm CEP (vertical R50 likewise) at a 10 km baseline.
RTK = Gnss("rtk_f9p", _cep(0.02), _p50(0.02), _p50(0.05), "u-blox ZED-F9P-02B datasheet R03")
# GAGAN APV-1 measured 95 %: horizontal about 3.0 m, vertical about 4.0 m (ISRO, ICG-11 2016).
# Velocity: GPS SPS PS 2020, 0.2 m/s 95 % on any axis.
SBAS = Gnss("sbas_gagan", _r95(3.0), _v95(4.0), _v95(0.2), "ISRO ICG-11 2016; GPS SPS PS 2020")
# GPS SPS PS 2020: a well-designed receiver gives 3 m horizontal and 5 m vertical, 95 %.
SPS = Gnss("gps_sps", _r95(3.0), _v95(5.0), _v95(0.2), "GPS SPS PS 5th ed. 2020")
# GPS SPS PS 2020 worst-site standard: 15 m horizontal, 33 m vertical, 95 %.
SPS_WORST = Gnss("gps_sps_worst_site", _r95(15.0), _v95(33.0), _v95(0.2), "GPS SPS PS 5th ed. 2020")

# --- Cellular link (P3-SA-03 §2.2) -----------------------------------------

# Barrios-Munoz et al. 2024 (arXiv 2411.09666v2), RTT at 60 m: median 59.91 ms,
# mean 66.41 ms. One way = RTT / 2 (symmetric link assumed). A log-normal with
# that median and mean has sigma_ln = sqrt(2 ln(mean / median)).
LTE_AIR_ONE_WAY_MEDIAN_S = 0.05991 / 2.0
LTE_AIR_SIGMA_LN = math.sqrt(2.0 * math.log(66.41 / 59.91))
# 3GPP TS 22.125 Table 7.2-1: 99.9 % reliability for command and control.
LINK_LOSS_3GPP = 1e-3
# Risk per check, tied to the same 99.9 % (P3-SA-03 §3.2, founder choice).
P_RISK = 1e-3

# --- Wind (P3-SA-03 §2.3) --------------------------------------------------

# CEEW: 100 m mean wind over India; only Gujarat and Tamil Nadu average above 15 km/h.
INDIA_MEAN_100M_MPS = 15.0 / 3.6
# MIL-F-8785C low-altitude Dryden, W20 (wind at 20 ft) for light / moderate / severe turbulence.
W20_LIGHT_MPS = 15 * KT
W20_MODERATE_MPS = 30 * KT
W20_SEVERE_MPS = 45 * KT
# WMO squall: a rise of at least 8 m/s to at least 11 m/s, lasting at least 1 min.
SQUALL_RISE_MPS = 8.0
SQUALL_PEAK_MPS = 11.0


def dryden(w20_mps: float, h_m: float) -> tuple[float, float, float, float]:
    """MIL-F-8785C low-altitude Dryden: (sigma_u, sigma_w, L_u, L_w), SI units.

    sigma_w = 0.1 W20; sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4;
    L_u = L_v = h / (0.177 + 0.000823 h)^1.2; L_w = h; h in ft in the formulas.
    """
    h_ft = max(h_m / FT, 10.0)
    base = 0.177 + 0.000823 * h_ft
    sigma_w = 0.1 * w20_mps
    return sigma_w / base ** 0.4, sigma_w, h_ft / base ** 1.2 * FT, h_ft * FT


# --- Airframe and autopilot (P3-SA-03 §2.4) --------------------------------

# Faessler et al. 2018: mass-normalised linear rotor drag, largest identified
# coefficient (d_x, circle trajectory), 0.610 kg quadrotor. Used on all axes.
DRAG_PER_S = 0.544
# PX4 MPC_TILTMAX_AIR default 45 deg: horizontal acceleration up to g tan 45.
# Per component (inscribed box of the horizontal disc): g tan 45 / sqrt 2.
A_PHYS_MPS2 = G * math.tan(math.radians(45.0)) / math.sqrt(2.0)
# PX4 COM_OF_LOSS_T default: offboard loss timeout before the lost action.
OFFBOARD_LOSS_S = 1.0
# Actuator (attitude) time constant: the former PX4 MC_ROLL_TC range was
# 0.15-0.25 s (QGroundControl metadata). The upper end is used.
TAU_ACT_S = 0.25

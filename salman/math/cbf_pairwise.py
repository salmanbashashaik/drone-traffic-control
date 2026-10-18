"""
salman/math/cbf_pairwise.py
Ticket: S-01 — Define pairwise CBF safety constraint.

Barrier function:  h_ij(x) = ||p_i - p_j||² - d_safe²
Safe set:          {x : h_ij(x) >= 0}  (drones further apart than d_safe)

No CVXPY here — pure NumPy. The QP solver lives in salman/cbf/qp_solver.py (S-04).
"""
import numpy as np


def h_ij(pos_i: np.ndarray, pos_j: np.ndarray, d_safe: float) -> float:
    """
    Pairwise barrier function value.

    Args:
        pos_i: position of drone i, shape (3,)
        pos_j: position of drone j, shape (3,)
        d_safe: minimum safe separation distance (meters)

    Returns:
        h >= 0 means safe; h < 0 means in collision zone.
    """
    delta = pos_i - pos_j
    return float(np.dot(delta, delta) - d_safe ** 2)


def grad_h_ij(pos_i: np.ndarray, pos_j: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Gradient of h_ij w.r.t. [pos_i, pos_j].

    Returns:
        (dh_dpi, dh_dpj) — each shape (3,)
        dh/dp_i =  2*(p_i - p_j)
        dh/dp_j = -2*(p_i - p_j)
    """
    delta = pos_i - pos_j
    dh_dpi = 2.0 * delta
    dh_dpj = -2.0 * delta
    return dh_dpi, dh_dpj


def lf_h(pos_i: np.ndarray, pos_j: np.ndarray,
         vel_i: np.ndarray, vel_j: np.ndarray) -> float:
    """
    Lie derivative of h along f (drift term).
    For double integrator dynamics (ẋ = v), Lf(h) = ∇h · [v_i, v_j].

    This is the first-order term in the CBF condition:
        Lf(h) + Lg(h)·u + α(h) >= 0
    """
    dh_dpi, dh_dpj = grad_h_ij(pos_i, pos_j)
    return float(np.dot(dh_dpi, vel_i) + np.dot(dh_dpj, vel_j))


def lg_h(pos_i: np.ndarray, pos_j: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Lie derivative of h along g (control input channels).
    For ẍ = u (acceleration control), g = I so Lg(h)·u = ∇h_p · u.

    Returns:
        (Lg_h_i, Lg_h_j) — each shape (3,), the CBF constraint row for drone i and j.
    """
    dh_dpi, dh_dpj = grad_h_ij(pos_i, pos_j)
    return dh_dpi, dh_dpj


# ---------------------------------------------------------------------------
# Quick self-test — run with: python -m salman.math.cbf_pairwise
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    d_safe = 5.0  # meters

    # Two drones: safe configuration
    p1 = np.array([0.0, 0.0, 50.0])
    p2 = np.array([10.0, 0.0, 50.0])
    v1 = np.array([1.0, 0.0, 0.0])
    v2 = np.array([-1.0, 0.0, 0.0])  # approaching each other

    h = h_ij(p1, p2, d_safe)
    lf = lf_h(p1, p2, v1, v2)
    lg_i, lg_j = lg_h(p1, p2)

    print(f"h_ij       = {h:.3f}  (> 0 means safe, d=10m, d_safe=5m)")
    print(f"Lf_h       = {lf:.3f}  (negative = barrier shrinking = approaching)")
    print(f"Lg_h_drone1 = {lg_i}")
    print(f"Lg_h_drone2 = {lg_j}")

    # Unsafe configuration (too close)
    p3 = np.array([3.0, 0.0, 50.0])
    h_unsafe = h_ij(p1, p3, d_safe)
    print(f"\nh_ij (unsafe, d=3m) = {h_unsafe:.3f}  (should be < 0)")

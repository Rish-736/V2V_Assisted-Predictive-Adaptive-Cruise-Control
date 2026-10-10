"""
statespace.py -- state-space model of the ACC loop, and LQR design.

WHY A STATE-SPACE MODEL AT ALL
------------------------------
The cascaded PID in the firmware was designed loop-by-loop in the frequency
domain: an inner speed loop, then an outer gap loop slow enough not to
interact with it. That works, and the bandwidth-separation check in
design_control.py is what makes it legitimate.

But it is two SISO designs stapled together. It cannot express the fact that
spacing error and follower speed are coupled states of one system, it cannot
trade gap-holding against control effort in a principled way, and the "keep
the loops 5x apart" rule is a constraint we imposed on ourselves, not
something the physics demanded.

The state-space formulation treats it as one 3-state MIMO-capable problem and
lets LQR choose the gains by minimising a cost we actually care about:

    J = integral ( q_e*e^2 + q_v*v_f^2 + q_i*(integral e)^2 + r*u^2 ) dt

One design, optimal by construction for that cost, with guaranteed stability
margins (an LQR state-feedback loop has >=60 degrees of phase margin and
infinite gain margin at the plant input).

THE MODEL
---------
States, with d the gap, v_f the follower speed and v_l the lead speed:

    x1 = e   = d - d0 - Th*v_f     spacing error (constant time-gap policy)
    x2 = v_r = v_l - v_f           RELATIVE velocity
    x3 = integral of e             added for zero steady-state error

Input u is normalised motor duty. The lead speed v_l is an exogenous
disturbance, NOT a state -- which matters, see `controllability()`.

WHY RELATIVE VELOCITY AND NOT THE FOLLOWER SPEED
------------------------------------------------
This is the one modelling choice that decides whether the LQR design means
anything. LQR minimises a quadratic in the STATES, so it drives the states
to zero. If x2 were the follower speed v_f, the cost would contain q_v*v_f^2
and the optimiser would read that as "standing still is ideal" -- it would
trade away gap-holding to go slower, because that is literally what it was
asked to do.

At equilibrium the follower matches the lead, so v_f -> v_l, which is NOT
zero. Relative velocity is: v_r -> 0. With x = [e, v_r, integral e] every
state is zero in steady state, so this is a genuine regulator problem and
the quadratic cost expresses what we actually want.

Derivation, using v_f = v_l - x2:
    d/dt(d)   = v_l - v_f = x2
    d/dt(v_f) = (K*u - v_f)/tau                     (first-order motor)

    d/dt(e)   = x2 - Th*d/dt(v_f)
              = (1 - Th/tau)*x2 + (Th/tau)*v_l - (Th*K/tau)*u
    d/dt(v_r) = a_l - d/dt(v_f)
              = -(1/tau)*x2 + (1/tau)*v_l - (K/tau)*u + a_l

so

        | 0   1 - Th/tau   0 |        | -Th*K/tau |        | Th/tau |
    A = | 0     -1/tau     0 |    B = |  -K/tau   |    E = | 1/tau  |
        | 1       0        0 |        |     0     |        |   0    |

WHY NO OBSERVER / KALMAN FILTER IS NEEDED -- AND WHAT THAT OWES TO V2V
----------------------------------------------------------------------
Every state is directly available on this vehicle: v_f from the encoder,
v_l from the V2V packet, hence v_r by subtraction; e from the gap and v_f;
and the integral is accumulated in software. So this is genuine full-state
feedback, not estimated-state feedback.

That is not a lucky accident, it is the V2V link paying off a second time.
A sensor-only ACC does not measure v_l at all. It would have to reconstruct
v_r by differentiating a noisy range signal, or build an observer to
estimate it -- adding estimator lag on top of filter lag, in precisely the
situation where reaction time is what matters.

`observability()` below checks the gap-only case to show what we would have
been forced into. We are not forced into it, and claiming a Kalman filter
we do not need would be dishonest.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .plant import PlantParams

__all__ = [
    "ACCStateSpace", "LQRResult", "solve_care", "lqr", "expm",
    "controllability_matrix", "observability_matrix", "matrix_rank",
]


# =====================================================================
#  Small linear-algebra helpers.
#  numpy is required; scipy is used when present but never assumed, so
#  this module works on a machine with only numpy installed.
# =====================================================================
def _np():
    import numpy as np
    return np


def matrix_rank(M, tol: float | None = None) -> int:
    np = _np()
    return int(np.linalg.matrix_rank(np.asarray(M, dtype=float), tol=tol))


def controllability_matrix(A, B):
    """[B, AB, A^2 B, ...] -- full rank means every mode can be driven."""
    np = _np()
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(A.shape[0], -1)
    cols = [B]
    for _ in range(A.shape[0] - 1):
        cols.append(A @ cols[-1])
    return np.hstack(cols)


def observability_matrix(A, C):
    """[C; CA; CA^2; ...] -- full rank means every state is inferable."""
    np = _np()
    A = np.asarray(A, dtype=float)
    C = np.asarray(C, dtype=float).reshape(-1, A.shape[0])
    rows = [C]
    for _ in range(A.shape[0] - 1):
        rows.append(rows[-1] @ A)
    return np.vstack(rows)


def expm(M):
    """Matrix exponential. scipy if available, else scaling-and-squaring."""
    np = _np()
    M = np.asarray(M, dtype=float)
    try:
        from scipy.linalg import expm as _sp_expm  # type: ignore
        return _sp_expm(M)
    except ImportError:
        pass
    # scaling and squaring with a Taylor series on the scaled matrix
    norm = float(np.max(np.sum(np.abs(M), axis=1))) if M.size else 0.0
    s = max(0, int(math.ceil(math.log2(norm))) + 1) if norm > 0.5 else 0
    A = M / (2.0 ** s)
    E = np.eye(A.shape[0])
    term = np.eye(A.shape[0])
    for k in range(1, 20):
        term = term @ A / k
        E = E + term
        if np.max(np.abs(term)) < 1e-18:
            break
    for _ in range(s):
        E = E @ E
    return E


def solve_care(A, B, Q, R):
    """Solve the continuous-time algebraic Riccati equation

        A'P + PA - P B R^-1 B' P + Q = 0

    scipy if present; otherwise via the Hamiltonian matrix, taking the
    invariant subspace spanned by the stable eigenvectors. The Hamiltonian
    route is the textbook method and needs only numpy, which keeps this
    runnable on a bare install.
    """
    np = _np()
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(A.shape[0], -1)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float).reshape(B.shape[1], B.shape[1])

    try:
        from scipy.linalg import solve_continuous_are  # type: ignore
        return solve_continuous_are(A, B, Q, R)
    except ImportError:
        pass

    n = A.shape[0]
    Rinv = np.linalg.inv(R)
    H = np.block([[A, -B @ Rinv @ B.T],
                  [-Q, -A.T]])
    w, v = np.linalg.eig(H)
    idx = [i for i in range(2 * n) if w[i].real < 0]
    if len(idx) != n:
        raise ValueError("CARE: could not split a stable invariant subspace "
                         "(is (A,B) stabilisable and Q >= 0?)")
    V = v[:, idx]
    X1, X2 = V[:n, :], V[n:, :]
    P = np.real(X2 @ np.linalg.inv(X1))
    return 0.5 * (P + P.T)          # symmetrise away round-off


@dataclass
class LQRResult:
    K: object                 # 1 x n gain, u = -K x
    P: object                 # Riccati solution
    poles: object             # closed-loop eigenvalues
    Q: object
    R: object

    @property
    def gains(self) -> list[float]:
        return [float(x) for x in _np().asarray(self.K).ravel()]

    def damping(self) -> list[tuple[float, float, float]]:
        """(|pole|, zeta, wn) for each closed-loop pole."""
        out = []
        for p in _np().asarray(self.poles).ravel():
            wn = abs(p)
            zeta = -p.real / wn if wn > 1e-12 else 1.0
            out.append((wn, float(zeta), float(wn)))
        return out

    def settling_time(self) -> float:
        """2% settling time estimated from the slowest pole."""
        np = _np()
        re = [abs(p.real) for p in np.asarray(self.poles).ravel() if p.real < -1e-9]
        return 4.0 / min(re) if re else float("inf")


def lqr(A, B, Q, R) -> LQRResult:
    """Continuous-time LQR. Returns K with u = -K x."""
    np = _np()
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(A.shape[0], -1)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float).reshape(B.shape[1], B.shape[1])

    P = solve_care(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    poles = np.linalg.eigvals(A - B @ K)
    return LQRResult(K=K, P=P, poles=poles, Q=Q, R=R)


# =====================================================================
#  The ACC state-space model
# =====================================================================
@dataclass
class ACCStateSpace:
    """Constant-time-gap ACC as a 3-state linear system.

    States  x = [ e, v_r, integral(e) ]   with v_r = v_lead - v_follow
    Input   u = normalised motor duty
    Disturbance w = v_lead
    """

    plant: PlantParams = field(default_factory=PlantParams)
    t_gap: float = 1.50
    d_standstill: float = 0.20
    _A: object = field(init=False, default=None)
    _B: object = field(init=False, default=None)
    _E: object = field(init=False, default=None)

    def __post_init__(self) -> None:
        np = _np()
        K, tau, Th = self.plant.K, self.plant.tau, self.t_gap
        self._A = np.array([
            [0.0, 1.0 - Th / tau, 0.0],
            [0.0, -1.0 / tau,     0.0],
            [1.0, 0.0,            0.0],
        ])
        self._B = np.array([[-Th * K / tau],
                            [-K / tau],
                            [0.0]])
        self._E = np.array([[Th / tau], [1.0 / tau], [0.0]])

    # ----- matrices ---------------------------------------------------
    @property
    def A(self):
        return self._A

    @property
    def B(self):
        return self._B

    @property
    def E(self):
        return self._E

    @property
    def n(self) -> int:
        return 3

    def C_full(self):
        """Everything measured: e from the gap, v_r from the encoder and the
        V2V packet, the integral accumulated in software."""
        return _np().eye(3)

    def C_gap_only(self):
        """Pathological case: only the gap is measured. Used to show WHY an
        observer would be needed if we had not instrumented the vehicle."""
        return _np().array([[1.0, 0.0, 0.0]])

    # ----- structural properties --------------------------------------
    def open_loop_poles(self):
        return _np().linalg.eigvals(self._A)

    def controllability(self) -> dict:
        Co = controllability_matrix(self._A, self._B)
        r = matrix_rank(Co)
        return {"matrix": Co, "rank": r, "n": self.n, "controllable": r == self.n}

    def observability(self, C=None) -> dict:
        C = self.C_full() if C is None else C
        Ob = observability_matrix(self._A, C)
        r = matrix_rank(Ob)
        return {"matrix": Ob, "rank": r, "n": self.n, "observable": r == self.n}

    def controllability_with_lead_as_state(self) -> dict:
        """The 4-state model that ALSO carries v_lead as a state.

        This is uncontrollable, necessarily and informatively: no amount of
        follower throttle changes what the lead vehicle is doing. The
        uncontrollable mode is exactly v_lead. That is why v_lead belongs in
        the model as a DISTURBANCE and not as a state -- and it is a good
        thing to be able to show, because it is the kind of modelling slip
        that silently produces a meaningless LQR design.
        """
        np = _np()
        K, tau, Th = self.plant.K, self.plant.tau, self.t_gap
        A4 = np.array([
            [0.0, 1.0 - Th / tau, 0.0, Th / tau],
            [0.0, -1.0 / tau,     0.0, 1.0 / tau],
            [1.0, 0.0,            0.0, 0.0],
            [0.0, 0.0,            0.0, 0.0],   # v_lead evolves on its own
        ])
        B4 = np.array([[-Th * K / tau], [-K / tau], [0.0], [0.0]])
        Co = controllability_matrix(A4, B4)
        r = matrix_rank(Co)
        return {"rank": r, "n": 4, "controllable": r == 4}

    # ----- design -----------------------------------------------------
    def bryson_weights(self, e_max: float = 0.10, vr_max: float = 0.15,
                       i_max: float = 0.30, u_max: float = 1.0):
        """Bryson's rule: weight each term by 1/(max acceptable value)^2.

        This turns weight selection from guesswork into a statement of
        requirements -- "I will tolerate 10 cm of spacing error, 0.15 m/s of
        speed mismatch, and full duty" -- which is far easier to defend in a
        viva than a hand-tuned Q.

        Raising e_max buys a gentler, more comfortable response; lowering it
        buys tighter gap-holding at the cost of more aggressive duty.
        """
        np = _np()
        Q = np.diag([1.0 / e_max ** 2, 1.0 / vr_max ** 2, 1.0 / i_max ** 2])
        R = np.array([[1.0 / u_max ** 2]])
        return Q, R

    def design(self, Q=None, R=None, **bryson) -> LQRResult:
        if Q is None or R is None:
            Q, R = self.bryson_weights(**bryson)
        return lqr(self._A, self._B, Q, R)

    # ----- discretisation --------------------------------------------
    def discretise(self, dt: float, with_disturbance: bool = False):
        """Zero-order-hold discretisation, via the block-matrix trick:

            expm([[A, B], [0, 0]] * dt) = [[Ad, Bd], [0, I]]

        Needed because the gains are implemented on a 50 Hz ESP32 loop, not
        in continuous time.

        With `with_disturbance`, B and E are stacked and discretised
        together and (Ad, Bd, Ed) is returned. Discretising E properly
        matters: E = [Th/tau, 1/tau, 0]^T is NOT small, and approximating
        Ed as E*dt (or worse, forgetting it) injects the lead speed into
        the wrong states and silently produces a simulation that settles
        at the wrong speed.
        """
        np = _np()
        n, m = self._A.shape[0], self._B.shape[1]
        Bin = np.hstack([self._B, self._E]) if with_disturbance else self._B
        k = Bin.shape[1]
        M = np.zeros((n + k, n + k))
        M[:n, :n] = self._A
        M[:n, n:] = Bin
        Md = expm(M * dt)
        Ad, Bd_all = Md[:n, :n], Md[:n, n:]
        if with_disturbance:
            return Ad, Bd_all[:, :m], Bd_all[:, m:]
        return Ad, Bd_all

    # ----- simulation --------------------------------------------------
    def simulate(self, K, v_lead_fn, duration: float = 45.0, dt: float = 0.02,
                 x0=None, u_min: float = -1.0, u_max: float = 1.0,
                 nonlinear: bool = True):
        """Closed-loop simulation under u = -Kx.

        With nonlinear=True the duty is saturated and the integrator stops
        charging while saturated (anti-windup) -- the same discipline the
        firmware PID uses. LQR theory is linear; the hardware is not, and a
        comparison that ignores saturation flatters LQR unfairly.
        """
        np = _np()
        K = np.asarray(K, dtype=float).reshape(1, -1)
        Ad, Bd, Ed = self.discretise(dt, with_disturbance=True)

        x = np.zeros((3, 1)) if x0 is None else np.asarray(x0, float).reshape(3, 1)
        ts, es, vrs, vfs, us, gaps, vls = [], [], [], [], [], [], []

        n = int(duration / dt)
        for k in range(n):
            t = k * dt
            v_l = float(v_lead_fn(t))

            u = float(-(K @ x)[0, 0])
            u_sat = min(max(u, u_min), u_max) if nonlinear else u

            e_k = float(x[0, 0])
            vr_k = float(x[1, 0])
            vf_k = v_l - vr_k               # follower speed, recovered

            ts.append(t)
            es.append(e_k)
            vrs.append(vr_k)
            vfs.append(vf_k)
            us.append(u_sat)
            vls.append(v_l)
            gaps.append(e_k + self.d_standstill + self.t_gap * vf_k)

            x_next = Ad @ x + Bd * u_sat + Ed * v_l
            if nonlinear and u_sat != u:
                x_next[2, 0] = x[2, 0]          # anti-windup: freeze integral
            if nonlinear and (v_l - float(x_next[1, 0])) < 0.0:
                x_next[1, 0] = v_l              # follower cannot reverse
            x = x_next

        return {"t": ts, "e": es, "v_r": vrs, "v_f": vfs,
                "u": us, "gap": gaps, "v_lead": vls}

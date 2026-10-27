#!/usr/bin/env python3
"""Constrained, nominal 1-D linear MPC. All quantities are upward-positive SI.

State x=[height, velocity]; input u=requested net acceleration (gravity excluded).
Prediction acceleration is K*u+b, with K=1,b=0 for the original model.
Predictions assume piecewise-constant acceleration over each dt. The real
vehicle still has actuator/estimator dynamics, so predicted bounds are not
physical guarantees. No integral action or disturbance observer is included.
"""
from dataclasses import asdict, dataclass
import time

import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import solve_discrete_are


@dataclass(frozen=True)
class MPCConfig:
    dt: float = 0.1
    horizon: int = 30
    height_weight: float = 12.0
    velocity_weight: float = 3.0
    acceleration_weight: float = 0.2
    delta_acceleration_weight: float = 0.8
    velocity_slack_weight: float = 1000.0
    max_velocity: float = 1.5
    min_acceleration: float = -2.0
    max_acceleration: float = 2.0
    max_jerk: float = 5.0
    acceleration_gain: float = 1.0
    acceleration_bias: float = 0.0


class AltitudeMPC:
    def __init__(self, config=None):
        self.config = c = config or MPCConfig()
        if c.dt <= 0 or c.horizon < 2 or c.max_velocity <= 0 or c.max_jerk <= 0:
            raise ValueError('Invalid MPC timing or bounds')
        if not c.min_acceleration < 0 < c.max_acceleration:
            raise ValueError('Acceleration bounds must straddle zero')
        if min(c.height_weight, c.velocity_weight, c.acceleration_weight, c.velocity_slack_weight) <= 0 or c.delta_acceleration_weight < 0:
            raise ValueError('Invalid MPC cost weights')
        if not np.isfinite([c.acceleration_gain, c.acceleration_bias]).all() or c.acceleration_gain <= 0:
            raise ValueError('Acceleration gain must be positive and model coefficients finite')
        A = np.array([[1., c.dt], [0., 1.]])
        kinematic_B = np.array([[0.5 * c.dt**2], [c.dt]])
        B = kinematic_B * c.acceleration_gain
        self.equilibrium_u = -c.acceleration_bias / c.acceleration_gain
        if not c.min_acceleration < self.equilibrium_u < c.max_acceleration:
            raise ValueError('Model requires a hover command outside input bounds')
        n = c.horizon
        # Stack x_1 ... x_N = F x_0 + G [u_0 ... u_(N-1)].
        self.F = np.vstack([np.linalg.matrix_power(A, k + 1) for k in range(n)])
        self.G = np.zeros((2*n, n))
        for k in range(n):
            for j in range(k + 1):
                self.G[2*k:2*k+2, j:j+1] = np.linalg.matrix_power(A, k-j) @ B
        # Exact zero-order-hold drift for constant affine acceleration bias.
        self.drift = np.concatenate([
            np.array([.5*((k+1)*c.dt)**2, (k+1)*c.dt]) * c.acceleration_bias
            for k in range(n)])
        Q = np.diag([c.height_weight, c.velocity_weight])
        # Standard LQR terminal cost for the two-state model; the move penalty
        # is additional shaping, not a formal constrained stability proof.
        terminal = solve_discrete_are(A, B, Q, [[c.acceleration_weight]])
        self.Qbar = sparse.block_diag([sparse.csc_matrix(Q)] * (n-1) + [sparse.csc_matrix(terminal)]).toarray()
        self.D = np.eye(n) - np.eye(n, k=-1)
        hessian = (self.G.T @ self.Qbar @ self.G + c.acceleration_weight * np.eye(n)
                   + c.delta_acceleration_weight * self.D.T @ self.D)
        # Decision [U, S], with nonnegative per-step speed slack S. A noisy
        # measured speed or model mismatch must not make recovery infeasible.
        # Acceleration and move bounds remain hard; speed excess is penalized.
        I, Z = np.eye(n), np.zeros((n, n))
        self.C = np.block([[I, Z], [self.G[1::2], -I], [-self.G[1::2], -I],
                           [self.D, Z], [Z, I]])
        hessian = sparse.block_diag([sparse.csc_matrix(2*hessian),
                                      sparse.eye(n)*2*c.velocity_slack_weight], format='csc')
        self.solver = osqp.OSQP()
        self.solver.setup(P=sparse.triu(hessian, format='csc'), q=np.zeros(2*n),
                          A=sparse.csc_matrix(self.C), l=np.full(5*n, -np.inf),
                          u=np.full(5*n, np.inf), verbose=False, eps_abs=1e-6,
                          eps_rel=1e-6, max_iter=10000, polishing=False,
                          time_limit=0.04)
        self.previous_u = 0.0
        self.solution = None

    def solve(self, height, velocity, target):
        started = time.perf_counter()
        c = self.config
        if not np.isfinite([height, velocity, target]).all():
            raise ValueError('MPC state/reference must be finite')
        free = self.F @ np.array([height, velocity]) + self.drift
        reference = np.tile([target, 0.0], c.horizon)
        previous = np.zeros(c.horizon)
        previous[0] = self.previous_u
        q = (2 * self.G.T @ self.Qbar @ (free - reference)
             - 2 * c.delta_acceleration_weight * self.D.T @ previous
             - 2 * c.acceleration_weight * self.equilibrium_u * np.ones(c.horizon))
        move_limit = c.max_jerk * c.dt
        n = c.horizon
        lower = np.concatenate([np.full(n, c.min_acceleration), np.full(2*n, -np.inf),
                                -move_limit + previous, np.zeros(n)])
        upper = np.concatenate([np.full(n, c.max_acceleration),
                                c.max_velocity - free[1::2], c.max_velocity + free[1::2],
                                move_limit + previous, np.full(n, np.inf)])
        self.solver.update(q=np.r_[q, np.zeros(n)], l=lower, u=upper)
        if self.solution is not None:
            self.solver.warm_start(x=np.r_[self.solution[1:n], self.solution[n-1],
                                           self.solution[n+1:], self.solution[-1]])
        result = self.solver.solve(raise_error=False)
        if result.info.status_val != 1 or result.x is None:
            raise RuntimeError(f'MPC QP failed: {result.info.status}')
        values = self.C @ result.x
        violation = max(0., float(np.max(lower-values)), float(np.max(values-upper)))
        if not np.isfinite(result.x).all() or violation > 2e-4:
            raise RuntimeError(f'MPC invalid solution; constraint violation {violation}')
        self.solution = result.x.copy()
        self.previous_u = float(result.x[0])
        states = np.vstack([[height, velocity], (free + self.G @ result.x[:n]).reshape(-1, 2)])
        return {'acceleration_m_s2': self.previous_u, 'predicted_states': states.tolist(),
                'planned_accelerations': result.x[:n].tolist(),
                'predicted_speed_slack_m_s': result.x[n:].tolist(),
                'max_speed_slack_m_s': max(0., float(result.x[n:].max())), 'solver_status': result.info.status,
                'iterations': int(result.info.iter), 'constraint_violation': violation,
                'solve_ms': (time.perf_counter()-started)*1000}

    def metadata(self):
        return {'type': 'affine_two_state_linear_mpc' if (self.config.acceleration_gain != 1.0 or self.config.acceleration_bias != 0.0) else 'nominal_linear_mpc', **asdict(self.config),
                'state': ['ground_relative_height_m', 'up_velocity_m_s'],
                'input': 'net_up_acceleration_m_s2', 'disturbance_observer': False,
                'terminal_cost': 'discrete LQR on height and velocity',
                'prediction_equation': 'h_dot=v; v_dot=acceleration_gain*u+acceleration_bias',
                'equilibrium_command_m_s2': self.equilibrium_u,
                'constraints': 'soft predicted speed bound with quadratic slack penalty; hard acceleration and delta-acceleration bounds'}

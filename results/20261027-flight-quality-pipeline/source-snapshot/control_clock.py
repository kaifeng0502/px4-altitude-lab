"""Phase-locked simulation-time deadlines; never execute catch-up bursts."""
import math


class ControlClock:
    def __init__(self, period, early_tolerance=0.):
        if not math.isfinite(period) or period <= 0:
            raise ValueError('Control period must be positive and finite')
        if not math.isfinite(early_tolerance) or not 0 <= early_tolerance < period/2:
            raise ValueError('Early tolerance must be finite and less than half a period')
        self.early_tolerance = early_tolerance
        self.period = period
        self.next_deadline = None
        self.last_seen = None

    def poll(self, simulation_time):
        if not math.isfinite(simulation_time):
            raise ValueError('Simulation time must be finite')
        if self.last_seen is not None and simulation_time < self.last_seen-1e-6:
            raise RuntimeError('Simulation clock moved backwards')
        self.last_seen = simulation_time
        if self.next_deadline is None:
            self.next_deadline = simulation_time
        if simulation_time+self.early_tolerance+1e-6 < self.next_deadline:
            return None
        skipped = max(0, math.floor((simulation_time-self.next_deadline+1e-6)/self.period))
        deadline = self.next_deadline+skipped*self.period
        self.next_deadline = deadline+self.period
        return {'scheduled_boot_s': deadline,
                'deadline_lateness_sim_ms': max(0., (simulation_time-deadline)*1000),
                'deadline_phase_error_sim_ms': (simulation_time-deadline)*1000,
                'skipped_control_ticks': skipped}

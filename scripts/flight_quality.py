"""Ground alignment and a bounded slow height-reference correction.

Neither helper reads simulation truth. The correction is an outer integral
around MPC, not a claim of offset-free MPC or estimator-bias correction.
"""
import math
import statistics


def ground_quality(samples, now, window=10.):
    data=[s for s in samples if s[0]>=now-window and all(math.isfinite(x) for x in s)]
    if len(data)<100 or data[-1][0]-data[0][0]<window-.5:
        return {'ready':False,'sample_count':len(data)}
    times=[x[0] for x in data];zs=[x[1] for x in data];vs=[x[2] for x in data]
    mt,mz=statistics.mean(times),statistics.mean(zs)
    slope=sum((t-mt)*(z-mz) for t,z in zip(times,zs))/sum((t-mt)**2 for t in times)
    std=statistics.pstdev(zs);mean_v=statistics.mean(vs)
    rms_v=math.sqrt(statistics.mean(v*v for v in vs))
    return {'ready':std<=.025 and abs(slope)<=.0025 and abs(mean_v)<=.015 and rms_v<=.035,
            'sample_count':len(data),'height_std_m':std,'height_slope_ned_m_s':slope,
            'mean_ned_velocity_m_s':mean_v,'rms_ned_velocity_m_s':rms_v,'window_s':window}


class HeightBiasCompensator:
    def __init__(self, gain=.15, limit=.12):
        self.gain=gain;self.limit=limit;self.correction=0.
        self.last_time=None;self.target=None;self.active=False

    def update(self, height, velocity, target, now, saturated=False):
        if not all(math.isfinite(v) for v in [height,velocity,target,now]):
            raise ValueError('Compensator state must be finite')
        if self.last_time is not None and now<self.last_time:
            raise RuntimeError('Compensator clock moved backwards')
        dt=0. if self.last_time is None else min(.2,now-self.last_time)
        self.last_time=now
        if self.target is None or abs(target-self.target)>.01:
            self.correction=0.;self.target=target;dt=0.
        error=target-height
        self.active=abs(error)<.15 and abs(velocity)<.15 and not saturated
        if self.active:
            self.correction=max(-self.limit,min(self.limit,self.correction+self.gain*error*dt))
        return self.correction

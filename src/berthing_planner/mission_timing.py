"""Piecewise-linear mission clocks and continuous horizontal-distance checks.

Positions are held before departure, in explicit time gaps, and after finish.
The clocks are geometric: constant segment speed or stationary holds do not
model acceleration, turning, depth, launch/recovery, or communication physics.
"""
from bisect import bisect_right
from dataclasses import dataclass
import math


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def _point(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f'{name} must contain two coordinates')
    return tuple(_finite(v, name) for v in value)


@dataclass(frozen=True)
class _Clock:
    source: dict
    segments: tuple
    starts: tuple
    start: float
    finish: float
    initial: tuple
    final: tuple


def _validated(timeline):
    required = ('segments', 'start_time_s', 'finish_time_s', 'speed_m_s',
                'initial_position', 'final_position', 'length_m')
    if not isinstance(timeline, dict) or any(key not in timeline for key in required):
        raise ValueError('Timeline is missing required clock fields')
    start = _finite(timeline['start_time_s'], 'start_time_s')
    finish = _finite(timeline['finish_time_s'], 'finish_time_s')
    speed = _finite(timeline['speed_m_s'], 'speed_m_s')
    stated_length = _finite(timeline['length_m'], 'length_m')
    if start < 0 or finish < start or speed < 0 or stated_length < 0:
        raise ValueError('Timeline times, speed and length must be nonnegative and ordered')
    initial = _point(timeline['initial_position'], 'initial_position')
    final = _point(timeline['final_position'], 'final_position')
    if not isinstance(timeline['segments'], (list, tuple)):
        raise ValueError('Timeline segments must be a sequence')
    segments, previous, previous_time, length = [], initial, start, 0.0
    for record in timeline['segments']:
        if not isinstance(record, dict) or any(key not in record for key in
                ('t0_s', 't1_s', 'p0', 'p1', 'scanning', 'kind')):
            raise ValueError('Timeline segment is missing required fields')
        t0 = _finite(record['t0_s'], 'segment t0_s')
        t1 = _finite(record['t1_s'], 'segment t1_s')
        p0, p1 = _point(record['p0'], 'segment p0'), _point(record['p1'], 'segment p1')
        if t0 < previous_time or t1 <= t0 or p0 != previous:
            raise ValueError('Segments must have positive duration, ordered times and no teleportation')
        if type(record['scanning']) is not bool or not isinstance(record['kind'], str) or not record['kind']:
            raise ValueError('Segment scanning must be bool and kind a nonempty string')
        distance = math.dist(p0, p1)
        if not math.isfinite(distance) or distance / (t1 - t0) > speed + max(1e-9, speed*1e-8):
            raise ValueError('Segment motion exceeds the declared speed or numeric range')
        segments.append((t0, t1, p0, p1))
        previous, previous_time = p1, t1
        length += distance
    if segments:
        if segments[0][0] != start or segments[-1][1] != finish or previous != final:
            raise ValueError('Timeline header does not match segment boundaries and final position')
    elif start != finish or initial != final:
        raise ValueError('An empty timeline must be stationary at one time and position')
    if not math.isclose(length, stated_length, rel_tol=1e-9, abs_tol=1e-7):
        raise ValueError('Timeline length does not match its geometry')
    return _Clock(timeline, tuple(segments), tuple(s[0] for s in segments),
                  start, finish, initial, final)


def time_parameterize(legs, speed_m_s, start_delay_s=0.0):
    """Assign arc-length time to A2-style ``points/scanning/kind`` legs.

    Nonzero pairs become timed segments; repeated vertices are skipped. A
    wholly zero-length active scan is rejected because no dwell rule is given.
    A stationary mission can use a non-scanning zero-length leg.
    """
    speed = _finite(speed_m_s, 'speed_m_s')
    start = _finite(start_delay_s, 'start_delay_s')
    if speed <= 0 or start < 0:
        raise ValueError('Speed must be positive and departure delay nonnegative')
    if not isinstance(legs, (list, tuple)) or not legs:
        raise ValueError('A mission needs at least one leg')
    segments, first, previous, current_time, total_length = [], None, None, start, 0.0
    for index, leg in enumerate(legs):
        if not isinstance(leg, dict) or any(key not in leg for key in ('points', 'scanning', 'kind')):
            raise ValueError('Each leg needs points, scanning and kind')
        if type(leg['scanning']) is not bool or not isinstance(leg['kind'], str) or not leg['kind']:
            raise ValueError('Leg scanning must be bool and kind a nonempty string')
        if not isinstance(leg['points'], (list, tuple)) or not leg['points']:
            raise ValueError('A leg needs at least one 2D point')
        points = [_point(point, 'leg point') for point in leg['points']]
        if previous is not None and points[0] != previous:
            raise ValueError('Legs must connect without teleportation')
        if first is None:
            first = points[0]
        leg_length = 0.0
        for p0, p1 in zip(points, points[1:]):
            distance = math.dist(p0, p1)
            if not math.isfinite(distance):
                raise ValueError('Leg length exceeds the numeric range')
            if distance == 0:
                continue
            finish = current_time + distance / speed
            if not math.isfinite(finish) or finish <= current_time:
                raise ValueError('Segment duration must be representable and positive')
            segment = {'t0_s': current_time, 't1_s': finish, 'p0': list(p0), 'p1': list(p1),
                       'scanning': leg['scanning'], 'kind': leg['kind'], 'leg_index': index}
            if 'task_id' in leg:
                segment['task_id'] = leg['task_id']
            segments.append(segment)
            current_time = finish
            leg_length += distance
        if leg['scanning'] and leg_length == 0:
            raise ValueError('Zero-length active scan requires an explicit dwell rule')
        total_length += leg_length
        previous = points[-1]
    return {'segments': segments, 'start_time_s': start, 'finish_time_s': current_time,
            'speed_m_s': speed, 'initial_position': list(first), 'final_position': list(previous),
            'length_m': total_length}


def _position(clock, time):
    if time <= clock.start or not clock.segments:
        return clock.initial
    if time >= clock.finish:
        return clock.final
    index = bisect_right(clock.starts, time) - 1
    t0, t1, p0, p1 = clock.segments[index]
    if time >= t1:
        return p1
    fraction = (time - t0) / (t1 - t0)
    return tuple(p0[j] + fraction*(p1[j] - p0[j]) for j in (0, 1))


def position_at(timeline, t):
    """Position at any finite time; before/after motion remains stationary."""
    clock = _validated(timeline)
    return list(_position(clock, _finite(t, 'query time')))


def _extrema(a, b, start, end):
    times = {start, end}
    for clock in (a, b):
        times.update(t for segment in clock.segments for t in segment[:2] if start < t < end)
    times = sorted(times)
    minimum, maximum = math.inf, -math.inf
    minimum_time = maximum_time = start
    for left, right in zip(times, times[1:]):
        a0, b0, a1, b1 = (_position(a, left), _position(b, left),
                          _position(a, right), _position(b, right))
        r0 = tuple(a0[j] - b0[j] for j in (0, 1))
        r1 = tuple(a1[j] - b1[j] for j in (0, 1))
        change = tuple(r1[j] - r0[j] for j in (0, 1))
        squared_change = sum(v*v for v in change)
        fraction = min(1.0, max(0.0, -sum(r0[j]*change[j] for j in (0, 1))/squared_change)) if squared_change else 0.0
        candidates = [(left, math.hypot(*r0)), (right, math.hypot(*r1)),
                      (left + fraction*(right-left),
                       math.hypot(*(r0[j] + fraction*change[j] for j in (0, 1))))]
        for time, distance in candidates:
            if distance < minimum or distance == minimum and time < minimum_time:
                minimum, minimum_time = distance, time
            if distance > maximum or distance == maximum and time < maximum_time:
                maximum, maximum_time = distance, time
    if start == end:
        minimum = maximum = math.dist(_position(a, start), _position(b, start))
    return {'minimum_distance_m': minimum, 'minimum_time_s': minimum_time,
            'maximum_distance_m': maximum, 'maximum_time_s': maximum_time,
            'start_time_s': start, 'end_time_s': end,
            'analytic_intervals_checked': max(0, len(times)-1)}


def pairwise_distance_extrema(a, b, *, start_time_s=0.0, end_time_s=None):
    """Continuous extrema on a closed window, including all implied holds.

    The default window is [0,max finish]. Explicit bounds can inspect a USV
    transit interval; returned bounds make that restricted scope auditable.
    Every boundary and the interior quadratic minimum are considered. The
    maximum of each relative linear interval occurs at a boundary.
    """
    first, second = _validated(a), _validated(b)
    start = _finite(start_time_s, 'start_time_s')
    end = max(start, first.finish, second.finish) if end_time_s is None else _finite(end_time_s, 'end_time_s')
    if start < 0 or end < start:
        raise ValueError('Distance window must satisfy 0 <= start <= end')
    return _extrema(first, second, start, end)


def _shift(clock, delay):
    if not math.isfinite(clock.finish + delay):
        raise ValueError('Shifted mission time exceeds the numeric range')
    result = dict(clock.source)
    result['start_time_s'], result['finish_time_s'] = clock.start + delay, clock.finish + delay
    result['initial_position'], result['final_position'] = list(clock.initial), list(clock.final)
    result['segments'] = [dict(record, t0_s=record['t0_s'] + delay, t1_s=record['t1_s'] + delay,
                               p0=list(record['p0']), p1=list(record['p1']))
                          for record in clock.source['segments']]
    shifted_segments = tuple((t0+delay, t1+delay, p0, p1) for t0, t1, p0, p1 in clock.segments)
    return _Clock(result, shifted_segments, tuple(t+delay for t in clock.starts),
                  clock.start+delay, clock.finish+delay, clock.initial, clock.final)


def shift_timeline(timeline, delay_s):
    """Add a nonnegative delay without changing route or leg metadata."""
    clock = _validated(timeline)
    delay = _finite(delay_s, 'delay_s')
    if delay < 0:
        raise ValueError('Departure delay must be nonnegative')
    return _shift(clock, delay).source


def plan_departure_delays(timelines, minimum_separation_m, delay_step_s=10, max_delay_s=3600):
    """Deterministic priority scheduling, including initial and final holds.

    Search additional delays 0,step,... and the exact maximum bound. Earlier
    platforms retain their selected schedules. This is a feasible greedy plan,
    not a completeness or optimality claim; failure never skips a platform.
    """
    separation = _finite(minimum_separation_m, 'minimum_separation_m')
    step, limit = _finite(delay_step_s, 'delay_step_s'), _finite(max_delay_s, 'max_delay_s')
    if separation < 0 or step <= 0 or limit < 0 or not math.isfinite(limit/step):
        raise ValueError('Separation and maximum delay must be nonnegative; delay step positive')
    if not isinstance(timelines, (list, tuple)) or not timelines:
        raise ValueError('At least one timeline is required')
    clocks = [_validated(timeline) for timeline in timelines]
    for j, clock in enumerate(clocks):
        for i in range(j):
            distance = math.dist(clocks[i].initial, clock.initial)
            if distance < separation:
                raise ValueError(f'Initial positions {i}/{j} violate separation; departure delays cannot repair holds at t=0')
    delays = [i*step for i in range(int(limit/step)+1)]
    if delays[-1] < limit:
        delays.append(limit)
    chosen, selected_delays, attempts = [], [], []
    for index, clock in enumerate(clocks):
        accepted = None
        for delay in delays:
            candidate = _shift(clock, delay)
            horizon = max(candidate.finish, *(c.finish for c in chosen)) if chosen else candidate.finish
            pair_checks = [dict(_extrema(other, candidate, 0.0, horizon),
                                a_index=i, b_index=index)
                           for i, other in enumerate(chosen)]
            passed = all(check['minimum_distance_m'] >= separation for check in pair_checks)
            attempts.append({'timeline_index': index, 'additional_delay_s': delay,
                             'passed': passed, 'pair_checks': pair_checks})
            if passed:
                accepted = candidate
                selected_delays.append(delay)
                break
        if accepted is None:
            raise ValueError(f'No feasible departure delay for timeline {index} in [0,{limit}]s with step {step}s and separation {separation}m')
        chosen.append(accepted)
    horizon = max(clock.finish for clock in chosen)
    checks = [dict(_extrema(chosen[i], chosen[j], 0.0, horizon), a_index=i, b_index=j)
              for j in range(len(chosen)) for i in range(j)]
    if any(check['minimum_distance_m'] < separation for check in checks):
        raise ValueError('Full-horizon departure-plan verification failed')
    return {'timelines': [clock.source for clock in chosen], 'delays_s': selected_delays,
            'checks': checks, 'attempts': attempts, 'minimum_separation_m': separation,
            'mission_end_time_s': horizon,
            'method': 'fixed priority, stepped delays plus exact limit, continuous full-horizon checks'}

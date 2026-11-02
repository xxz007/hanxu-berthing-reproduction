"""Necessary forward-turn workspace bounds and stop-to-stop arc-length clocks.

The supporting-halfplane test is a certificate of infeasibility, not a complete
planner: passing it does not establish obstacle-free vehicle motion. Speed laws
are scalar path kinematics, with no hull, current, propulsion or depth model.
"""
import math


def convex_hull(points):
    points = sorted(set(tuple(p) for p in points))
    if len(points) < 3 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('At least three finite 2D vertices required')
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for chain, vertices in ((lower, points), (upper, reversed(points))):
        for p in vertices:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], p) <= 0:
                chain.pop()
            chain.append(p)
    hull = lower[:-1]+upper[:-1]
    if len(hull) < 3:
        raise ValueError('Workspace must have positive area')
    return hull


def forward_turn_witnesses(position, heading, hull, minimum_radius_m):
    """Return violated necessary bounds for a forward-only bounded-curvature path.

    For outward unit normal n and initial unit tangent u with n.u>0, the least
    unavoidable outward displacement before a non-outward heading is reached
    is R*(1-|cross(n,u)|). This follows by integrating cos(theta+s/R) until
    theta+s/R=pi/2. Stops cannot reduce the required geometric workspace.
    Entry checks use a reversed path tangent (heading+pi). In-place rotation
    and backward travel are excluded from this model.
    """
    if (len(position) != 2 or not all(math.isfinite(v) for v in (*position, heading, minimum_radius_m))
            or minimum_radius_m <= 0):
        raise ValueError('Finite pose and positive minimum radius required')
    u = (math.cos(heading), math.sin(heading))
    witnesses = []
    for index, (a, b) in enumerate(zip(hull, hull[1:]+hull[:1])):
        length = math.dist(a, b)
        n = ((b[1]-a[1])/length, (a[0]-b[0])/length)
        outward = n[0]*u[0]+n[1]*u[1]
        if outward <= 1e-10:
            continue
        factor = max(0.0, 1-abs(n[0]*u[1]-n[1]*u[0]))
        clearance = n[0]*(a[0]-position[0])+n[1]*(a[1]-position[1])
        required = minimum_radius_m*factor
        if clearance < required-1e-7:
            witnesses.append({'supporting_edge_index': index, 'outward_normal': list(n),
                              'edge_start': list(a), 'edge_end': list(b),
                              'position': list(position), 'path_heading_rad': heading,
                              'available_outward_distance_m': clearance,
                              'unavoidable_outward_distance_m': required,
                              'workspace_deficit_m': required-clearance,
                              'necessary_radius_upper_bound_m': max(0.0, clearance)/factor})
    return witnesses


def stopping_profile(pieces, cruise_mps, tangential_accel_mps2, lateral_accel_mps2):
    """Rest-to-rest motion on every analytic line/arc; speed is continuous.

    Stop at every primitive join, including curvature jumps. Acceleration and
    jerk may jump; no steering actuator or yaw-acceleration bound is asserted.
    Active scan status is not inferred from primitive geometry.
    """
    if any(not math.isfinite(v) or v <= 0 for v in
           (cruise_mps, tangential_accel_mps2, lateral_accel_mps2)):
        raise ValueError('Positive finite speed and acceleration limits required')
    if not pieces:
        raise ValueError('At least one nonzero analytic primitive required')
    result, current_time, distance = [], 0.0, 0.0
    for i, piece in enumerate(pieces):
        if (piece.kind not in ('line', 'arc') or piece.direction != 1
                or not math.isfinite(piece.length) or piece.length <= 0
                or not math.isfinite(piece.curvature)):
            raise ValueError('Only positive-length forward lines and arcs supported')
        if i and (math.dist(pieces[i-1].evaluate(1)[:2], piece.evaluate(0)[:2]) > 1e-6
                  or abs(math.remainder(pieces[i-1].evaluate(1)[2]-piece.evaluate(0)[2], 2*math.pi)) > 1e-8):
            raise ValueError('Primitive positions and tangent headings must join')
        cap = min(cruise_mps, math.sqrt(lateral_accel_mps2/abs(piece.curvature))) if piece.curvature else cruise_mps
        peak = min(cap, math.sqrt(tangential_accel_mps2*piece.length))
        ramp = peak/tangential_accel_mps2
        ramp_distance = peak*peak/(2*tangential_accel_mps2)
        cruise_distance = max(0.0, piece.length-2*ramp_distance)
        cruise_duration = cruise_distance/peak
        duration = 2*ramp+cruise_duration
        result.append({'piece_index': i, 't0_s': current_time, 't1_s': current_time+duration,
                       's0_m': distance, 'length_m': piece.length, 'curvature_per_m': piece.curvature,
                       'speed_cap_mps': cap, 'peak_speed_mps': peak, 'ramp_duration_s': ramp,
                       'cruise_duration_s': cruise_duration, 'ramp_distance_m': ramp_distance,
                       'accel_mps2': tangential_accel_mps2,
                       'maximum_lateral_acceleration_mps2': peak*peak*abs(piece.curvature)})
        current_time += duration
        distance += piece.length
    return {'segments': result, 'duration_s': current_time, 'length_m': distance,
            'cruise_mps': cruise_mps, 'tangential_accel_limit_mps2': tangential_accel_mps2,
            'lateral_accel_limit_mps2': lateral_accel_mps2, 'zero_speed_at_all_joins': True}


def profile_state(profile, time_s):
    if not math.isfinite(time_s):
        raise ValueError('Query time must be finite')
    segments = profile['segments']
    if time_s <= 0:
        return {'piece_index': 0, 'piece_distance_m': 0.0, 'distance_m': 0.0, 'speed_mps': 0.0}
    if time_s >= profile['duration_s']:
        return {'piece_index': len(segments)-1, 'piece_distance_m': segments[-1]['length_m'],
                'distance_m': profile['length_m'], 'speed_mps': 0.0}
    segment = next(s for s in segments if time_s <= s['t1_s'])
    t = time_s-segment['t0_s']
    ramp, hold = segment['ramp_duration_s'], segment['cruise_duration_s']
    a, peak = segment['accel_mps2'], segment['peak_speed_mps']
    if t <= ramp:
        distance, speed = .5*a*t*t, a*t
    elif t <= ramp+hold:
        distance, speed = segment['ramp_distance_m']+peak*(t-ramp), peak
    else:
        remaining = max(0.0, segment['t1_s']-time_s)
        distance, speed = segment['length_m']-.5*a*remaining*remaining, a*remaining
    return {'piece_index': segment['piece_index'], 'piece_distance_m': distance,
            'distance_m': segment['s0_m']+distance, 'speed_mps': speed}

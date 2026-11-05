"""Fully actuated planar acceleration surrogate, not marine hydrodynamics.

State is (east, north, yaw, water-relative east/north velocity, yaw rate).
World-frame acceleration commands are held constant between updates; constant
drift enters position only. Independent lateral actuation is an explicit
assumption. Heading does not constrain the velocity direction in this model.
"""
import math


def feedback_command(state, sensed, reference, feedforward, integral, step, limits, gains):
    ex, ey = reference[0]-sensed[0], reference[1]-sensed[1]
    integral = tuple(max(-gains['integral_limit'], min(gains['integral_limit'], v+e*step))
                     for v, e in zip(integral, (ex, ey)))
    ax = feedforward[0]+gains['position_p']*ex+gains['velocity_d']*(reference[3]-sensed[3])+gains['position_i']*integral[0]
    ay = feedforward[1]+gains['position_p']*ey+gains['velocity_d']*(reference[4]-sensed[4])+gains['position_i']*integral[1]
    norm = math.hypot(ax, ay)
    if norm > limits['acceleration_mps2']:
        scale = limits['acceleration_mps2']/norm; ax *= scale; ay *= scale
    vx, vy = state[3]+ax*step, state[4]+ay*step
    speed = math.hypot(vx, vy)
    if speed > limits['speed_mps']:
        scale = limits['speed_mps']/speed; vx *= scale; vy *= scale
    ax, ay = (vx-state[3])/step, (vy-state[4])/step
    yaw_error = math.remainder(reference[2]-sensed[2], 2*math.pi)
    alpha = feedforward[2]+gains['heading_p']*yaw_error+gains['yaw_rate_d']*(reference[5]-sensed[5])
    alpha = max(-limits['yaw_acceleration_radps2'], min(limits['yaw_acceleration_radps2'], alpha))
    rate = max(-limits['yaw_rate_radps'], min(limits['yaw_rate_radps'], state[5]+step*alpha))
    return (ax, ay, (rate-state[5])/step), integral


def plant_step(state, command, drift, step):
    """Exact constant-command / constant-drift integration, no reference input."""
    x, y, yaw, vx, vy, rate = state; ax, ay, alpha = command
    return (x+step*(vx+drift[0])+.5*step*step*ax,
            y+step*(vy+drift[1])+.5*step*step*ay,
            yaw+step*rate+.5*step*step*alpha,
            vx+step*ax, vy+step*ay, rate+step*alpha)

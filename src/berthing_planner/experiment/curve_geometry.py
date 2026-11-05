"""Certified chord enclosure of constant-curvature pieces."""
import math


def sample_enclosure(pieces, tolerance):
    points=[]
    worst=0.0
    for piece in pieces:
        if piece.curvature:
            radius=1/abs(piece.curvature)
            angle=piece.length/radius
            maximum=2*math.acos(1-tolerance/radius)
            count=max(1,math.ceil(angle/maximum))
            worst=max(worst,radius*(1-math.cos(angle/(2*count))))
        else:
            count=1
        vertices=[list(piece.evaluate(i/count)[:2]) for i in range(count+1)]
        points.extend(vertices if not points else vertices[1:])
    return points,worst

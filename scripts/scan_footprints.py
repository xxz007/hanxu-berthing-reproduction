"""Native bounds for a straight segment's exact circular sensor footprint."""
import math


def straight_footprint_bounds(geos,points,radius,quadrant_segments=30):
    # Keep the historical inscribed-buffer lower footprint unchanged. A tight
    # upper bound uses the exact-width strip and circumscribed endpoint disks;
    # enlarging a whole long-line buffer would needlessly widen the strip.
    a,b=points;length=math.dist(a,b);outer_sensor_radius=radius+1e-6
    normal=[-(b[1]-a[1])*outer_sensor_radius/length,(b[0]-a[0])*outer_sensor_radius/length]
    ring=[[p[j]+sign*normal[j]for j in (0,1)]for p,sign in ((a,1),(b,1),(b,-1),(a,-1),(a,1))]
    lower=geos.operation('buffer',{'type':'LineString','coordinates':points},distance=radius,quadrant_segments=quadrant_segments)
    # A1e-6m outward numerical guard avoids false Covers at analytically exact
    # circle tangencies after native floating-point polygon union.
    outer_radius=outer_sensor_radius/math.cos(math.pi/(4*quadrant_segments))
    disks=[geos.operation('buffer',{'type':'Point','coordinates':p},distance=outer_radius,quadrant_segments=quadrant_segments)for p in points]
    upper=geos.operation('union',{'type':'GeometryCollection','geometries':[{'type':'Polygon','coordinates':[ring]}]+disks})
    return lower,upper

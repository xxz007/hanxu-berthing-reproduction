"""Ellipsoidal Mercator and explicit E/N-to-grid coordinates.

Default axes are the thesis values, not a silently substituted WGS84 ellipsoid.
Angles at this public boundary are degrees; projected distances are metres.
"""
from dataclasses import dataclass, field
import math


def _finite(*values):
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Coordinates and projection parameters must be finite')


@dataclass(frozen=True)
class Mercator:
    a: float = 6378137.0
    b: float = 6356755.0
    lon0_deg: float = 0.0
    lat_ts_deg: float = 0.0
    _e: float = field(init=False, repr=False)
    _K: float = field(init=False, repr=False)

    def __post_init__(self):
        _finite(self.a, self.b, self.lon0_deg, self.lat_ts_deg)
        if not 0 < self.b <= self.a or not -180 <= self.lon0_deg <= 180:
            raise ValueError('Require 0 < b <= a and central longitude in [-180, 180]')
        if not -90 < self.lat_ts_deg < 90:
            raise ValueError('Latitude of true scale must exclude the poles')
        e = math.sqrt(1 - (self.b / self.a)**2)
        phi_ts = math.radians(self.lat_ts_deg)
        k = self.a * math.cos(phi_ts) / math.sqrt(1 - e*e*math.sin(phi_ts)**2)
        object.__setattr__(self, '_e', e)
        object.__setattr__(self, '_K', k)

    def forward(self, lon_deg, lat_deg):
        """Return east/north metres; longitude wraps about the central meridian."""
        _finite(lon_deg, lat_deg)
        if not -180 <= lon_deg <= 180 or not -90 < lat_deg < 90:
            raise ValueError('Require longitude in [-180, 180] and latitude strictly inside (-90, 90)')
        lam = math.remainder(math.radians(lon_deg - self.lon0_deg), 2*math.pi)
        phi = math.radians(lat_deg)
        psi = math.asinh(math.tan(phi)) - self._e*math.atanh(self._e*math.sin(phi))
        return self._K*lam, self._K*psi

    def inverse(self, E, N):
        """Return (longitude, latitude) in degrees from standard east/north metres."""
        _finite(E, N)
        psi = N / self._K
        # Use a non-positive exponential to avoid overflow in the south.
        t = math.exp(-abs(psi))
        phi = math.pi/2 - 2*math.atan(t)
        for _ in range(30):
            es = self._e*math.sin(phi)
            updated = math.pi/2 - 2*math.atan(t*((1-es)/(1+es))**(self._e/2))
            if abs(updated - phi) <= 1e-14:
                phi = updated
                break
            phi = updated
        else:
            raise ValueError('Ellipsoidal Mercator latitude iteration did not converge')
        if phi >= math.pi/2:
            raise ValueError('Northing is too close to a pole for finite-precision inversion')
        lon = math.remainder(self.lon0_deg + math.degrees(E / self._K), 360.0)
        return lon, math.copysign(math.degrees(phi), N)


@dataclass(frozen=True)
class GridTransform:
    """Continuous grid coordinates measured from a corner, not from cell centres.

    ``north`` rows increase with N; ``south`` rows increase downward in images.
    dx_m and dy_m are metres per cell, so world_to_grid divides by them.
    """
    origin_E: float
    origin_N: float
    dx_m: float
    dy_m: float
    row_direction: str = 'north'

    def __post_init__(self):
        _finite(self.origin_E, self.origin_N, self.dx_m, self.dy_m)
        if self.dx_m <= 0 or self.dy_m <= 0:
            raise ValueError('Cell width and height must be positive metres per cell')
        if self.row_direction not in ('north', 'south'):
            raise ValueError('row_direction must be north or south')

    def world_to_grid(self, E, N):
        _finite(E, N)
        sign = 1 if self.row_direction == 'north' else -1
        return (E-self.origin_E)/self.dx_m, sign*(N-self.origin_N)/self.dy_m

    def grid_to_world(self, col, row):
        _finite(col, row)
        sign = 1 if self.row_direction == 'north' else -1
        return self.origin_E+col*self.dx_m, self.origin_N+sign*row*self.dy_m

    def cell_center(self, col, row):
        return self.grid_to_world(col+.5, row+.5)

    def cell_index(self, E, N):
        """Containing cell; half-open cells put a boundary in the next cell."""
        col, row = self.world_to_grid(E, N)
        return math.floor(col), math.floor(row)


def thesis_inverse_literal(x, y):
    """Audit-only literal eq. (3.8): x enters latitude and y longitude.

    This does not invert eq. (3.7)'s x=E, y=N convention. The regular inverse
    method is used for actual planning; the literal expression is kept visible.
    """
    return Mercator().inverse(y, x)


def thesis_grid_literal(Ux, Vx, origin_x, origin_y, dx, dy):
    """Audit-only multiplication in eq. (3.9), without assigning scale units."""
    _finite(Ux, Vx, origin_x, origin_y, dx, dy)
    return (Ux-origin_x)*dx, (Vx-origin_y)*dy

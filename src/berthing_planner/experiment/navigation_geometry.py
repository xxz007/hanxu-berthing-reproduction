"""Reuse native static chart geometries during many A6 candidate queries.

Same GEOS predicates as chart_geometry, with retained native objects rather
than repeatedly serializing the unchanged chart for every candidate.
"""
import ctypes as C
from berthing_planner.experiment.chart_geometry import Geos


class NavigationGeometry:
    def __init__(self,navigation,hazards):
        self.geos=Geos()
        g=self.geos
        self.navigation=g._read(navigation)
        self.hazards=g._read(hazards)
        self.prepared=g.lib.GEOSPrepare_r(g.context,self.navigation)
        if not self.prepared:raise ValueError('Could not prepare navigation geometry')

    def __enter__(self):return self

    def __exit__(self,*unused):
        g=self.geos
        g.lib.GEOSPreparedGeom_destroy_r(g.context,self.prepared)
        g.lib.GEOSGeom_destroy_r(g.context,self.navigation)
        g.lib.GEOSGeom_destroy_r(g.context,self.hazards)
        g.__exit__()

    def covers(self,shape):
        g=self.geos;p=g._read(shape)
        try:
            value=g.lib.GEOSPreparedCovers_r(g.context,self.prepared,p)
            if value==2:raise ValueError('Prepared navigation query failed')
            return bool(value)
        finally:g.lib.GEOSGeom_destroy_r(g.context,p)

    def distance(self,shape,*,to_hazards=False):
        g=self.geos;p=g._read(shape);distance=C.c_double()
        try:
            target=self.hazards if to_hazards else self.navigation
            if not g.lib.GEOSDistance_r(g.context,p,target,C.byref(distance)):raise ValueError('Static chart distance failed')
            return distance.value
        finally:g.lib.GEOSGeom_destroy_r(g.context,p)

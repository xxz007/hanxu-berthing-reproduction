"""Small GeoJSON boundary for the existing GEOS C library used in preprocessing."""
import ctypes as C
import ctypes.util
import json
from pathlib import Path


class Geos:
    def __init__(self, library=None):
        if library is None:
            library = ctypes.util.find_library('geos_c')
            if library is None:
                matches = sorted(Path('/Applications/Postgres.app/Contents/Versions/18/lib').glob('libgeos_c.*.dylib'))
                library = str(matches[0]) if matches else None
        if library is None:
            raise ValueError('Existing GEOS C library required; no dependency is installed automatically')
        self.lib = C.CDLL(str(library))
        self.library = str(library)
        pointer = C.c_void_p
        signatures = {
            'GEOS_init_r': ([], pointer), 'GEOS_finish_r': ([pointer], None),
            'GEOSversion': ([], C.c_char_p),
            'GEOSGeoJSONReader_create_r': ([pointer], pointer),
            'GEOSGeoJSONReader_destroy_r': ([pointer, pointer], None),
            'GEOSGeoJSONReader_readGeometry_r': ([pointer, pointer, C.c_char_p], pointer),
            'GEOSGeoJSONWriter_create_r': ([pointer], pointer),
            'GEOSGeoJSONWriter_destroy_r': ([pointer, pointer], None),
            'GEOSGeoJSONWriter_writeGeometry_r': ([pointer, pointer, pointer, C.c_int], pointer),
            'GEOSGeom_destroy_r': ([pointer, pointer], None), 'GEOSFree_r': ([pointer, pointer], None),
            'GEOSUnaryUnion_r': ([pointer, pointer], pointer),
            'GEOSBuffer_r': ([pointer, pointer, C.c_double, C.c_int], pointer),
            'GEOSIntersection_r': ([pointer, pointer, pointer], pointer),
            'GEOSDifference_r': ([pointer, pointer, pointer], pointer),
            'GEOSPointOnSurface_r': ([pointer, pointer], pointer),
            'GEOSArea_r': ([pointer, pointer, C.POINTER(C.c_double)], C.c_int),
            'GEOSLength_r': ([pointer, pointer, C.POINTER(C.c_double)], C.c_int),
            'GEOSDistance_r': ([pointer, pointer, pointer, C.POINTER(C.c_double)], C.c_int),
            'GEOSCovers_r': ([pointer, pointer, pointer], C.c_ubyte),
            'GEOSPrepare_r': ([pointer, pointer], pointer),
            'GEOSPreparedGeom_destroy_r': ([pointer, pointer], None),
            'GEOSPreparedCovers_r': ([pointer, pointer, pointer], C.c_ubyte),
            'GEOSGeom_createRectangle_r': ([pointer, C.c_double, C.c_double, C.c_double, C.c_double], pointer),
            'GEOSisValid_r': ([pointer, pointer], C.c_ubyte),
            'GEOSGetNumGeometries_r': ([pointer, pointer], C.c_int)
        }
        for name, (args, result) in signatures.items():
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = args, result
        self.context = self.lib.GEOS_init_r()
        self.reader = self.lib.GEOSGeoJSONReader_create_r(self.context)
        self.writer = self.lib.GEOSGeoJSONWriter_create_r(self.context)
        self.version = self.lib.GEOSversion().decode()

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        self.lib.GEOSGeoJSONReader_destroy_r(self.context, self.reader)
        self.lib.GEOSGeoJSONWriter_destroy_r(self.context, self.writer)
        self.lib.GEOS_finish_r(self.context)

    def _read(self, geometry):
        result = self.lib.GEOSGeoJSONReader_readGeometry_r(
            self.context, self.reader, json.dumps(geometry).encode())
        if not result:
            raise ValueError('GEOS could not read input GeoJSON geometry')
        return result

    def operation(self, name, *geometries, distance=0.0, quadrant_segments=30):
        inputs, result, encoded = [], None, None
        try:
            for geometry in geometries:
                inputs.append(self._read(geometry))
            if name == 'union':
                result = self.lib.GEOSUnaryUnion_r(self.context, inputs[0])
            elif name == 'buffer':
                result = self.lib.GEOSBuffer_r(self.context, inputs[0], distance, quadrant_segments)
            elif name == 'point_on_surface':
                result = self.lib.GEOSPointOnSurface_r(self.context, inputs[0])
            elif name in ('intersection', 'difference'):
                fn = self.lib.GEOSIntersection_r if name == 'intersection' else self.lib.GEOSDifference_r
                result = fn(self.context, *inputs)
            else:
                raise ValueError(f'Unknown geometry operation: {name}')
            if not result:
                raise ValueError(f'GEOS {name} failed; no geometry repair or exclusion was applied')
            encoded = self.lib.GEOSGeoJSONWriter_writeGeometry_r(self.context, self.writer, result, 0)
            if not encoded:
                raise ValueError('GEOS GeoJSON serialization failed')
            return json.loads(C.string_at(encoded))
        finally:
            if encoded:
                self.lib.GEOSFree_r(self.context, encoded)
            if result:
                self.lib.GEOSGeom_destroy_r(self.context, result)
            for item in inputs:
                self.lib.GEOSGeom_destroy_r(self.context, item)

    def measure(self, geometry):
        pointer = self._read(geometry)
        try:
            area = C.c_double()
            if not self.lib.GEOSArea_r(self.context, pointer, C.byref(area)):
                raise ValueError('GEOS area calculation failed')
            length = C.c_double()
            if not self.lib.GEOSLength_r(self.context, pointer, C.byref(length)):
                raise ValueError('GEOS length calculation failed')
            valid = self.lib.GEOSisValid_r(self.context, pointer)
            count = self.lib.GEOSGetNumGeometries_r(self.context, pointer)
            if valid == 2 or count < 0:
                raise ValueError('GEOS geometry inspection failed')
            return {'area_m2': area.value, 'valid': bool(valid), 'components': count,
                    'length_m': length.value}
        finally:
            self.lib.GEOSGeom_destroy_r(self.context, pointer)

    def distance(self, a, b):
        pa, pb = self._read(a), None
        try:
            pb = self._read(b)
            result = C.c_double()
            if not self.lib.GEOSDistance_r(self.context, pa, pb, C.byref(result)):
                raise ValueError('GEOS distance calculation failed')
            return result.value
        finally:
            self.lib.GEOSGeom_destroy_r(self.context, pa)
            if pb:
                self.lib.GEOSGeom_destroy_r(self.context, pb)

    def covers(self, container, geometry):
        """Whole-geometry inclusion, including a covered boundary."""
        outer, inner = self._read(container), None
        try:
            inner = self._read(geometry)
            value = self.lib.GEOSCovers_r(self.context, outer, inner)
            if value == 2:
                raise ValueError('GEOS covers predicate failed')
            return bool(value)
        finally:
            self.lib.GEOSGeom_destroy_r(self.context, outer)
            if inner:
                self.lib.GEOSGeom_destroy_r(self.context, inner)

    def covers_rectangles(self, container, rectangles):
        """Prepared whole-cell inclusion for an iterable of [xmin,ymin,xmax,ymax].

        This tests complete closed rectangles, rather than their centers or
        corners. The returned booleans retain the input order.
        """
        outer, prepared = self._read(container), None
        try:
            prepared = self.lib.GEOSPrepare_r(self.context, outer)
            if not prepared:
                raise ValueError('GEOS could not prepare inclusion geometry')
            result = []
            for bounds in rectangles:
                rectangle = self.lib.GEOSGeom_createRectangle_r(self.context, *bounds)
                if not rectangle:
                    raise ValueError('GEOS could not create inclusion rectangle')
                try:
                    value = self.lib.GEOSPreparedCovers_r(self.context, prepared, rectangle)
                    if value == 2:
                        raise ValueError('GEOS prepared covers predicate failed')
                    result.append(bool(value))
                finally:
                    self.lib.GEOSGeom_destroy_r(self.context, rectangle)
            return result
        finally:
            if prepared:
                self.lib.GEOSPreparedGeom_destroy_r(self.context, prepared)
            self.lib.GEOSGeom_destroy_r(self.context, outer)

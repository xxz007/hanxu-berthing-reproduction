"""Whole-cell inclusion, boundary contact, holes and raster orientation."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from chart_geometry import Geos
from prepare_coverage_grid import rasterize_closed_cell_grid

CRS = {'type': 'name', 'properties': {'name': 'EPSG:3857'}}


def rectangle(x0, y0, x1, y1):
    return {'type': 'Polygon', 'coordinates': [[[x0, y0], [x1, y0],
                                               [x1, y1], [x0, y1], [x0, y0]]]}


class CoverageGridTests(unittest.TestCase):
    def test_native_length_covers_and_representative_point(self):
        outer, hole = rectangle(0, 0, 8, 8), rectangle(3, 3, 5, 5)
        with Geos() as geos:
            frame = geos.operation('difference', outer, hole)
            self.assertAlmostEqual(geos.measure(outer)['length_m'], 32)
            self.assertAlmostEqual(geos.measure(frame)['length_m'], 40)
            line = {'type': 'LineString', 'coordinates': [[0, 0], [3, 4]]}
            self.assertAlmostEqual(geos.measure(line)['length_m'], 5)
            self.assertTrue(geos.covers(outer, line))
            self.assertFalse(geos.covers(frame, {'type': 'LineString', 'coordinates': [[0, 0], [6, 6]]}))
            self.assertTrue(geos.covers(outer, {'type': 'LineString', 'coordinates': [[0, 0], [8, 0]]}))
            point = geos.operation('point_on_surface', frame)
            self.assertEqual(point['type'], 'Point')
            self.assertTrue(geos.covers(frame, point))
            self.assertFalse(geos.covers(hole, point))
            # All four corners of this cell lie in frame, but its interior
            # contains the hole. A corners-only check would incorrectly pass.
            self.assertEqual(geos.covers_rectangles(frame, [(1, 1, 2, 2), (2, 2, 6, 6),
                                                           (0, 0, 1, 1), (7, 7, 9, 9)]),
                             [True, False, True, False])

    def test_hole_and_source_boundary_touch_are_blocked(self):
        with Geos() as geos:
            frame = geos.operation('difference', rectangle(1, 1, 7, 7), rectangle(3.2, 3.2, 4.8, 4.8))
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'grid'
            grid, summary = rasterize_closed_cell_grid(frame, CRS, (0, 0, 8, 8), 8, out)
            rows = grid['rows_bottom_to_top']
            self.assertEqual(rows[2][2], 0)
            self.assertEqual(rows[3][3], 1)
            self.assertEqual(rows[4][4], 1)
            self.assertEqual(rows[1][2], 1)
            self.assertEqual(rows[6][2], 1)
            self.assertTrue(summary['complete_free_cell_original_covers_passed'])
            self.assertTrue(summary['complete_free_cell_inset_covers_passed'])
            self.assertEqual(json.loads((out / 'closed_cell_audit.json').read_text())[
                'not_covered_by_original_candidate'], [])

    def test_slanted_boundary_touch_is_not_center_only(self):
        triangle = {'type': 'Polygon', 'coordinates': [[[1, 1], [7, 1], [1, 7], [1, 1]]]}
        with tempfile.TemporaryDirectory() as folder:
            grid, summary = rasterize_closed_cell_grid(triangle, CRS, (0, 0, 8, 8), 8,
                                                      Path(folder) / 'grid')
            rows = grid['rows_bottom_to_top']
            self.assertEqual(rows[2][2], 0)
            self.assertEqual(rows[3][2], 0)
            # Center (3.5,3.5) is inside, but upper corner touches the diagonal.
            self.assertEqual(rows[3][3], 1)
            self.assertLess(summary['retained_closed_cell_area_m2'], summary['source_candidate_area_m2'])

    def test_narrow_neck_components_are_all_retained(self):
        with Geos() as geos:
            joined = geos.operation('union', {'type': 'GeometryCollection', 'geometries': [
                rectangle(1, 1, 3, 7), rectangle(2, 3.8, 6, 4.2), rectangle(5, 1, 7, 7)]})
            self.assertEqual(geos.measure(joined)['components'], 1)
        with tempfile.TemporaryDirectory() as folder:
            _, summary = rasterize_closed_cell_grid(joined, CRS, (0, 0, 8, 8), 16,
                                                   Path(folder) / 'grid')
            self.assertEqual(summary['connected_components_4'], 2)
            self.assertTrue(summary['all_components_retained'])
            self.assertEqual(len(summary['components_4']), 2)
            self.assertTrue(all(part['cells'] > 0 for part in summary['components_4']))

    def test_north_down_raw_to_bottom_up_grid_is_explicit(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'grid'
            grid, summary = rasterize_closed_cell_grid(rectangle(1.2, 1.2, 3.8, 3.8), CRS,
                                                      (0, 0, 8, 8), 8, out)
            rows = grid['rows_bottom_to_top']
            raw = (out / 'blocked.bin').read_bytes()
            self.assertEqual(rows[2][2], 0)
            self.assertEqual(rows[5][2], 1)
            self.assertEqual(raw[(7-2)*8+2], 0)
            self.assertEqual(raw[(7-5)*8+2], 1)
            self.assertEqual(summary['north_down_GDAL_transform'], [0, 1, 0, 8, 0, -1])
            self.assertTrue(summary['rows_bottom_to_top_conversion_checked'])

    def test_empty_candidate_produces_no_free_components(self):
        empty = {'type': 'Polygon', 'coordinates': []}
        with tempfile.TemporaryDirectory() as folder:
            grid, summary = rasterize_closed_cell_grid(empty, CRS, (0, 0, 8, 8), 8,
                                                      Path(folder) / 'grid')
            self.assertTrue(all(value == 1 for row in grid['rows_bottom_to_top'] for value in row))
            self.assertEqual(summary['free_cells'], 0)
            self.assertEqual(summary['connected_components_4'], 0)

    def test_bounds_and_existing_outputs_are_not_silently_changed(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, 'entire fixed candidate'):
                rasterize_closed_cell_grid(rectangle(-1, 1, 3, 3), CRS, (0, 0, 8, 8), 8,
                                           Path(folder) / 'outside')
            out = Path(folder) / 'existing'; out.mkdir(); (out / 'keep').write_text('original')
            with self.assertRaises(FileExistsError):
                rasterize_closed_cell_grid(rectangle(1, 1, 3, 3), CRS, (0, 0, 8, 8), 8, out)
            self.assertEqual((out / 'keep').read_text(), 'original')

    def test_cli_grid_size_and_manifest_use_actual_resolution(self):
        with tempfile.TemporaryDirectory() as folder:
            prepared = Path(folder) / 'prepared'; prepared.mkdir()
            candidate = prepared / 'searchable.geojson'
            candidate.write_text(json.dumps({'type': 'FeatureCollection', 'crs': CRS,
                                            'features': [{'type': 'Feature', 'properties': {},
                                                          'geometry': rectangle(1000, 1000, 4000, 4000)}]}))
            digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
            (prepared / 'manifest.json').write_text(json.dumps({
                'derived_files_sha256': {'searchable.geojson': digest}}))
            for name in ('config.json', 'summary.json'):
                (prepared / name).write_text('{}\n')
            out = Path(folder) / 'grid'
            command = [sys.executable, str(Path(__file__).resolve().parents[1] / 'scripts/prepare_coverage_grid.py'),
                       '--prepared-dir', str(prepared), '--output-dir', str(out), '--n', '8']
            subprocess.run(command, check=True, capture_output=True, text=True)
            grid = json.loads((out / 'grid.json').read_text())
            manifest = json.loads((out / 'manifest.json').read_text())
            self.assertEqual(grid['n'], 8)
            self.assertEqual(grid['dx_m'], 640)
            self.assertEqual(manifest['n'], 8)
            self.assertEqual(manifest['dx_m'], 640)
            self.assertEqual(manifest['dy_m'], 640)
            self.assertEqual(manifest['candidate_sha256'], digest)
            self.assertEqual(manifest['grid_sha256'], hashlib.sha256((out / 'grid.json').read_bytes()).hexdigest())


if __name__ == '__main__':
    unittest.main()

import io
import struct
import tempfile
import unittest
from pathlib import Path
import zipfile

from berthing_planner.gshhg import clip_polygon, read_polygons


class GshhgTests(unittest.TestCase):
    def test_clip_enclosing_polygon_and_disjoint_polygon(self):
        clipped = clip_polygon([(-5, -5), (5, -5), (5, 5), (-5, 5)], (0, 0, 2, 2))
        self.assertEqual(set(clipped), {(0, 0), (2, 0), (2, 2), (0, 2)})
        self.assertEqual(clip_polygon([(5, 5), (6, 5), (5, 6)], (0, 0, 2, 2)), [])

    def test_native_big_endian_header_hierarchy_and_greenwich_wrap(self):
        data = io.BytesIO()
        rings = [(1, 1, [(350, 0), (5, 0), (5, 5), (350, 5), (350, 0)]),
                 (2, 2, [(1, 1), (2, 1), (2, 2), (1, 2), (1, 1)]),
                 (3, 1, [(50, 50), (51, 50), (51, 51), (50, 50)])]
        for identifier, level, points in rings:
            flag = level | (15 << 8) | ((1 if identifier == 1 else 0) << 16)
            west = -10 if identifier == 1 else min(p[0] for p in points)
            header = (identifier, len(points), flag, int(west*1e6), int(max(p[0] for p in points if p[0] < 270)*1e6),
                      int(min(p[1] for p in points)*1e6), int(max(p[1] for p in points)*1e6), 0, 0, -1, identifier)
            data.write(struct.pack('>11i', *header))
            for x, y in points:
                data.write(struct.pack('>ii', int(x*1e6), int(y*1e6)))
        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp)/'fixture.zip'
            with zipfile.ZipFile(archive, 'w') as z:
                z.writestr('gshhs_c.b', data.getvalue())
            polygons = read_polygons(archive, 'c', (0, 0, 4, 4))
        self.assertEqual([p['id'] for p in polygons], [1, 2])
        self.assertEqual([p['level'] for p in polygons], [1, 2])
        self.assertEqual(set(polygons[0]['points_lonlat']), {(0, 0), (4, 0), (4, 4), (0, 4)})

    def test_reject_truncated_input_and_unsupported_window(self):
        with self.assertRaises(ValueError):
            read_polygons('unused.zip', 'x', (0, 0, 1, 1))
        with self.assertRaises(ValueError):
            read_polygons('unused.zip', 'c', (170, 0, 190, 1))
        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp)/'broken.zip'
            with zipfile.ZipFile(archive, 'w') as z:
                z.writestr('gshhs_c.b', b'bad')
            with self.assertRaisesRegex(ValueError, 'Truncated'):
                read_polygons(archive, 'c', (0, 0, 1, 1))


if __name__ == '__main__':
    unittest.main()

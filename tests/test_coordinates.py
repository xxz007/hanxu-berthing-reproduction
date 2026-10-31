import math
import random
import unittest

from berthing_planner.coordinates import (
    GridTransform, Mercator, thesis_grid_literal, thesis_inverse_literal)


class MercatorTests(unittest.TestCase):
    def test_default_ellipsoid_is_thesis_not_wgs84(self):
        p = Mercator()
        self.assertEqual((p.a, p.b), (6378137, 6356755))

    def test_origin_and_equatorial_degree_have_analytic_values(self):
        p = Mercator()
        self.assertEqual(p.forward(0, 0), (0, 0))
        self.assertAlmostEqual(p.forward(1, 0)[0], 6378137*math.pi/180, places=8)
        self.assertEqual(p.forward(1, 0)[1], 0)

    def test_independent_proj_9_7_1_reference_values(self):
        # Frozen outputs from official PROJ CLI using precisely the thesis a,b.
        for lon, lat, E, N in (
                (103.8, 1.2, 11554963.144341796637, 132699.074712052825),
                (12, 45, 1335833.889519282850, 5591299.712522516027)):
            p = Mercator()
            actual = p.forward(lon, lat)
            self.assertAlmostEqual(actual[0], E, places=7)
            self.assertAlmostEqual(actual[1], N, places=7)
            recovered = p.inverse(E, N)
            self.assertAlmostEqual(recovered[0], lon, places=10)
            self.assertAlmostEqual(recovered[1], lat, places=10)

    def test_nonzero_standard_parallel_and_central_meridian(self):
        p = Mercator(lon0_deg=103, lat_ts_deg=30)
        self.assertAlmostEqual(p.forward(103.8, 1.2)[0], 77189.016088547069, places=7)
        self.assertAlmostEqual(p.forward(103.8, 1.2)[1], 115017.043959374481, places=7)
        # Dateline wrapping is the PROJ convention, not a 340-degree detour.
        E, N = p.forward(-170, -60)
        self.assertAlmostEqual(E, 8394305.499629613012, places=7)
        self.assertAlmostEqual(N, -7248380.621355306357, places=7)

    def test_sphere_has_independent_closed_form_at_45_degrees(self):
        p = Mercator(a=1000, b=1000)
        self.assertAlmostEqual(p.forward(0, 45)[1], 1000*math.log(1+math.sqrt(2)), places=10)

    def test_east_north_axes_are_independent(self):
        p = Mercator()
        self.assertEqual(p.forward(1, 0)[1], 0)
        self.assertEqual(p.forward(0, 1)[0], 0)
        self.assertGreater(p.forward(1, 0)[0], 0)
        self.assertGreater(p.forward(0, 1)[1], 0)

    def test_roundtrip_with_fixed_random_inputs_and_hemispheres(self):
        rng = random.Random(20261030)
        for p in (Mercator(), Mercator(lon0_deg=103.8, lat_ts_deg=1.2)):
            for _ in range(100):
                lon, lat = rng.uniform(-179, 179), rng.uniform(-85, 85)
                actual = p.inverse(*p.forward(lon, lat))
                self.assertAlmostEqual(actual[0], lon, places=10)
                self.assertAlmostEqual(actual[1], lat, places=10)

    def test_literal_thesis_inverse_does_not_invert_forward_axes(self):
        p = Mercator()
        x, y = p.forward(103.8, 1.2)
        literal_lon, literal_lat = thesis_inverse_literal(x, y)
        self.assertAlmostEqual(literal_lon, y/p.a*180/math.pi, places=10)
        self.assertGreater(abs(literal_lon-103.8), 100)
        self.assertGreater(abs(literal_lat-1.2), 60)

    def test_nonfinite_and_polar_inputs_are_rejected(self):
        p = Mercator()
        for lon, lat in ((0, 90), (0, -90), (181, 0), (0, math.nan), (math.inf, 0)):
            with self.subTest(lon=lon, lat=lat), self.assertRaises(ValueError):
                p.forward(lon, lat)
        for E, N in ((math.inf, 0), (0, math.nan), (0, 1e100)):
            with self.subTest(E=E, N=N), self.assertRaises(ValueError):
                p.inverse(E, N)
        for kwargs in ({'a':0}, {'b':0}, {'b':7e6}, {'lat_ts_deg':90}, {'a':math.inf}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Mercator(**kwargs)


class GridTransformTests(unittest.TestCase):
    def test_north_up_anisotropic_grid_uses_division(self):
        g = GridTransform(1000, 2000, 20, 50)
        self.assertEqual(g.world_to_grid(1060, 2100), (3, 2))
        self.assertEqual(g.grid_to_world(3, 2), (1060, 2100))
        self.assertEqual(g.cell_center(3, 2), (1070, 2125))

    def test_south_down_image_grid_has_explicit_sign(self):
        g = GridTransform(1000, 2500, 20, 50, 'south')
        self.assertEqual(g.world_to_grid(1060, 2400), (3, 2))
        self.assertEqual(g.cell_center(3, 2), (1070, 2375))

    def test_half_open_cell_boundaries_and_negative_indices(self):
        g = GridTransform(1000, 2000, 20, 50)
        self.assertEqual(g.cell_index(1020, 2050), (1, 1))
        self.assertEqual(g.cell_index(1019.999, 2049.999), (0, 0))
        self.assertEqual(g.cell_index(999.999, 1999.999), (-1, -1))

    def test_eq39_literal_is_only_equivalent_with_reciprocal_scale_units(self):
        g = GridTransform(1000, 2000, 20, 50)
        correct = g.world_to_grid(1060, 2100)
        self.assertEqual(thesis_grid_literal(1060,2100,1000,2000,1/20,1/50), correct)
        self.assertNotEqual(thesis_grid_literal(1060,2100,1000,2000,20,50), correct)

    def test_invalid_grid_parameters_and_coordinates_are_rejected(self):
        for args in ((0,0,0,1), (0,0,1,-1), (0,0,1,1,'east'), (math.inf,0,1,1)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                GridTransform(*args)
        with self.assertRaises(ValueError):
            GridTransform(0,0,1,1).world_to_grid(math.nan, 0)


if __name__ == '__main__':
    unittest.main()

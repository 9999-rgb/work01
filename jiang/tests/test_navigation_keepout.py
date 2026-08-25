"""Unit tests for the navigation keep-out band crossing detour.

The keep-out band models the cabinet row that splits the map into a west
aisle (control face) and an east aisle (switch dock).  The axis-by-axis
navigation policy must route crossing legs through clear space instead of
letting Nav2 hug the band's inflation edge, where DWB throttles the base to
a crawl and the leg times out.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
# ``control_gateway.__init__`` intentionally exposes the ROS-backed server.
# Load this pure module without requiring a sourced ROS workspace.
import types  # noqa: E402

CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from control_gateway.robot_adapter import NavigationKeepoutConfig  # noqa: E402
from control_gateway.robot_adapter import keepout_crossing_waypoint  # noqa: E402


class KeepoutCrossingWaypointTest(unittest.TestCase):
    def setUp(self) -> None:
        # Band matching the current map: wall x~2.0-2.75, y~-2.85..2.85,
        # with free corridors above/below the ends.
        self.band = NavigationKeepoutConfig(
            x_min=1.8,
            x_max=3.0,
            y_min=-2.85,
            y_max=2.85,
            corridor_offset=0.65,
            side_clearance=1.2,
        )

    def test_disabled_band_never_detours(self) -> None:
        self.assertIsNone(
            keepout_crossing_waypoint(1.19, 0.57, 3.433, -0.18, None)
        )

    def _assert_waypoint(self, actual, expected) -> None:
        self.assertIsNotNone(actual)
        assert actual is not None
        self.assertAlmostEqual(actual[0], expected[0], places=6)
        self.assertAlmostEqual(actual[1], expected[1], places=6)

    def test_west_to_east_crossing_routes_via_clear_north_corridor(
        self,
    ) -> None:
        # Start near the knob station, target the switch dock across the wall.
        waypoint = keepout_crossing_waypoint(
            1.19, 0.57, 3.433, -0.18, self.band
        )
        # North corridor is closer to the west-aisle start (y=0.57).
        self._assert_waypoint(waypoint, (0.6, 3.5))

    def test_east_to_west_crossing_routes_via_closer_south_corridor(
        self,
    ) -> None:
        # From the switch dock (y=-0.18) the south corridor is marginally
        # closer than the north one, so the route leaves via the south end.
        waypoint = keepout_crossing_waypoint(
            3.433, -0.18, 1.235, 0.513, self.band
        )
        self._assert_waypoint(waypoint, (4.2, -3.5))

    def test_near_north_tip_prefers_north_corridor(self) -> None:
        # Robot already high in the west aisle should use the near (north) end.
        waypoint = keepout_crossing_waypoint(
            1.32, 2.49, 3.433, -0.18, self.band
        )
        self._assert_waypoint(waypoint, (0.6, 3.5))

    def test_start_south_of_band_uses_south_corridor(self) -> None:
        waypoint = keepout_crossing_waypoint(
            1.19, -0.57, 3.433, 0.18, self.band
        )
        self._assert_waypoint(waypoint, (0.6, -3.5))

    def test_same_aisle_route_needs_no_detour(self) -> None:
        # West aisle to west aisle (knob station to another west control).
        self.assertIsNone(
            keepout_crossing_waypoint(1.235, 0.513, 1.235, -0.5, self.band)
        )
        # East aisle to east aisle.
        self.assertIsNone(
            keepout_crossing_waypoint(3.433, -0.18, 3.6, 0.5, self.band)
        )

    def test_route_entirely_above_or_below_band_needs_no_detour(self) -> None:
        # Both endpoints north of the band: Nav2 goes around the end on its own.
        self.assertIsNone(
            keepout_crossing_waypoint(0.5, 4.0, 3.5, 4.0, self.band)
        )
        self.assertIsNone(
            keepout_crossing_waypoint(0.5, -4.0, 3.5, -4.0, self.band)
        )

    def test_start_past_corridor_is_clear_space_no_detour(self) -> None:
        # The inflation zone reaches corridor_offset beyond either band edge
        # (y_max + 0.65 = 3.5).  A start already north of that corridor drives
        # its X leg in clear space, so the detour must not drag it back south.
        self.assertIsNone(
            keepout_crossing_waypoint(0.6, 4.0, 4.0, -0.08, self.band)
        )
        self.assertIsNone(
            keepout_crossing_waypoint(0.6, -4.0, 4.0, 0.08, self.band)
        )

    def test_start_in_throttling_zone_still_detours(self) -> None:
        # start_y = 3.4 sits between the band edge (2.85) and the corridor
        # (3.5): the X leg hugs inflation there, so the route must still climb
        # to the corridor via point before crossing.
        self._assert_waypoint(
            keepout_crossing_waypoint(0.6, 3.4, 4.0, -0.08, self.band),
            (0.6, 3.5),
        )
        self._assert_waypoint(
            keepout_crossing_waypoint(0.6, -3.4, 4.0, 0.08, self.band),
            (0.6, -3.5),
        )

    def test_keepout_parses_from_adapter_yaml(self) -> None:
        from control_gateway.robot_adapter import load as load_robot_adapter

        adapter = load_robot_adapter(
            str(
                JIANG_DIR.parent
                / "xczs_inspection_robot_control"
                / "config"
                / "cabinet_robot_adapter.yaml"
            ),
            toolset="B",
        )
        band = adapter.navigation_keepout
        self.assertIsNotNone(band)
        assert band is not None
        self.assertEqual(band.x_min, 1.8)
        self.assertEqual(band.x_max, 3.0)
        self.assertEqual(band.y_min, -2.85)
        self.assertEqual(band.y_max, 2.85)
        self.assertEqual(band.corridor_offset, 0.65)
        self.assertEqual(band.side_clearance, 1.2)
        # A crossing against the real band yields the real waypoint.
        self._assert_waypoint(
            keepout_crossing_waypoint(1.19, 0.57, 3.433, -0.18, band),
            (0.6, 3.5),
        )


if __name__ == "__main__":
    unittest.main()

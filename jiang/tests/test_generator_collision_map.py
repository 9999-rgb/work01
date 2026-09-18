"""The generator navigation map must match the CPU lidar collision geometry."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/tools'))
from generate_scene_maps import build_collision_occupancy


def test_generator_map_matches_known_floor_and_equipment():
    model = ROOT / 'xczs_inspection_robot_description/urdf/scenes/generator_plant.xacro'
    grid, meta = build_collision_occupancy(
        model, resolution=.05, margin=1, floor_threshold=.3, clearance_height=.9)
    def cell(x, y):
        col = int((x-meta['origin'][0])/.05)
        row = grid.shape[0]-1-int((y-meta['origin'][1])/.05)
        return grid[row, col]
    for x in (-13.1867, -15.457, -21.997, -30.1, -35.01):
        assert cell(x, 5.8113) == 254
        assert cell(x, 4.3733) == 0
    assert cell(-22, 10) == 0
    assert cell(-13, 10) == 254
    assert cell(-40, 5) == 0
    assert set(np.unique(grid)) == {0, 254}

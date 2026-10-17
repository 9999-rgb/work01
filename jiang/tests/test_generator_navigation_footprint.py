"""Scene parameter generation must cover folded B tools in the Nav2 frame."""
import ast
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]


class GeneratorNavigationFootprintTest(unittest.TestCase):
    def generated(self, scene, toolset):
        source = ROOT / 'xczs_inspection_robot_bringup/launch/inspection_robot.launch.py'
        tree = ast.parse(source.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == '_nav2_params_for_scene')
        class Configuration:
            def __init__(self, name):
                self.name = name
            def perform(self, context):
                return context[self.name]
        with tempfile.TemporaryDirectory() as directory:
            namespace = {'Path': Path, 'yaml': yaml, 'LaunchConfiguration': Configuration,
                         '_normalize_toolset': lambda value: value.upper(),
                         '_active_robot_spawn': lambda _: {'x': 1., 'y': 2., 'yaw': .3},
                         '_make_generated_directory': lambda _: Path(directory)}
            exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
            path = namespace['_nav2_params_for_scene']({
                'scene': scene, 'toolset': toolset,
                'nav2_params_file': str(ROOT / 'xczs_inspection_robot_nav2/config/nav2_params.yaml')})
            return yaml.safe_load(Path(path).read_text())

    def test_generator_b_covers_both_folded_tools(self):
        config = self.generated('generator_plant', 'B')
        for name in ('local_costmap', 'global_costmap'):
            params = config[name][name]['ros__parameters']
            footprint = yaml.safe_load(params['footprint'])
            # Folded tools measured at body X +/-0.75 m; body->base yaw=-pi/2
            # maps those extrema to base_link Y, with 30 mm geometric clearance.
            self.assertLessEqual(min(p[1] for p in footprint), -.78)
            self.assertGreaterEqual(max(p[1] for p in footprint), .78)
            self.assertEqual(params['robot_base_frame'], 'base_link')
        controller = config['controller_server']['ros__parameters']['FollowPath']
        self.assertIn('ObstacleFootprint', controller['critics'])
        self.assertGreater(controller['ObstacleFootprint.scale'], 0.)
        # At the goal, an unobstructed turn must improve the score rather
        # than lose to Twirling's velocity penalty (the previous 100 did).
        for yaw_error in (-1.57, 1.57):
            turn = .2 if yaw_error > 0 else -.2
            lookahead = controller['RotateToGoal.lookahead_time']
            stopped_cost = controller['RotateToGoal.scale'] * abs(yaw_error)
            turning_cost = (controller['RotateToGoal.scale'] * abs(yaw_error - turn * lookahead)
                            + controller['Twirling.scale'] * abs(turn))
            self.assertLess(turning_cost, stopped_cost)
        checker = config['controller_server']['ros__parameters']['general_goal_checker']
        self.assertLessEqual(checker['xy_goal_tolerance'], .08)
        self.assertEqual(controller['xy_goal_tolerance'], checker['xy_goal_tolerance'])
        self.assertLessEqual(controller['sim_time'], 1.)
        for name in ('controller_server', 'bt_navigator', 'velocity_smoother'):
            self.assertEqual(config[name]['ros__parameters']['odom_topic'], '/xczs/nav_odom')
        self.assertEqual(config['amcl']['ros__parameters']['initial_pose']['x'], 1.)
        # Exact simulated odometry must not acquire decimetres of artificial
        # uncertainty during a trip along the repetitive cabinet row.
        for key in ('alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5'):
            self.assertLessEqual(config['amcl']['ros__parameters'][key], .0001)

    def test_generator_local_map_keeps_known_obstacles_outside_lidar_fan(self):
        config = self.generated('generator_plant', 'B')
        local = config['local_costmap']['local_costmap']['ros__parameters']
        self.assertLess(local['plugins'].index('static_layer'),
                        local['plugins'].index('obstacle_layer'))
        self.assertEqual(local['static_layer']['plugin'], 'nav2_costmap_2d::StaticLayer')
        self.assertTrue(local['static_layer']['map_subscribe_transient_local'])

    def test_other_scene_and_toolset_keep_configured_footprint(self):
        original = yaml.safe_load((ROOT / 'xczs_inspection_robot_nav2/config/nav2_params.yaml').read_text())
        for scene, toolset in [('electrical_mezzanine', 'A'), ('generator_plant', 'A')]:
            config = self.generated(scene, toolset)
            self.assertEqual(config['controller_server'], original['controller_server'])
            for name in ('local_costmap', 'global_costmap'):
                self.assertEqual(config[name][name]['ros__parameters']['footprint'],
                                 original[name][name]['ros__parameters']['footprint'])
                self.assertEqual(config[name][name]['ros__parameters']['plugins'],
                                 original[name][name]['ros__parameters']['plugins'])

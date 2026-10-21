"""Exercise real capability teardown after SIGINT, without a Gazebo instance."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def main():
    executable, toolset = sys.argv[1:3]
    during_startup = len(sys.argv) > 3 and sys.argv[3] == "startup"
    share = Path(get_package_share_directory('xczs_inspection_robot_moveit_config'))
    description = Path(get_package_share_directory('xczs_inspection_robot_description'))
    config = (MoveItConfigsBuilder('xczs_inspection_robot', package_name='xczs_inspection_robot_moveit_config')
              .robot_description(file_path=str(description / 'urdf/xczs_inspection_robot.urdf.xacro'),
                                 mappings={'toolset': toolset})
              .robot_description_semantic(file_path=str(share / f'config/xczs_inspection_robot_toolset_{toolset}.srdf'))
              .robot_description_kinematics(file_path=str(share / 'config/kinematics.yaml'))
              .joint_limits(file_path=str(share / f'config/joint_limits_toolset_{toolset}.yaml'))
              .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
              .trajectory_execution(file_path=str(share / f'config/moveit_controllers_toolset_{toolset}.yaml'),
                                    moveit_manage_controllers=False)
              .to_moveit_configs().to_dict())
    config['use_sim_time'] = False
    with tempfile.TemporaryDirectory(prefix='xczs-move-group-shutdown-') as tmp:
        params = Path(tmp) / 'params.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
        log = Path(tmp) / 'runtime.log'
        with log.open('w') as stream:
            proc = subprocess.Popen([executable, '--ros-args', '--params-file', str(params)],
                                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 60
                while proc.poll() is None and time.monotonic() < deadline:
                    marker = 'Loading planning pipeline' if during_startup else 'You can start planning now!'
                    if marker in log.read_text(errors='replace'):
                        break
                    time.sleep(.1)
                else:
                    raise RuntimeError('move_group did not finish loading its real capabilities')
                proc.send_signal(signal.SIGINT)
                result = proc.wait(timeout=25)
                if result != 0:
                    raise RuntimeError(f'move_group shutdown exited {result}')
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                print(log.read_text(errors='replace'))
    print(f'Capability shutdown {toolset} passed')


if __name__ == '__main__':
    main()

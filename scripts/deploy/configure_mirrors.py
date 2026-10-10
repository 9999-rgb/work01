"""只修改容器内 Ubuntu/ROS 软件源，保留原签名密钥与校验。"""
from pathlib import Path

replacements = {
    'http://ports.ubuntu.com/ubuntu-ports': 'https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports',
    'http://archive.ubuntu.com/ubuntu': 'https://mirrors.tuna.tsinghua.edu.cn/ubuntu',
    'http://security.ubuntu.com/ubuntu': 'https://mirrors.tuna.tsinghua.edu.cn/ubuntu',
    'http://packages.ros.org/ros2/ubuntu': 'https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu',
    'https://packages.ros.org/ros2/ubuntu': 'https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu',
}
paths = [Path('/etc/apt/sources.list')]
paths += list(Path('/etc/apt/sources.list.d').glob('*.list'))
paths += list(Path('/etc/apt/sources.list.d').glob('*.sources'))
for path in paths:
    if not path.exists():
        continue
    text = path.read_text()
    for old, new in replacements.items():
        text = text.replace(old, new)
    path.write_text(text)
Path('/etc/apt/apt.conf.d/80-xczs-network').write_text(
    'Acquire::Retries "3";\nAcquire::http::Timeout "30";\nAcquire::https::Timeout "30";\n'
)
sources = Path('/etc/ros/rosdep/sources.list.d')
sources.mkdir(parents=True, exist_ok=True)
# 不运行会访问 GitHub RAW 的 rosdep init；采用镜像站的规则文件。
(sources / '20-default.list').write_text(''.join(
    f'yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/{name}.yaml\n'
    for name in ('base', 'python', 'ruby')
))

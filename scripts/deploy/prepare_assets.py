"""在镜像内重定位地图，宿主机原文件不变。"""
from pathlib import Path
import yaml

root = Path('/opt/work01')
for scene in ('electrical_mezzanine', 'generator_plant'):
    config = root / 'jiang/data/assets/scene' / scene / 'scenes.yaml'
    text = config.read_text()
    for item in yaml.safe_load(text)['scenes']:
        old = item.get('nav2_map')
        if old:
            target = config.parent / 'maps' / Path(old).name
            if not target.is_file():
                raise SystemExit(f'缺少地图：{target}')
            text = text.replace(old, str(target))
    config.write_text(text)

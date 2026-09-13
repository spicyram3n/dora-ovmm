#!/usr/bin/env python3
"""Install tracked grasp-specific files into the separately cloned ROS checkout."""
import argparse
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    args = parser.parse_args()
    overlay = Path(__file__).resolve().parent / 'grasp_overlay'
    manifest = json.loads((overlay / 'manifest.json').read_text())
    for repo, details in manifest.items():
        destination = args.workspace / 'src' / repo
        if not (destination / '.git').exists():
            raise SystemExit(f'Missing vendor checkout: {destination}')
        for name in details['files']:
            source = overlay / repo / name
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        print(f'Installed {len(details["files"])} grasp overlay files in {repo}')


if __name__ == '__main__':
    main()

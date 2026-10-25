"""v0.6 corner-ring regression entry point (no device access or flashing).

Example: python tools/test_ring_corners.py --compiler E:/mingw64/bin/gcc.exe
Legacy ZORE/ring-state fixture tests remain historical, not v0.6 acceptance.
"""
import argparse
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=shutil.which('gcc') or 'gcc')
    parser.add_argument('--suite', action='append', help='Run only named suite(s).')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output or root / 'demo/mdk/Objects/host-tests/corners'
    output.mkdir(parents=True, exist_ok=True)
    suites = {
        'test_ring_corner_geometry': ['ring_corner'],
        'test_ring_corner_control': ['ring_control'],
        'test_ring_corner_integration': ['track_vision', 'ring_corner', 'ring_control', 'track_control'],
        'test_ring_isolation': ['track_vision', 'ring_corner', 'ring_control', 'track_control', 'pid'],
        'test_track_vision': ['track_vision', 'ring_corner'],
        'test_curved_edge': ['track_vision', 'ring_corner'],
        'test_near_route': ['track_vision', 'ring_corner'],
        'test_vision_stability': ['track_vision', 'ring_corner', 'img_process'],
        'test_ground_preprocess': ['track_vision', 'ring_corner', 'img_process'],
        'test_track_preprocess': ['track_vision', 'ring_corner', 'img_process', 'track_control'],
        'test_line_following': ['track_vision', 'ring_corner', 'track_control', 'pid'],
        'test_track_control': ['track_control'],
        'test_pid': ['pid'],
        'test_motor_switch': [],
        'test_imu_navigation': ['imu_navigation'],
        'test_ring_feature_switch': ['track_vision', 'ring_corner', 'ring_control', 'track_control'],
    }
    selected = args.suite or list(suites)
    unknown = set(selected) - suites.keys()
    if unknown:
        parser.error('Unknown suite(s): ' + ', '.join(sorted(unknown)))
    for name in selected:
        print('RUN ' + name, flush=True)
        exe = output / (name + '.exe')
        command = [args.compiler, '-std=c99', '-Wall', '-Wextra', '-Werror',
                   '-Wshadow', '-pedantic', '-O2', '-I', str(root / 'demo/user/inc'),
                   str(root / 'tools' / (name + '.c'))]
        command += [str(root / 'demo/user/src' / (source + '.c')) for source in suites[name]]
        if name == 'test_ring_isolation':
            command += [str(root / 'tools/fixtures/ordinary_before_zore.c')]
        if name == 'test_ring_feature_switch':
            command += ['-DRING_FEATURE_ENABLED=0']
        command += ['-lm', '-o', str(exe)]
        subprocess.run(command, check=True)
        subprocess.run([str(exe.resolve())], check=True)
    print(f'PASS all {len(selected)} host suites', flush=True)


if __name__ == '__main__':
    main()

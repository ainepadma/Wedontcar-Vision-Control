"""Verify the C primary algorithms and E brake provenance without mutation."""
from pathlib import Path
import argparse,hashlib,json

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(root,name):return (root/name).read_text(encoding='utf-8')
def require(test,message):
    if not test:raise SystemExit('FAIL '+message)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--c-base',type=Path,required=True)
    p.add_argument('--e-base',type=Path,required=True)
    args=p.parse_args();c=args.c_base;e=args.e_base;root=Path(__file__).resolve().parents[1]
    modified={'demo/user/src/main.c','demo/user/src/isr.c','demo/user/inc/main.h',
              'demo/user/inc/track_control.h','demo/user/src/track_control.c',
              'demo/user/inc/track_stop_diagnostics.h'}
    preserved=[]
    for directory in ['demo/user','libraries']:
        for path in (c/directory).rglob('*'):
            name=path.relative_to(c).as_posix()
            if path.is_file() and name not in modified:
                require((root/name).is_file() and digest(path)==digest(root/name),'C file differs: '+name)
                preserved.append(name)
    exact_e=['demo/user/inc/track_stripe_brake.h','demo/user/src/track_stripe_brake.c',
             'demo/user/src/isr.c','demo/user/inc/main.h','demo/user/src/track_control.c',
             'demo/user/inc/track_stop_diagnostics.h','demo/mdk/rt1064.uvprojx',
             'tools/test_stripe_brake.c','tools/test_stripe_brake.py']
    for name in exact_e:require(digest(root/name)==digest(e/name),'E brake integration differs: '+name)
    name='demo/user/inc/track_control.h'
    require(read(root,name)==read(c,name).replace('    TRACK_STOP_RING\n','    TRACK_STOP_RING,\n    TRACK_STOP_STRIPE\n'),'C speed/config changed')
    name='demo/user/src/track_control.c'
    require(read(root,name)==read(c,name).replace('reason <= TRACK_STOP_RING','reason <= TRACK_STOP_STRIPE'),'normal track control changed')
    name='demo/user/inc/track_stop_diagnostics.h'
    require(read(root,name)==read(c,name).replace('reason > TRACK_STOP_RING','reason > TRACK_STOP_STRIPE'),'normal stop diagnostics changed')
    require('ring_third_left_lean=0.06f' in read(root,'demo/user/src/ring_corner.c'),'user lean missing')
    require('#define TRACK_SPEED_STRAIGHT 90.0f' in read(root,'demo/user/inc/track_control.h'),'user straight speed missing')
    require('#define TRACK_SPEED_CURVE_MIN 70.0f' in read(root,'demo/user/inc/track_control.h'),'user curve speed missing')
    # Display-only C overlay remains intact after the independent stripe UI.
    name='demo/user/src/main.c';merged=read(root,name);original=read(c,name)
    start='static void screen_draw_track_overlay(void)';end='static track_ring_vision_state_t ring_vision_model;'
    require(merged[merged.index(start):merged.index(end)]==original[original.index(start):original.index(end)],'C track overlay changed')
    require(merged.count('FW=0.28.0')==3 and 'FW=0.27.0' not in merged,'firmware versions not synchronized')
    print(json.dumps({'passed':True,'c_files_identical':len(preserved),'e_brake_files_identical':len(exact_e),
                      'speed_straight':90,'speed_curve_min':70,'third_left_lean':0.06},ensure_ascii=False))

if __name__=='__main__':main()

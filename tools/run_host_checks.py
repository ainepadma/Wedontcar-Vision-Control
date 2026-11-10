"""Run host checks with console failures instead of Windows CRT dialogs."""
from pathlib import Path
import argparse,json,os,subprocess,sys

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--full',action='store_true',help='Include historical PS1 checks; known failures are reported.')
    args=parser.parse_args();root=Path(__file__).resolve().parents[1]
    output=root/'demo/mdk/Objects/release-checks';output.mkdir(parents=True,exist_ok=True)
    include=output/'include';include.mkdir(exist_ok=True)
    (include/'assert.h').write_text('''#ifndef HOST_CHECK_SAFE_ASSERT_H
#define HOST_CHECK_SAFE_ASSERT_H
#include <stdio.h>
#include <stdlib.h>
#define assert(test) do {if(!(test)){fprintf(stderr,"FAIL %s:%d: %s\\n",__FILE__,__LINE__,#test);exit(EXIT_FAILURE);}}while(0)
#endif
''')
    environment=os.environ.copy()
    environment['CPATH']=str(include)+(os.pathsep+environment['CPATH'] if environment.get('CPATH') else '')
    environment['PYTHONIOENCODING']='utf-8'
    names=['test_ring_third_repair.ps1','test_ring_road_continuation.ps1','test_ring_revision4.ps1','test_ring_visual_exit.ps1',
           'test_track_control.ps1','test_motor_switch.ps1','test_pid.ps1']
    if args.full:names=[p.name for p in sorted((root/'tools').glob('test_*.ps1'))]
    names+=['test_stripe_brake.py','test_stop_diagnostics.py','test_steering_export.py']
    results=[]
    for name in names:
        path=root/'tools'/name
        command=([sys.executable,str(path)] if path.suffix=='.py' else
                 ['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(path),'-Compiler','gcc'])
        try:
            result=subprocess.run(command,cwd=root,env=environment,capture_output=True,timeout=120)
            code=result.returncode;data=result.stdout+result.stderr
        except subprocess.TimeoutExpired as error:
            code=-999;data=(error.stdout or b'')+(error.stderr or b'')
        (output/(name+'.log')).write_bytes(data)
        results.append({'test':name,'exit_code':code,'passed':code==0,'log':name+'.log'})
        print(('PASS' if code==0 else 'FAIL')+' '+name,flush=True)
    (output/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Results: '+str(output/'results.json'))
    return 0 if all(row['passed'] for row in results) else 1

if __name__=='__main__':sys.exit(main())

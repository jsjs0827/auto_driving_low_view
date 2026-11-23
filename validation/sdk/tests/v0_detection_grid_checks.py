#!/usr/bin/env python3
"""Check the actual decoder grid fragment against saved V0 X5 tail cases.

The expected values are external captures. This compiles the actual source
fragment and mapping function, retaining separate FP32 RawBox/Map call
boundaries. Target model and full-dataset verification remain separate.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import struct
import subprocess


def sha(b):return hashlib.sha256(b).hexdigest()
def bits(x):return struct.unpack('<I',struct.pack('<f',x))[0]


def function(text,name):
    start=text.index(name+'(')
    start=text.rfind('\n',0,start)+1
    opened=text.index('{',start);end=opened+1;depth=1
    while depth:
        if text[end]=='{':depth+=1
        elif text[end]=='}':depth-=1
        end+=1
    return text[start:end]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project',type=Path,default=Path(__file__).resolve().parents[3])
    p.add_argument('--runner',type=Path)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();project=a.project.resolve(strict=True);out=a.out.resolve()
    if out.is_relative_to(project):p.error('--out must be outside source tree')
    out.mkdir(parents=True,exist_ok=False)
    runner=a.runner or project/'features/inference/backend/src/runner.cpp'
    plain=subprocess.check_output(['cat',str(runner)])
    text=plain.decode();post=function(text,'PostProcessDetections')
    start=post.index('const float x1 =')
    end=post.index('candidates.push_back(det);',start)
    grid=post[start:end].rstrip()
    mapping=function(text,'MapDetectionToOriginal')
    # noinline avoids introducing contractions across the real RawBox storage
    # and mapping boundary. No arithmetic in either extracted block is edited.
    cpp='''#include <algorithm>
struct Box {float x=0,y=0,w=0,h=0;};
struct RawDetection {Box box;};
struct LetterboxMeta {float x_shift,y_shift,x_scale,y_scale;};
__attribute__((noinline,noclone))
'''+mapping+'''
__attribute__((noinline,noclone)) RawDetection make_raw(const float* ltrb,int gh,int gw,int stride) {
    RawDetection det;
'''+grid+'''
    return det;
}
extern "C" void coordinates(const float* distances,int row,int col,int stride,float* result) {
    const auto raw=make_raw(distances,row,col,stride);
    LetterboxMeta meta{150,0,480.f/544.f,480.f/544.f};
    const auto mapped=MapDetectionToOriginal(raw,meta,640,544);
    result[0]=mapped.box.x;result[1]=mapped.box.y;
    result[2]=mapped.box.x+mapped.box.w;result[3]=mapped.box.y+mapped.box.h;
}
'''
    source=out/'actual_grid_mapping.cpp';source.write_text(cpp)
    library=out/'grid_mapping.so'
    argv=['g++','-std=c++17','-O3','-march=native','-fno-fast-math','-ffp-contract=fast','-shared','-fPIC',str(source),'-o',str(library)]
    subprocess.run(argv,check=True)
    loaded=ctypes.CDLL(str(library));call=loaded.coordinates
    call.argtypes=[ctypes.POINTER(ctypes.c_float),ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_float)]
    call.restype=None
    fixture=Path(__file__).with_name('v0_detection_grid_golden.json');golden=json.loads(fixture.read_text())
    rows=[]
    for c in golden['cases']:
        src=(ctypes.c_float*4)(*c['ltrb']);value=(ctypes.c_float*4)()
        call(src,c['row'],c['col'],c['stride'],value)
        actual=list(value);expected=c['expected_xyxy']
        rows.append({'name':c['name'],'actual':actual,'expected':expected,
                     'exact_fp32_bits_equal':all(bits(x)==bits(y) for x,y in zip(actual,expected)),
                     'ulp_errors':[abs(bits(x)-bits(y)) for x,y in zip(actual,expected)]})
    result={'schema':'detseg.v0-detection-grid-regression.v1','scope':'actual source grid and mapping extraction against external V0 X5 coordinates; host native FMA build, no BPU or timing claim',
            'runner_path':str(runner),'runner_plaintext_sha256':sha(plain),'grid_fragment_sha256':sha(grid.encode()),
            'mapping_function_sha256':sha(mapping.encode()),'fixture_sha256':sha(fixture.read_bytes()),
            'compiler_argv':argv,'library_sha256':sha(library.read_bytes()),'rows':rows,
            'passed':all(r['exact_fp32_bits_equal'] for r in rows)}
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},ensure_ascii=False))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':main()

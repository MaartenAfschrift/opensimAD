"""Fresh MATLAB/Python full-model parity, run from any directory.

python tests/validate_parity.py --casadi-matlab C:/path/to/casadi
Requires MATLAB/OpenSim Java bindings and the bundled GenF.exe for the oracle.
"""
import argparse
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import casadi as ca
import numpy as np
import opensim as osim
from scipy.io import loadmat

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from opensimad import generate_external_function, build
from opensimad.verification import reference_inverse_dynamics
from test_python import all_options


def compare_metadata(left, right, path='IO'):
    if isinstance(left, dict):
        if set(left) != set(right):
            raise AssertionError(f'{path}: field mismatch {set(left)^set(right)}')
        for key in left:
            compare_metadata(left[key],right[key],path+'.'+key)
    else:
        np.testing.assert_array_equal(np.asarray(left).reshape(-1),np.asarray(right).reshape(-1),err_msg=path)


def matlab_string(value):
    return "'" + str(value).replace('\\','/').replace("'","''") + "'"


def matlab_options():
    return '''
forces=struct('body','torso','point_in_body',[-.1,.3,0],'name','back_push','reference_frame','ground');
moments(1)=struct('body','tibia_l','name','exo_shank_l','reference_frame','tibia_l');
moments(2)=struct('body','calcn_l','name','exo_foot_l','reference_frame','tibia_l');
positions(1)=struct('body','tibia_l','point_in_body',[0,-.012,0],'name','left_shin');
positions(2)=struct('body','tibia_r','point_in_body',[0,-.012,0],'name','right_shin');
orientation=struct('body','calcn_r','reference_frame','tibia_r','name','ankle_quat');
velocity=positions(1);
'''


def validate(root, matlab, casadi_path):
    osim.Logger.setLevelString('error')
    repo = build.REPOSITORY
    summary = []
    for model_name in ('Hamner_modified','LaiArnold_modified'):
        source=repo/'examples'/f'{model_name}.osim'
        oracle=root/model_name/'matlab'
        oracle.mkdir(parents=True,exist_ok=True)
        script=root/model_name/'oracle.m'
        script.write_text(f'addpath({matlab_string(repo/"utilities")});\naddpath({matlab_string(casadi_path)});\n'
                          + matlab_options() +
                          f'generateExternalFunction({matlab_string(source)},{matlab_string(oracle)},[],[],forces,moments,'
                          'positions,orientation,velocity,velocity,true,true,true,true,'
                          "'F_reference','Visual Studio 18 2026',false,false,false,true);\n",encoding='utf-8')
        build.run([matlab,'-batch',f'run({matlab_string(script)})'], 'MATLAB reference generation',root)
        opts=all_options()
        result=generate_external_function(source,root/model_name/'python',output_filename='F_python',**opts)
        artifacts=result['artifacts']
        compare_metadata(loadmat(oracle/'F_reference_IO.mat',simplify_cells=True)['IO'],
                         loadmat(artifacts['mat'],simplify_cells=True)['IO'])
        reference=ca.Function.load(str(oracle/'F_reference.casadi'))
        symbolic=ca.Function.load(artifacts['casadi'])
        external=ca.external('F',artifacts['dll'])
        io=result['io']
        errors={'outputs':0.,'jacobian':0.,'dynamic_id':0.}
        for k in range(8):
            x=.03*np.sin(np.arange(io['input']['nInputs'])+k)
            if 'pelvis_ty' in io['input']['Qs']:
                x[io['input']['Qs']['pelvis_ty']]=.95
            y=np.array(symbolic(x)).reshape(-1)
            r=np.array(reference(x)).reshape(-1)
            j=np.array(symbolic.jacobian()(x,y))
            jr=np.array(reference.jacobian()(x,r))
            np.testing.assert_allclose(y,r,atol=1e-9,rtol=1e-10)
            np.testing.assert_allclose(j,jr,atol=1e-9,rtol=1e-10)
            np.testing.assert_allclose(y,np.array(external(x)).reshape(-1),atol=1e-9,rtol=1e-10)
            np.testing.assert_allclose(j,np.array(external.jacobian()(x,y)),atol=1e-9,rtol=1e-10)
            id_expected=reference_inverse_dynamics(source,io,x,opts)
            id_error=float(np.max(abs(y[:len(io['coordi'])]-id_expected)))
            np.testing.assert_allclose(y[:len(io['coordi'])],id_expected,atol=1e-6,rtol=1e-8)
            errors={key:max(errors[key],value) for key,value in
                    [('outputs',float(np.max(abs(y-r)))),('jacobian',float(np.max(abs(j-jr)))),('dynamic_id',id_error)]}
        # Dynamics-only independently exercises absence of all optional outputs.
        plain=generate_external_function(source,root/model_name/'plain',output_filename='F_plain',no_dll=True)
        plain_f=ca.Function.load(plain['artifacts']['casadi'])
        x_plain=x[:3*len(io['coordi'])]
        x_zero=x.copy(); x_zero[3*len(io['coordi']):]=0
        np.testing.assert_allclose(np.array(plain_f(x_plain)).reshape(-1),np.array(symbolic(x_zero)).reshape(-1)[:len(io['coordi'])],atol=1e-9,rtol=1e-10)
        summary.append({'model':model_name,'states':8,'metadata':'exact','max_errors':errors,'verification':result['verification']})
        print(json.dumps(summary[-1]),flush=True)
    (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--casadi-matlab',required=True,type=Path)
    parser.add_argument('--matlab',default=shutil.which('matlab'))
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not args.matlab:
        parser.error('MATLAB executable was not found')
    if args.output:
        args.output.resolve().mkdir(parents=True,exist_ok=True)
        validate(args.output.resolve(),args.matlab,args.casadi_matlab.resolve())
    else:
        with tempfile.TemporaryDirectory(prefix='oa_parity_') as directory:
            validate(Path(directory),args.matlab,args.casadi_matlab.resolve())

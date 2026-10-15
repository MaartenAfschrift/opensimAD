"""Run: python -m unittest discover -s tests -p test_python.py -v.

Set OPENSIMAD_INTEGRATION=1 for SDK/compiler/OpenSim numerical tests.
"""
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import casadi as ca
import numpy as np
import opensim as osim
from scipy.io import loadmat

from opensimad import generate_external_function
from opensimad import build
from opensimad.generator import function_code, matlab_indices, write_cpp
from opensimad.verification import reference_inverse_dynamics, reference_model, set_state, verify_inverse_dynamics
from opensimad.generator import vector

osim.Logger.setLevelString('error')
REPO = build.REPOSITORY
INTEGRATION = os.environ.get('OPENSIMAD_INTEGRATION') == '1'


def options(**overrides):
    result = {k: [] for k in ('joints_order', 'coordinates_order', 'input_3d_body_forces',
                             'input_3d_body_moments', 'export_3d_positions', 'export_3d_orientations',
                             'export_3d_velocities', 'export_3d_velocities_proj_ground')}
    result.update(export_grfs=False, export_grms=False, export_separate_grfs=False, export_contact_powers=False)
    result.update(overrides)
    return result


def multiplier(function, scale):
    result = osim.MultiplierFunction(function, scale)
    function.thisown = False  # MultiplierFunction takes ownership of its child.
    return result


def all_options():
    point = dict(body='tibia_l', point_in_body=[0, -.012, 0], name='left_shin')
    return options(
        input_3d_body_forces=[dict(body='torso', point_in_body=[-.1, .3, 0], name='back_push', reference_frame='ground')],
        input_3d_body_moments=[dict(body='tibia_l', name='exo_shank_l', reference_frame='tibia_l'),
                               dict(body='calcn_l', name='exo_foot_l', reference_frame='tibia_l')],
        export_3d_positions=[point, dict(body='tibia_r', point_in_body=[0, -.012, 0], name='right_shin')],
        export_3d_orientations=[dict(body='calcn_r', reference_frame='tibia_r', name='ankle_quat')],
        export_3d_velocities=[point], export_3d_velocities_proj_ground=[point],
        export_grfs=True, export_grms=True, export_separate_grfs=True, export_contact_powers=True)


def small_model(path, kind='PinJoint', polynomial=None, locked=False, gravity=(1, -9.81, 2)):
    model = osim.Model()
    model.setName('fixture')
    model.set_gravity(osim.Vec3(*gravity))
    body = osim.Body('body', 2, osim.Vec3(0, -.3, 0), osim.Inertia(.1, .2, .3))
    model.addBody(body)
    args = ('joint', model.getGround(), osim.Vec3(), osim.Vec3(), body, osim.Vec3(), osim.Vec3())
    if kind == 'CustomJoint':
        transform = osim.SpatialTransform()
        for i in range(3):
            transform.updTransformAxis(i).set_axis(osim.Vec3(0, 0, 1) if i == 0 else
                                                   (osim.Vec3(1, 0, 0) if i == 1 else osim.Vec3(0, 1, 0)))
        axis = transform.updTransformAxis(0)
        names = osim.ArrayStr()
        names.append('angle')
        function = polynomial or osim.LinearFunction(1, 0)
        nested = function.getFunction() if function.getConcreteClassName() == 'MultiplierFunction' else function
        multivariate = nested.getConcreteClassName() == 'MultivariatePolynomialFunction'
        if multivariate:
            names.append('angle2')
        axis.setCoordinateNames(names)
        axis.set_function(function)
        if multivariate:
            axis2 = transform.updTransformAxis(1)
            names2 = osim.ArrayStr(); names2.append('angle2')
            axis2.setCoordinateNames(names2)
            axis2.set_function(osim.LinearFunction(1, 0))
        joint = osim.CustomJoint(*args, transform)
        joint.finalizeFromProperties()
    else:
        joint = getattr(osim, kind)(*args)
    if kind == 'PlanarJoint':
        for i in range(3):
            joint.upd_coordinates(i).setName('coord' + str(i))
    else:
        joint.upd_coordinates(0).setName('angle')
        joint.upd_coordinates(0).set_locked(locked)
    model.addJoint(joint)
    if locked:
        other = osim.Body('other', 1, osim.Vec3(0, -.1, 0), osim.Inertia(.1))
        model.addBody(other)
        free = osim.PinJoint('free', body, osim.Vec3(), osim.Vec3(), other, osim.Vec3(), osim.Vec3())
        free.upd_coordinates(0).setName('free_angle')
        model.addJoint(free)
    model.finalizeConnections()
    model.printToXML(str(path))
    return path


class CoreTests(unittest.TestCase):
    def test_matlab_indices_do_not_shift_counts(self):
        actual = matlab_indices({'input': {'nInputs': 9, 'Qs': {'q': 0}}, 'jointi': {'weld': []}})
        self.assertEqual(actual['input']['nInputs'], 9)
        self.assertEqual(actual['input']['Qs']['q'], 1)
        self.assertEqual(actual['jointi']['weld'].size, 0)

    def test_polynomial_multiplier_uses_its_own_coefficients(self):
        coeffs = osim.Vector(7, 0.)
        for i in range(7):
            coeffs.set(i, (i+1)/100)
        lines = []
        result = function_code(multiplier(osim.PolynomialFunction(coeffs), 2), 'test', lines.append)
        self.assertIn('MultiplierFunction(new PolynomialFunction(test_coeffs)', result)
        self.assertIn('0.07000000000000000666', '\n'.join(lines))

    def test_supported_functions(self):
        functions = [osim.Constant(2), osim.LinearFunction(1, 2), osim.PolynomialFunction(osim.Vector(3, .1)),
                     osim.MultivariatePolynomialFunction(osim.Vector(3, .2), 2, 1),
                     multiplier(osim.Constant(2), 3),
                     multiplier(osim.MultivariatePolynomialFunction(osim.Vector(3, .1), 2, 1), 2)]
        for function in functions:
            with self.subTest(function=function.getConcreteClassName()):
                self.assertTrue(function_code(function, 'test', lambda _: None))
        with self.assertRaises(ValueError):
            function_code(osim.SimmSpline(), 'test', lambda _: None)

    def test_multivariate_model_dependency_names(self):
        with tempfile.TemporaryDirectory() as d:
            path = small_model(Path(d)/'model.osim','CustomJoint',
                               multiplier(osim.MultivariatePolynomialFunction(osim.Vector(3,.1),2,1),2))
            io = write_cpp(path,d,'F',options())
            self.assertEqual(list(io['coordi']),['angle','angle2'])
            source=(Path(d)/'F.cpp').read_text()
            self.assertIn('names_joint_0.append("angle2")',source)
            self.assertIn('new MultivariatePolynomialFunction(st_joint_0_coeffs, 2, 1)',source)

    def test_generator_full_mapping_and_independent_switches(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as d:
            root = Path(d)
            full = write_cpp(REPO / 'examples/Hamner_modified.osim', root, 'F', all_options())
            self.assertEqual(full['input']['nInputs'], 102)
            self.assertEqual(full['input']['Forces']['back_push'], [93, 94, 95])
            self.assertEqual(full['position']['left_shin'], [31, 32, 33])
            self.assertEqual(full['orientation']['ankle_quat'], [37, 38, 39, 40])
            self.assertEqual(json.loads((root/'F_IO.json').read_text())['IO'], full)
            matlab = loadmat(root/'F_IO.mat', simplify_cells=True)['IO']
            self.assertEqual(matlab['input']['nInputs'], 102)
            self.assertEqual(matlab['input']['Qs']['pelvis_tilt'], 1)
            for key, field in (('export_grms','GRMs'), ('export_separate_grfs','GRFs'),
                               ('export_contact_powers','P_contact_deformation_y')):
                io = write_cpp(REPO/'examples/Hamner_modified.osim', root, 'F', options(**{key: True}))
                self.assertIn(field, io)
                self.assertNotIn('position', io)

    def test_invalid_order_and_references(self):
        with tempfile.TemporaryDirectory() as d:
            model = small_model(Path(d)/'pin.osim')
            for bad in (options(joints_order=['joint'], coordinates_order=['wrong']),
                        options(export_3d_positions=[dict(body='wrong',name='p',point_in_body=[0,0,0])]),
                        options(export_3d_positions=[dict(body='body',name='p',point_in_body=[0,0])])):
                with self.assertRaises(ValueError):
                    write_cpp(model, d, 'F', bad)

    def test_workspace_and_command_failures(self):
        cwd = Path.cwd()
        with tempfile.TemporaryDirectory() as d, patch.dict(os.environ, {'OPENSIMAD_BUILD_ROOT': d}):
            with self.assertRaisesRegex(RuntimeError, 'deliberate'):
                with build.workspace() as work:
                    build.run([sys.executable, '-c', 'raise RuntimeError("deliberate")'], 'test stage', work)
            self.assertEqual(list(Path(d).iterdir()), [])
        self.assertEqual(Path.cwd(), cwd)
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(RuntimeError, 'missing artifact'):
                build.require_file(Path(d)/'absent', 'test')
            lock = Path(d)/'lockFile.txt'
            lock.write_text('other owner')
            with self.assertRaises(TimeoutError):
                with build.recorder_lock(d, timeout=0):
                    pass
            self.assertEqual(lock.read_text(), 'other owner')
            lock.unlink()
            with self.assertRaises(RuntimeError):
                with build.recorder_lock(d):
                    (Path(d)/'foo.py').write_text('stale')
                    raise RuntimeError('test')
            self.assertEqual(list(Path(d).iterdir()), [])


@unittest.skipUnless(INTEGRATION, 'Set OPENSIMAD_INTEGRATION=1 for builds')
class IntegrationTests(unittest.TestCase):
    def test_build_stage_failures_cleanup_and_retry(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            source=root/'bad.cpp'
            for contents,stage in [('invalid C++', 'Expression graph compilation'),
                                    ('int main(){return 7;}', 'Expression graph execution'),
                                    ('int main(){return 0;}', 'missing artifact')]:
                source.write_text(contents)
                with self.assertRaisesRegex(RuntimeError,stage):
                    with build.workspace() as work:
                        build.build_graph(source,work,None,False)
                self.assertFalse((REPO/'opensimAD-install/bin/lockFile.txt').exists())
                self.assertFalse((REPO/'opensimAD-install/bin/foo.py').exists())
            source.write_text('#include <fstream>\nint main(){std::ofstream("foo.py") << "retry";}')
            with build.workspace() as work:
                build.build_graph(source,work,None,False)
                self.assertEqual((work/'foo.py').read_text(),'retry')
                (work/'foo.py').write_text('invalid python syntax !')
                with self.assertRaisesRegex(RuntimeError,'CasADi code generation'):
                    build.run([sys.executable,str(REPO/'opensimad/codegen.py'),str(work),'1'],
                              'CasADi code generation',work)
                (work/'foo_jac.c').write_text('invalid C source')
                with self.assertRaisesRegex(RuntimeError,'External function compilation'):
                    build.build_library(work,root,'F',None,False)

    def test_small_models_dynamic_id_kinematics_and_derivatives(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as d:
            root = Path(d)
            coeffs = osim.Vector(7, .001)
            for case, (kind, polynomial, locked) in enumerate([('PinJoint',None,False), ('PlanarJoint',None,False),
                                             ('CustomJoint',multiplier(osim.PolynomialFunction(coeffs),2),False),
                                             ('CustomJoint',osim.MultivariatePolynomialFunction(osim.Vector(3,.1),2,1),False),
                                             ('CustomJoint',multiplier(osim.MultivariatePolynomialFunction(osim.Vector(3,.1),2,1),2),False),
                                             ('PinJoint',None,True)]):
                model = small_model(root/'model.osim',kind,polynomial,locked)
                free_body = 'other' if locked else 'body'
                point = dict(body=free_body,point_in_body=[0,-.2,0],name='p')
                opts = dict(input_3d_body_forces=[dict(body=free_body,point_in_body=[0,-.2,0],name='push',reference_frame='body')],
                            input_3d_body_moments=[dict(body=free_body,name='turn',reference_frame='ground')],
                            export_3d_positions=[point],export_3d_velocities=[point],
                            export_3d_orientations=[dict(body=free_body,name='quat',reference_frame='ground')],
                            export_3d_velocities_proj_ground=[point])
                result = generate_external_function(model,root/'out',output_filename='F_case' + str(case),
                                                     verify_id=True,second_order_derivatives=kind=='PinJoint' and not locked,**opts)
                io=result['io']
                f=ca.Function.load(result['artifacts']['casadi'])
                dll=ca.external('F', result['artifacts']['dll'])
                x=.03*np.cos(np.arange(io['input']['nInputs']))
                actual=np.array(f(x)).ravel()
                np.testing.assert_allclose(actual,np.array(dll(x)).ravel(),atol=1e-10,rtol=1e-10)
                expected=reference_inverse_dynamics(model,io,x,opts)
                np.testing.assert_allclose(actual[:len(io['coordi'])],expected,atol=1e-6,rtol=1e-8)
                # Kinematics are independently evaluated with ordinary OpenSim.
                ordinary=reference_model(model)
                state=ordinary.initSystem()
                set_state(ordinary,state,io,x)
                body=ordinary.getBodySet().get(free_body)
                station=osim.Vec3(*point['point_in_body'])
                position=body.findStationLocationInGround(state,station)
                velocity=body.findStationVelocityInGround(state,station)
                projected=osim.Vec3(position.get(0),0,position.get(2))
                in_body=ordinary.getGround().findStationLocationInAnotherFrame(state,projected,body)
                projected_velocity=body.findStationVelocityInGround(state,in_body)
                quat=body.findTransformBetween(state,ordinary.getGround()).R().convertRotationToQuaternion()
                for field,value in [('position',vector(position)),('velocity',vector(velocity)),
                                    ('velocityProj',vector(projected_velocity)),('orientation',[quat.get(i) for i in range(4)])]:
                    label='quat' if field=='orientation' else 'p'
                    np.testing.assert_allclose(actual[io[field][label]],value,atol=1e-9,rtol=1e-9)
                jac=np.array(f.jacobian()(x,actual))
                fd=np.column_stack([(np.array(f(x+np.eye(len(x))[i]*1e-6)).ravel()-np.array(f(x-np.eye(len(x))[i]*1e-6)).ravel())/2e-6 for i in range(len(x))])
                np.testing.assert_allclose(jac,fd,atol=1e-5,rtol=1e-5)
                if kind=='PinJoint' and not locked:
                    sx=ca.MX.sym('x',len(x))
                    hd=ca.Function('hd',[sx],[ca.hessian(ca.sum1(dll(sx)),sx)[0]])
                    hs=ca.Function('hs',[sx],[ca.hessian(ca.sum1(f(sx)),sx)[0]])
                    np.testing.assert_allclose(np.array(hd(x)),np.array(hs(x)),atol=1e-8,rtol=1e-8)
                    del hd, hs
                del dll

    def test_path_build_failure_retry_and_serialized_only(self):
        cwd = Path.cwd()
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            builds=root/'build with spaces'
            builds.mkdir()
            with patch.dict(os.environ,OPENSIMAD_BUILD_ROOT=str(builds)):
                model=small_model(root/'pin.osim')
                with self.assertRaisesRegex(RuntimeError,'Expression graph configuration'):
                    generate_external_function(model,root/'out',compiler='Visual Studio 99 9999',verify_id=False)
                self.assertEqual(list(builds.iterdir()),[])
                long_name='F_Falisse_et_al_2022_slope07_torso125pct-with spaces'
                result=generate_external_function(model,root/'output with spaces'/('deep_'*12),output_filename=long_name,verify_id=True)
                self.assertIn((long_name+'.dll').encode(),Path(result['artifacts']['lib']).read_bytes())
                # A stale DLL must not be verified during a serialized-only call.
                Path(result['artifacts']['dll']).write_bytes(b'invalid stale DLL')
                serial=generate_external_function(model,Path(result['artifacts']['dll']).parent,output_filename=long_name,no_dll=True)
                self.assertNotIn('dll',serial['artifacts'])
                self.assertEqual(list(builds.iterdir()),[])
                too_long=root/('long_'*30)
                too_long.mkdir()
                with patch.dict(os.environ,OPENSIMAD_BUILD_ROOT=str(too_long)):
                    with self.assertRaisesRegex(ValueError,'too long'):
                        with build.workspace():
                            pass
        self.assertEqual(Path.cwd(),cwd)


if __name__ == '__main__':
    unittest.main()

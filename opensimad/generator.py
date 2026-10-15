"""Reconstruct the supported MATLAB OpenSimAD model and its packed interface.

Port of utilities/writeCppFile.m (Apache-2.0, Antoine Falisse/Lars D'Hondt).
The Python and MATLAB metadata share field names, but Python indices start at 0.
"""
import json
import re
import warnings
from pathlib import Path

import numpy as np
import opensim as osim
from scipy.io import savemat


def items(collection):
    return [collection.get(i) for i in range(collection.getSize())]


def vector(value):
    return [float(value.get(i)) for i in range(value.size() if hasattr(value, 'size') else 3)]


def number(value):
    return f'{float(value):.20f}'


def vec3(value):
    return 'Vec3(' + ', '.join(number(x) for x in value) + ')'


def identifier(name):
    if not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', name):
        raise ValueError(f'Name is not a supported C++/MATLAB identifier: {name!r}')
    return name


def matlab_indices(value):
    """Convert index leaves, preserving counts and zero-coordinate weld joints."""
    if isinstance(value, dict):
        return {k: v if k == 'nInputs' else matlab_indices(v) for k, v in value.items()}
    if isinstance(value, list):
        return np.array([v + 1 for v in value], dtype=float)
    return float(value + 1)


def function_code(function, stem, emit, *, multiplied=False):
    kind = function.getConcreteClassName()
    obj = getattr(osim, kind).safeDownCast(function)
    if kind == 'Constant':
        return f'new Constant({number(obj.getValue())})'
    if kind == 'LinearFunction' and not multiplied:
        # Retain the reference's four-decimal linear-function representation.
        return f'new LinearFunction({obj.getSlope():.4f}, {obj.getIntercept():.4f})'
    if kind in ('PolynomialFunction', 'MultivariatePolynomialFunction'):
        coefficients = vector(obj.getCoefficients())
        if kind == 'PolynomialFunction' and len(coefficients) not in (2, 3, 4, 5, 7):
            raise ValueError('Supported univariate coefficient counts: 2, 3, 4, 5, 7')
        emit(f'Vector {stem}_coeffs({len(coefficients)});')
        for i, coefficient in enumerate(coefficients):
            emit(f'{stem}_coeffs[{i}] = {number(coefficient)};')
        arguments = f'{stem}_coeffs'
        if kind == 'MultivariatePolynomialFunction':
            arguments += f', {obj.getDimension()}, {obj.getOrder()}'
        return f'new {kind}({arguments})'
    if kind == 'MultiplierFunction' and not multiplied:
        nested = function_code(obj.getFunction(), stem, emit, multiplied=True)
        return f'new MultiplierFunction({nested}, {number(obj.getScale())})'
    raise ValueError(f'Unsupported transform function: {kind}')


HELPERS = r'''
template<typename T> T value(const Recorder& e) { return e; }
template<> double value(const Recorder& e) { return e.getValue(); }
SimTK::Array_<int> getIndicesOSInSimbody(const Model& model) {
    auto s = model.getWorkingState();
    const auto names = model.getStateVariableNames();
    SimTK::Array_<int> indices(s.getNQ());
    s.updQ() = 0;
    for (int i = 0; i < s.getNQ(); ++i) {
        s.updQ()[i] = SimTK::NaN;
        const auto values = model.getStateVariableValues(s);
        for (int j = 0; j < names.size(); ++j) {
            if (SimTK::isNaN(values[j])) {
                s.updQ()[i] = 0;
                indices[i] = j/2;
                break;
            }
        }
    }
    return indices;
}
SimTK::Array_<int> getIndicesSimbodyInOS(const Model& model) {
    auto forward = getIndicesOSInSimbody(model);
    SimTK::Array_<int> inverse(forward.size());
    for (int i = 0; i < forward.size(); ++i)
        for (int j = 0; j < forward.size(); ++j)
            if (forward[j] == i) { inverse[i] = j; break; }
    return inverse;
}
'''


def write_cpp(model_path, output_dir, name, options):
    model = osim.Model(str(model_path))
    model.initSystem()
    if model.getConstraintSet().getSize():
        warnings.warn('OpenSimAD ignores OpenSim constraints.', stacklevel=2)
    bodies = [b for b in items(model.getBodySet()) if b.getName() not in ('patella_l', 'patella_r')]
    joints = [j for j in items(model.getJointSet()) if j.getName() not in ('patellofemoral_l', 'patellofemoral_r')]
    joint_by_name = {j.getName(): j for j in joints}
    order = options['joints_order']
    coorder = options['coordinates_order']
    if not order or not coorder:
        order = list(joint_by_name)
        coorder = [j.get_coordinates(i).getName() for j in joints for i in range(j.numCoordinates())
                   if not j.get_coordinates(i).get_locked()]
    if len(order) != len(joints) or set(order) != set(joint_by_name):
        raise ValueError('joints_order must contain each retained joint exactly once')
    joints = [joint_by_name[j] for j in order]
    coordinates = [j.get_coordinates(i) for j in joints for i in range(j.numCoordinates())
                   if not j.get_coordinates(i).get_locked()]
    if list(coorder) != [c.getName() for c in coordinates]:
        raise ValueError('coordinates_order must match the coordinates of joints_order')
    n = len(coordinates)
    if not n:
        raise ValueError('At least one retained coordinate is required')
    body_names = {b.getName() for b in bodies}
    for obj in bodies + joints + coordinates:
        identifier(obj.getName())
    for joint in joints:
        locked = [joint.get_coordinates(i) for i in range(joint.numCoordinates()) if joint.get_coordinates(i).get_locked()]
        if locked and (joint.getConcreteClassName() != 'PinJoint' or locked[0].getDefaultValue() != 0):
            raise ValueError('Only locked PinJoints at zero default angle are supported; replace others with pose-preserving welds')
    for key in ('input_3d_body_forces', 'input_3d_body_moments', 'export_3d_positions',
                'export_3d_orientations', 'export_3d_velocities', 'export_3d_velocities_proj_ground'):
        selections = options[key]
        names = [identifier(s['name']) for s in selections]
        if len(set(names)) != len(names):
            raise ValueError(f'Duplicate names in {key}')
        for selection in selections:
            if selection['body'] not in body_names:
                raise ValueError(f'Unknown or excluded body: {selection["body"]}')
            if 'reference_frame' in selection:
                ref = selection['reference_frame']
                if ref.lower() != 'ground' and ref not in body_names:
                    raise ValueError(f'Unknown reference frame: {ref}')
            if key != 'input_3d_body_moments' and key != 'export_3d_orientations':
                station = selection['point_in_body']
                if len(station) != 3 or not np.isfinite(station).all():
                    raise ValueError('point_in_body must contain three finite numbers')
    forces = options['input_3d_body_forces']
    moments = options['input_3d_body_moments']
    contacts = []
    for force in items(model.getForceSet()):
        if force.getConcreteClassName() == 'SmoothSphereHalfSpaceForce':
            contact = osim.SmoothSphereHalfSpaceForce.safeDownCast(force)
            # Resolve sockets by semantic name rather than binding-specific array APIs.
            sphere = osim.ContactSphere.safeDownCast(force.getSocket('sphere').getConnecteeAsObject())
            half = osim.ContactHalfSpace.safeDownCast(force.getSocket('half_space').getConnecteeAsObject())
            contacts.append((contact, sphere, half))
        elif osim.Muscle.safeDownCast(force) is None:
            warnings.warn(f'Force component {force.getName()} ({force.getConcreteClassName()}) is not included.', stacklevel=2)
    io = {'input': {'Qs': {}, 'Qdots': {}, 'Qdotdots': {}, 'nInputs': 3*n + 3*(len(forces)+len(moments))},
          'coordi': {c.getName(): i for i, c in enumerate(coordinates)}, 'jointi': {}}
    for i, c in enumerate(coordinates):
        for field, index in (('Qs', 2*i), ('Qdots', 2*i+1), ('Qdotdots', 2*n+i)):
            io['input'][field][c.getName()] = index
    io['jointi'] = {j.getName(): [io['coordi'][j.get_coordinates(i).getName()] for i in range(j.numCoordinates())
                                if not j.get_coordinates(i).get_locked()] for j in joints}
    io['jointi']['translations'] = [i for i, c in enumerate(coordinates) if c.getMotionType() == osim.Coordinate.Translational]
    io['jointi']['rotations'] = [i for i in range(n) if i not in io['jointi']['translations']]
    lines = []
    emit = lines.append
    headers = ['Simulation/Model/Model', 'Simulation/SimbodyEngine/PinJoint', 'Simulation/SimbodyEngine/WeldJoint',
               'Simulation/SimbodyEngine/PlanarJoint', 'Simulation/SimbodyEngine/CustomJoint',
               'Simulation/SimbodyEngine/SpatialTransform', 'Common/LinearFunction', 'Common/PolynomialFunction',
               'Common/MultivariatePolynomialFunction', 'Common/MultiplierFunction', 'Common/Constant',
               'Simulation/Model/SmoothSphereHalfSpaceForce']
    for header in headers:
        emit(f'#include <OpenSim/{header}.h>')
    emit('#include "SimTKcommon/internal/recorder.h"\n#include <vector>\nusing namespace SimTK;\nusing namespace OpenSim;')
    emit(HELPERS)
    emit('template<typename T> int F_generic(const T** arg, T** res) {')
    emit('auto model = new OpenSim::Model();')
    for body in bodies:
        b = body.getName()
        inertia = body.get_inertia()
        diag = ', '.join(number(inertia.get(i)) for i in range(3))
        emit(f'auto {b} = new OpenSim::Body("{b}", {number(body.getMass())}, {vec3(vector(body.getMassCenter()))}, Inertia({diag}, 0., 0., 0.));')
        emit(f'model->addBody({b});')
    def frame(name):
        return 'model->getGround()' if name.lower() == 'ground' else '*' + name
    for joint in joints:
        j, kind = joint.getName(), joint.getConcreteClassName()
        if kind not in ('CustomJoint', 'PinJoint', 'WeldJoint', 'PlanarJoint'):
            raise ValueError(f'Unsupported joint type: {kind}')
        extra = ''
        if kind == 'CustomJoint':
            emit(f'SpatialTransform st_{j};')
            transform = osim.CustomJoint.safeDownCast(joint).get_SpatialTransform()
            for i in range(6):
                axis = transform.getTransformAxis(i)
                coordinate_names = axis.getCoordinateNamesInArray()
                names = [coordinate_names.get(k) for k in range(coordinate_names.getSize())]
                if names:
                    emit(f'OpenSim::Array<std::string> names_{j}_{i};')
                    for coordinate in names:
                        emit(f'names_{j}_{i}.append("{identifier(coordinate)}");')
                    emit(f'st_{j}[{i}].setCoordinateNames(names_{j}_{i});')
                expr = function_code(axis.get_function(), f'st_{j}_{i}', emit)
                emit(f'st_{j}[{i}].setFunction({expr});')
                emit(f'st_{j}[{i}].setAxis({vec3(vector(axis.get_axis()))});')
            extra = f', st_{j}'
        elif kind == 'PinJoint' and joint.get_coordinates(0).get_locked():
            kind = 'WeldJoint'
        frames = [joint.get_frames(i) for i in range(2)]
        args = []
        for f in frames:
            parent = f.getParentFrame().getName()
            if parent != 'ground' and parent not in body_names:
                raise ValueError(f'Unsupported joint frame parent: {parent}')
            args.extend([frame(parent), vec3(vector(f.get_translation())), vec3(vector(f.get_orientation()))])
        emit(f'auto {j} = new OpenSim::{kind}("{j}", ' + ', '.join(args) + extra + ');')
    for j in order:
        emit(f'model->addJoint({j});')
    for contact, sphere, half in contacts:
        c = identifier(contact.getName())
        sbody, hbody = sphere.getFrame().getName(), half.getFrame().getName()
        if sbody not in body_names or (hbody != 'ground' and hbody not in body_names):
            raise ValueError('Contact frames must be retained bodies or ground')
        emit(f'auto {c} = new SmoothSphereHalfSpaceForce("{c}", {frame(sbody)}, {frame(hbody)});')
        emit(f'Vec3 {c}_location = {vec3(vector(sphere.get_location()))};')
        emit(f'double {c}_radius = {number(sphere.getRadius())};')
        emit(f'{c}->set_contact_sphere_location({c}_location);\n{c}->set_contact_sphere_radius({c}_radius);')
        emit(f'{c}->set_contact_half_space_location({vec3(vector(half.get_location()))});')
        emit(f'{c}->set_contact_half_space_orientation({vec3(vector(half.get_orientation()))});')
        for prop in ('stiffness', 'dissipation', 'static_friction', 'dynamic_friction', 'viscous_friction',
                     'transition_velocity', 'constant_contact_force', 'hertz_smoothing', 'hunt_crossley_smoothing'):
            emit(f'{c}->set_{prop}({number(getattr(contact, "get_" + prop)())});')
        emit(f'{c}->connectSocket_sphere_frame({frame(sbody)});\n{c}->connectSocket_half_space_frame({frame(hbody)});')
        emit(f'model->addComponent({c});')
    emit('auto state = new State(model->initSystem());')
    emit(f'Vector QsUs({2*n});\nfor (int i=0; i<{2*n}; ++i) QsUs[i]=arg[0][i];')
    emit('model->setStateVariableValues(*state, QsUs);\nmodel->realizeVelocity(*state);')
    emit(f'Vector appliedMobilityForces({n}, 0.), knownUdot({n}, 0.), residual({n}, 0.);')
    emit('Vector_<SpatialVec> appliedBodyForces(model->getBodySet().getSize()+1);\nappliedBodyForces.setToZero();')
    emit(f'Vec3 gravity = {vec3(vector(model.get_gravity()))};')
    emit('for (int i=0; i<model->getBodySet().getSize(); ++i) {\nconst auto& b = model->getBodySet().get(i);\nmodel->getMatterSubsystem().addInStationForce(*state,b.getMobilizedBodyIndex(),b.getMassCenter(),b.getMass()*gravity,appliedBodyForces);\n}')
    emit(f'auto osOrder = getIndicesOSInSimbody(*model);\nfor (int i=0; i<{n}; ++i) knownUdot[i]=arg[1][osOrder[i]];')
    for i, (contact, sphere, _) in enumerate(contacts):
        emit(f'auto Force_{i} = {contact.getName()}->getRecordValues(*state);')
        emit(f'SpatialVec GRF_{i}(Vec3(Force_{i}[3],Force_{i}[4],Force_{i}[5]), Vec3(Force_{i}[0],Force_{i}[1],Force_{i}[2]));')
        emit(f'appliedBodyForces[{sphere.getFrame().getName()}->getMobilizedBodyIndex()] += GRF_{i};')
    offset = n
    for field, selections in (('Forces', forces), ('Moments', moments)):
        if selections:
            io['input'][field] = {}
        for i, s in enumerate(selections):
            stem = f'load_{field}_{i}'
            emit(f'Vec3 {stem}(arg[1][{offset}],arg[1][{offset+1}],arg[1][{offset+2}]);')
            if s['reference_frame'].lower() != 'ground':
                emit(f'{stem} = {s["reference_frame"]}->getMobilizedBody().expressVectorInGroundFrame(*state,{stem});')
            body = s['body']
            if field == 'Forces':
                emit(f'model->getMatterSubsystem().addInStationForce(*state,{body}->getMobilizedBodyIndex(),{vec3(s["point_in_body"])},{stem},appliedBodyForces);')
            else:
                emit(f'model->getMatterSubsystem().addInBodyTorque(*state,{body}->getMobilizedBodyIndex(),{stem},appliedBodyForces);')
            io['input'][field][s['name']] = list(range(2*n+offset, 2*n+offset+3))
            offset += 3
    emit('model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,appliedMobilityForces,appliedBodyForces,knownUdot,residual);')
    emit(f'auto simOrder = getIndicesSimbodyInOS(*model);\nfor (int i=0; i<{n}; ++i) res[0][i]=value<T>(residual[simOrder[i]]);')
    output = n
    def export(field, label, expression, size=3, quaternion=False):
        nonlocal output
        io.setdefault(field, {})[label] = output if size == 1 else list(range(output, output+size))
        if size == 1:
            emit(f'res[0][{output}]=value<T>({expression});')
        else:
            accessor = '.get(i)' if quaternion else '[i]'
            emit(f'for (int i=0; i<{size}; ++i) res[0][{output}+i]=value<T>({expression}{accessor});')
        output += size
    for field, key in (('position', 'export_3d_positions'), ('orientation', 'export_3d_orientations'),
                       ('velocity', 'export_3d_velocities'), ('velocityProj', 'export_3d_velocities_proj_ground')):
        for i, s in enumerate(options[key]):
            stem, b = f'out_{field}_{i}', s['body']
            if field == 'orientation':
                ref = frame(s['reference_frame'])
                emit(f'auto {stem} = {b}->getMobilizedBody().findBodyRotationInAnotherBody(*state,({ref}).getMobilizedBody()).convertRotationToQuaternion();')
            elif field == 'velocityProj':
                emit(f'auto {stem}_G = {b}->findStationLocationInGround(*state,{vec3(s["point_in_body"])});\n{stem}_G[1]=0;')
                emit(f'auto {stem}_B = model->getGround().findStationLocationInAnotherFrame(*state,{stem}_G,*{b});')
                emit(f'auto {stem} = {b}->findStationVelocityInGround(*state,{stem}_B);')
            else:
                method = 'Location' if field == 'position' else 'Velocity'
                emit(f'auto {stem} = {b}->findStation{method}InGround(*state,{vec3(s["point_in_body"])});')
            export(field, s['name'], stem, 4 if field == 'orientation' else 3, field == 'orientation')
    def side(contact):
        c = contact.getName().lower()
        if c.startswith('r_') or c.endswith('_r'):
            return 'r'
        if c.startswith('l_') or c.endswith('_l'):
            return 'l'
        raise ValueError(f'Cannot identify contact side: {contact.getName()}')
    if options['export_grfs']:
        emit('Vec3 total_r(0), total_l(0);')
        for i, (contact, _, _) in enumerate(contacts):
            emit(f'total_{side(contact)} += GRF_{i}[1];')
        export('GRFs', 'right_total', 'total_r')
        export('GRFs', 'left_total', 'total_l')
    if options['export_separate_grfs']:
        io.setdefault('GRFs', {})
        for i, (_, sphere, _) in enumerate(contacts):
            export('GRFs', identifier(sphere.getName()), f'GRF_{i}[1]')
    if options['export_grms']:
        emit('Vec3 grm_r(0), grm_l(0), normal(0,1,0);')
        for i, (contact, sphere, _) in enumerate(contacts):
            c, b = contact.getName(), sphere.getFrame().getName()
            emit(f'auto cp_{i} = {b}->findStationLocationInGround(*state,{c}_location) - {c}_radius*normal;\ncp_{i} -= 0.5*cp_{i}[1]*normal;')
            emit(f'auto cpB_{i} = model->getGround().findStationLocationInAnotherFrame(*state,cp_{i},*{b});')
            emit(f'grm_{side(contact)} += ({b}->getMobilizedBody().getBodyTransform(*state)*cpB_{i}) % GRF_{i}[1];')
        export('GRMs', 'right_total', 'grm_r')
        export('GRMs', 'left_total', 'grm_l')
    if options['export_contact_powers']:
        io.setdefault('P_contact_deformation_y', {})
        for i, (contact, sphere, _) in enumerate(contacts):
            emit(f'auto vel_{i} = {sphere.getFrame().getName()}->findStationVelocityInGround(*state,{contact.getName()}_location);')
            export('P_contact_deformation_y', identifier(sphere.getName()), f'vel_{i}[1]*GRF_{i}[1][1]', 1)
    emit('delete state;\ndelete model;\nreturn 0;\n}')
    emit(f'''int main() {{
Recorder x[{2*n}], u[{offset}], tau[{output}];
for (int i=0; i<{2*n}; ++i) x[i] <<= 0;
for (int i=0; i<{offset}; ++i) u[i] <<= 0;
const Recorder* args[2] = {{x,u}};
Recorder* results[1] = {{tau}};
F_generic<Recorder>(args,results);
double values[{output}];
for (int i=0; i<{output}; ++i) tau[i] >>= values[i];
Recorder::stop_recording();
return 0;
}}''')
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / (name + '.cpp')).write_text('\n'.join(lines) + '\n', encoding='utf-8')
    savemat(output_dir / (name + '_IO.mat'), {'IO': matlab_indices(io)}, long_field_names=True)
    (output_dir / (name + '_IO.json')).write_text(json.dumps({'index_base': 0, 'IO': io}, indent=2) + '\n', encoding='utf-8')
    return io

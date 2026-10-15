"""Independent ordinary-OpenSim inverse dynamics and static IDTool check."""
from pathlib import Path

import casadi as ca
import numpy as np
import opensim as osim

from . import build
from .generator import items, vector


def reference_model(model_path):
    """Remove excluded muscle/force components and the documented patella chain."""
    model = osim.Model(str(model_path))
    forces = model.updForceSet()
    for i in reversed(range(forces.getSize())):
        if forces.get(i).getConcreteClassName() != 'SmoothSphereHalfSpaceForce':
            forces.remove(i)
    for collection, excluded in ((model.updJointSet(), {'patellofemoral_l', 'patellofemoral_r'}),
                                 (model.updBodySet(), {'patella_l', 'patella_r'})):
        for i in reversed(range(collection.getSize())):
            if collection.get(i).getName() in excluded:
                collection.remove(i)
    constraints = model.updConstraintSet()
    for i in reversed(range(constraints.getSize())):
        constraints.remove(i)
    model.finalizeConnections()
    return model


def set_state(model, state, io, z):
    for name in io['coordi']:
        coordinate = model.getCoordinateSet().get(name)
        coordinate.setValue(state, float(z[io['input']['Qs'][name]]), False)
        coordinate.setSpeedValue(state, float(z[io['input']['Qdots'][name]]))
    model.realizeVelocity(state)


def reference_inverse_dynamics(model_path, io, z, options=None):
    """Solve with the ordinary OpenSim engine, including selected body loads."""
    z = np.asarray(z, dtype=float).reshape(-1)
    model = reference_model(model_path)
    state = model.initSystem()
    set_state(model, state, io, z)
    options = options or {}
    pending = []
    for field, key in (('Forces', 'input_3d_body_forces'), ('Moments', 'input_3d_body_moments')):
        for selection in options.get(key, []):
            xyz = osim.Vec3(*z[io['input'][field][selection['name']]])
            ref = selection['reference_frame']
            if ref.lower() != 'ground':
                xyz = model.getBodySet().get(ref).expressVectorInGround(state, xyz)
            force = osim.PrescribedForce()
            force.setName('verify_' + selection['name'])
            force.setBodyName('/bodyset/' + selection['body'])
            force.setForceIsInGlobalFrame(True)
            force.setPointIsInGlobalFrame(False)
            if field == 'Forces':
                force.setForceFunctions(*(osim.Constant(x) for x in vector(xyz)))
                force.setPointFunctions(*(osim.Constant(x) for x in selection['point_in_body']))
            else:
                force.setTorqueFunctions(*(osim.Constant(x) for x in vector(xyz)))
            pending.append(force)
    for force in pending:
        model.addForce(force)
    if options.get('input_3d_body_forces') or options.get('input_3d_body_moments'):
        state = model.initSystem()
        set_state(model, state, io, z)
    # Determine each speed's mobility index by setting an independent speed.
    mappings = {}
    for name in io['coordi']:
        probe = osim.State(state)
        probe.updU().setToZero()
        model.getCoordinateSet().get(name).setSpeedValue(probe, 1.)
        mappings[name] = vector(probe.getU()).index(1.)
    udot = osim.Vector(state.getNU(), 0.)
    for name, i in mappings.items():
        udot.set(i, float(z[io['input']['Qdotdots'][name]]))
    result = osim.InverseDynamicsSolver(model).solve(state, udot)
    return np.array([result.get(mappings[name]) for name in io['coordi']])


def verify_inverse_dynamics(model_path, io, artifacts, verbose=False):
    """Use the reference static pose and fail if any fresh function disagrees.

    Temporary motions/setup/results are isolated. The IDTool input is the
    ordinary model with the same documented component exclusions as the port.
    """
    n = len(io['coordi'])
    z = np.zeros(io['input']['nInputs'])
    z[0:2*n:2] = .05
    if 'pelvis_ty' in io['input']['Qs']:
        z[io['input']['Qs']['pelvis_ty']] = -.05
    with build.workspace() as temporary:
        model = reference_model(model_path)
        model_file = temporary / 'reference.osim'
        model.printToXML(str(model_file))
        motion = temporary / 'verify.mot'
        names = list(io['coordi'])
        q = z[0:2*n:2]
        with motion.open('w', encoding='utf-8') as handle:
            handle.write('version=1\nnRows=10\nnColumns=' + str(n+1) + '\ninDegrees=no\nendheader\n')
            handle.write('\t'.join(['time', *names]) + '\n')
            for time in np.arange(1, 11)*.01:
                handle.write('\t'.join(str(x) for x in [time, *q]) + '\n')
        tool = osim.InverseDynamicsTool()
        tool.setModelFileName(str(model_file))
        tool.setCoordinatesFileName(str(motion))
        tool.setResultsDir(str(temporary))
        tool.setStartTime(.01)
        tool.setEndTime(.1)
        tool.setOutputGenForceFileName('reference.sto')
        setup = temporary / 'SetupID.xml'
        tool.printToXML(str(setup))
        build.run(['opensim-cmd', 'run-tool', setup], 'Inverse dynamics verification', temporary, verbose)
        result_path = temporary / 'reference.sto'
        build.require_file(result_path, 'Inverse dynamics verification')
        table = osim.TimeSeriesTable(str(result_path))
        labels = table.getColumnLabels()
        expected = []
        for name in names:
            # OpenSim labels Coupled coordinates as forces, although MATLAB's
            # legacy IO.jointi.rotations groups them with rotational coordinates.
            matches = [name + suffix for suffix in ('_force', '_moment') if name + suffix in labels]
            if len(matches) != 1:
                raise RuntimeError(f'Missing or ambiguous IDTool output for {name}: {labels}')
            expected.append(table.getDependentColumn(matches[0]).to_numpy()[0])
        expected = np.asarray(expected)
        errors = {}
        for kind in ('casadi', 'dll'):
            if kind not in artifacts:
                continue
            function = ca.Function.load(artifacts[kind]) if kind == 'casadi' else ca.external('F', artifacts[kind])
            actual = np.asarray(function(z)).reshape(-1)[:n]
            error = float(np.max(np.abs(expected-actual)))
            if not np.isfinite(error) or error >= 1e-6:
                raise RuntimeError(f'Inverse dynamics verification failed for {kind}: max error {error:g} (limit 1e-6)')
            errors[kind] = error
    return {'passed': True, 'max_absolute_errors': errors,
            'reference': 'Ordinary OpenSim IDTool with documented component exclusions'}

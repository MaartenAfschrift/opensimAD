"""Public Python equivalent of generateExternalFunction.m."""
import os
import re
import shutil
import sys
from pathlib import Path

from . import build
from .generator import write_cpp


def generate_external_function(
    model_path, output_dir, *, joints_order=None, coordinates_order=None,
    input_3d_body_forces=None, input_3d_body_moments=None,
    export_3d_positions=None, export_3d_orientations=None,
    export_3d_velocities=None, export_3d_velocities_proj_ground=None,
    export_grfs=False, export_grms=False, export_separate_grfs=False,
    export_contact_powers=False, output_filename='F', compiler=None,
    verbose=False, verify_id=True, second_order_derivatives=False, no_dll=False,
):
    """Generate rigid-body inverse dynamics and selected kinematics.

    Optional selections are lists of dictionaries matching MATLAB structs.
    Forces: body, point_in_body, name, reference_frame. Moments/orientations:
    body, name, reference_frame. Positions/velocities: body, point_in_body, name.
    F takes one column vector: interleaved q/speed, accelerations, xyz forces,
    xyz moments. Returned io indices and JSON are zero based; _IO.mat is one based.
    Verification mismatches raise an exception. Requires Windows CMake/MSVC and
    the repository's recorder SDK; never invokes MATLAB or GenF.exe.
    """
    if os.name != 'nt':
        raise RuntimeError('This workflow supports Windows with the bundled OpenSimAD SDK')
    model_path, output_dir = Path(model_path).resolve(), Path(output_dir).resolve()
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    if (not isinstance(output_filename, str) or not output_filename
            or re.search(r'[<>:"/\\|?*\x00-\x1f]', output_filename)
            or output_filename.endswith(('.', ' '))):
        raise ValueError('output_filename must be a valid Windows file basename')
    options = {key: value or [] for key, value in locals().copy().items()
               if key.startswith('export_3d_') or key.startswith('input_3d_') or key in ('joints_order', 'coordinates_order')}
    options.update(export_grfs=export_grfs, export_grms=export_grms,
                   export_separate_grfs=export_separate_grfs, export_contact_powers=export_contact_powers)
    io = write_cpp(model_path, output_dir, output_filename, options)
    artifacts = {key: str(output_dir / (output_filename + suffix)) for key, suffix in
                 (('cpp', '.cpp'), ('casadi', '.casadi'), ('mat', '_IO.mat'), ('json', '_IO.json'))}
    with build.workspace() as target:
        build.build_graph(Path(artifacts['cpp']), target, compiler, verbose)
        command = [sys.executable, str(Path(__file__).with_name('codegen.py')), str(target), str(io['input']['nInputs'])]
        if second_order_derivatives:
            command.append('--second-order')
        build.run(command, 'CasADi code generation', target, verbose)
        for filename in ('foo_jac.c', 'F_foo.casadi'):
            build.require_file(target / filename, 'CasADi code generation')
        shutil.copy2(target / 'F_foo.casadi', artifacts['casadi'])
        if not no_dll:
            build.build_library(target, output_dir, output_filename, compiler, verbose)
            artifacts.update(dll=str(output_dir / (output_filename + '.dll')),
                             lib=str(output_dir / (output_filename + '.lib')))
    verification = None
    if verify_id:
        from .verification import verify_inverse_dynamics
        verification = verify_inverse_dynamics(model_path, io, artifacts, verbose=verbose)
    return {'artifacts': artifacts, 'io': io, 'verification': verification}

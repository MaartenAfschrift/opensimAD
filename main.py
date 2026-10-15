"""Generate a differentiable OpenSim function, as in main_opensimAD.m.

Inputs: interleaved joint positions/speeds, accelerations, optional body loads.
Outputs: joint forces/torques and optional kinematics/contact quantities.
"""
from pathlib import Path

from opensimad import generate_external_function


# Locate this repository, independently of the working directory.
path_main = Path(__file__).resolve().parent

# %% User inputs

# Path to the OpenSim model.
path_open_sim_model = path_main / "examples" / "Hamner_modified.osim"

# Output folder and filename.
output_dir = path_main / "examples"
output_filename = "F_python"

# Use CMake's default compiler, or specify e.g. "Visual Studio 18 2026".
compiler = None

# Print build information and verify the generated function against OpenSim.
verbose = True
verify_id = True

# Leave both empty to retrieve the joint/coordinate order from the model.
joints_order = []
coordinates_order = []

# Gravity and contact forces are included by default. Add body forces/moments
# to represent a pusher, an exoskeleton or a custom contact model.
# Leave these lists empty to omit the additional load inputs.
input_3d_body_forces = [
    dict(
        body="torso",
        point_in_body=[-0.1, 0.3, 0],
        name="back_push",
        reference_frame="ground",
    ),
]

input_3d_body_moments = [
    dict(
        body="tibia_l",
        name="exo_shank_l",
        reference_frame="tibia_l",
    ),
    dict(
        body="calcn_l",
        name="exo_foot_l",
        reference_frame="tibia_l",
    ),
]

# Joint forces/torques are always returned. The following selections add
# outputs of interest. Leave a list empty to omit that output group.

# Export point positions in the ground frame.
export_3d_positions = [
    dict(
        body="tibia_l",
        point_in_body=[0, -0.012, 0],
        name="left_shin",
    ),
    dict(
        body="tibia_r",
        point_in_body=[0, -0.012, 0],
        name="right_shin",
    ),
]

# Export relative orientations as scalar-first quaternions.
export_3d_orientations = [
    dict(
        body="calcn_r",
        reference_frame="tibia_r",
        name="ankle_quat",
    ),
]

# Export point velocities in the ground frame.
export_3d_velocities = [
    dict(
        body="tibia_l",
        point_in_body=[0, -0.012, 0],
        name="left_shin",
    ),
]

# Export station velocities evaluated at locations projected onto ground y=0.
export_3d_velocities_proj_ground = [
    dict(
        body="tibia_l",
        point_in_body=[0, -0.012, 0],
        name="left_shin",
    ),
]

# Export total ground reaction forces: right, then left.
export_grfs = True

# Export the ground reaction force of each contact sphere.
export_separate_grfs = True

# Export total ground reaction moments: right, then left.
export_grms = True

# Export vertical deformation power for each contact sphere.
export_contact_powers = True

# The Jacobian is always generated. Enable additional derivative functions
# for second-order derivatives; this increases compilation time.
second_order_derivatives = False

# Set True to generate only the serialized function, without a DLL/LIB.
no_dll = False

# %% Generate the external function

result = generate_external_function(
    path_open_sim_model,
    output_dir,
    joints_order=joints_order,
    coordinates_order=coordinates_order,
    input_3d_body_forces=input_3d_body_forces,
    input_3d_body_moments=input_3d_body_moments,
    export_3d_positions=export_3d_positions,
    export_3d_orientations=export_3d_orientations,
    export_3d_velocities=export_3d_velocities,
    export_3d_velocities_proj_ground=export_3d_velocities_proj_ground,
    export_grfs=export_grfs,
    export_grms=export_grms,
    export_separate_grfs=export_separate_grfs,
    export_contact_powers=export_contact_powers,
    output_filename=output_filename,
    compiler=compiler,
    verbose=verbose,
    verify_id=verify_id,
    second_order_derivatives=second_order_derivatives,
    no_dll=no_dll,
)

print(result)

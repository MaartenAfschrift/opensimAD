# Python/MATLAB parity

The Python implementation ports `dev/bug_long_path` at
`2b0fc7a6fd7eeb15f0876215780f7ff51e3318d8`. The rigid-body calculations still
use the bundled OpenSimAD/SimbodyAD recorder libraries; this is not a dynamics
rewrite. The Python path does not invoke MATLAB or the bundled `GenF.exe`.

## Interface

The option names are the snake-case equivalents of `generateExternalFunction.m`.
Sequences contain dictionaries with the same keys as MATLAB structs. `None` or
an empty sequence disables a selection. If either joint/coordinate ordering is
omitted, both are derived from the model, matching MATLAB. Explicit orderings
must be mutually consistent; malformed permutations now fail before compilation.

Inputs: interleaved positions/speeds, coordinate accelerations, XYZ body forces,
then XYZ body moments. Coordinates and loads retain their selected order.

Outputs: generalized forces; point positions; relative quaternions; point
velocities; ground-projected station velocities; right/left total GRFs; separate
GRFs; right/left GRMs; contact deformation powers. Quaternions are scalar first.
Projected velocities are **not** velocities with their Y component removed:
they are velocities of body stations whose current locations are projected to
ground Y=0. Contact power remains the product of sphere vertical velocity and
vertical force. GRMs preserve the reference contact-point correction and
ground-origin convention. These definitions assume the reference's Y-up ground
plane, even when gravity has nonzero X/Z components.

`result['io']` and `<name>_IO.json` contain zero-based indices. JSON wraps the map
as `{"index_base": 0, "IO": ...}`. `<name>_IO.mat` contains the original `IO`
struct with one-based indices. Counts remain counts in both representations.
No coordinates or IO keys are renamed. The CasADi function is always named `F`.

## Deliberate corrections and retained limitations

- The MATLAB multiplier/univariate-polynomial branch used the wrong coefficient
  variable for seven coefficients. A small CustomJoint fixture reproduced
  `Unrecognized function or variable 'dofSel_f_coeffs'`. Both generators now use
  the wrapped polynomial's coefficients.
- Verification uses private temporary model/motion/setup/results files and
  fails on command errors, missing artifacts, unloadable functions and numerical
  disagreement. Static disagreement no longer produces only a warning.
- Coupled coordinates use an IDTool `_force` label. Both verifiers resolve the
  actual unique `_force`/`_moment` column; the legacy IO rotational/translational
  grouping is retained for compatibility.
- Serialized-only verification ignores stale DLLs from previous conversions.
- Python reads all multivariate dependencies directly from the transform axis.
  It preserves the SDK's coefficient ordering and function definitions.
- Locked PinJoints at zero default angle can become weld joints. Nonzero locked
  angles and locked Custom/Planar coordinates are rejected rather than silently
  changing pose or creating inconsistent coordinate counts.
- As in MATLAB, only diagonal inertia components are reconstructed. The supported
  custom-function combinations and univariate coefficient counts (2,3,4,5,7)
  are unchanged. Coordinate splines and additional joint types are unsupported.
- Constraints are ignored and reported. Muscle forces are excluded; other force
  types besides SmoothSphereHalfSpaceForce are reported as excluded. Independent
  verification uses the same documented exclusions, not the unchanged original
  model when patella/constraint exclusions apply.

## Builds and concurrency

Windows builds use the bundled SDK, CMake, Visual Studio and RelWithDebInfo.
Public filenames are independent of short internal targets. All build stages
retain diagnostics on failure, check artifacts and clean owned temporary files.
The caller's working directory is never changed by the Python workflow.

`OPENSIMAD_BUILD_ROOT` overrides the default temporary root. It must exist and
be short enough to reserve 120 characters for Visual Studio intermediate paths.
Model, SDK and delivered artifact paths must themselves remain toolchain-accessible.

Python exclusively creates the SDK `bin/lockFile.txt` and waits up to 300 seconds
for another recorder. Only its acquired lock and graph are removed. MATLAB's
existing check/create locking is not atomic, so simultaneous mixed-language
recording is not guaranteed race-free. Run mixed MATLAB/Python conversions
sequentially. Graph processing runs in a private Python subprocess to prevent
stale module reuse.

Serialization is version-sensitive. Use compatible CasADi readers; use the same
CasADi version when validating serialization across languages. The bundled
MATLAB GenF executable uses 3.6.2, while this checkout's Python environment uses
3.7.2. Both retain symbolic `F`, Jacobian and optional second-derivative code.

## Reproducing validation

Run from the repository using the Python interpreter containing OpenSim:

```powershell
python -m unittest discover -s tests -p test_python.py -v
$env:OPENSIMAD_INTEGRATION = '1'
python -m unittest discover -s tests -p test_python.py -v
python tests/validate_parity.py --casadi-matlab C:/path/to/casadi --output C:/path/to/results
```

The parity runner generates fresh MATLAB references for Hamner_modified and
LaiArnold_modified, compares metadata, eight deterministic states, complete
Jacobians, DLL/serialized results, dynamic ordinary-OpenSim inverse dynamics
and dynamics-only outputs. Integration tests cover short fixtures, body loads,
full gravity, station/orientation kinematics, multivariate functions, locked
pins, finite-difference Jacobians and Hessians, long paths, retries and stale
DLLs. Tests may create temporary build artifacts; large outputs are not committed.

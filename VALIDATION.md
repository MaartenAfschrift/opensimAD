# Python port validation — 2026-10-14

Reference: MATLAB `dev/bug_long_path`, commit
`2b0fc7a6fd7eeb15f0876215780f7ff51e3318d8`, with the documented narrow fixes.

Environment: Windows, Python 3.10.20, OpenSim Python 4.5.2, OpenSim CLI 4.5,
CasADi Python/MATLAB 3.7.2, bundled GenF 3.6.2, MATLAB R2025b, Visual Studio
2026 and the repository's bundled recorder SDK.

## Fresh full-model references

Each model was generated independently by MATLAB and Python. All selected body
loads, station outputs, relative orientation, ground-projected station velocity,
contact totals, individual contact forces and deformation powers were enabled.
Eight deterministic nonzero input states were compared per model.

| Check | Hamner_modified | LaiArnold_modified |
|---|---:|---:|
| MATLAB/Python IO metadata | Exact | Exact |
| Maximum output difference | 0 | 0 |
| Maximum Jacobian difference | 0 | 0 |
| DLL/serialized outputs and Jacobians | Passed | Passed |
| Maximum dynamic ordinary-OpenSim ID error | 3.10e-11 | 2.28e-11 |
| Static IDTool error, DLL and serialized | 4.91e-9 | 9.94e-9 |
| Dynamics-only serialized generation/check | Passed | Passed |

Parity tolerance: absolute 1e-9, relative 1e-10. Static IDTool threshold: 1e-6.
The reference models apply documented force/patella/constraint exclusions.
The ignored generated numerical summary is retained in
`examples/python_parity_summary.json`.

## Regression coverage

Python tests cover metadata/index counts, supported transform functions,
multivariate dependency names, invalid input references/order, lock ownership,
command failures and cleanup. Compiled fixtures cover PinJoint, PlanarJoint,
scaled seven-coefficient polynomials, direct/scaled multivariate transforms,
zero-angle locked PinJoint conversion, full gravity, nonzero body loads,
ordinary-OpenSim station/orientation kinematics, finite-difference Jacobians and
symbolic/external Hessians. Path tests cover long public names, spaces, short
build roots, invalid generators, retries, public DLL import-library names,
serialized-only conversion with a stale invalid DLL, and long-root rejection.

The MATLAB `testBuildPaths(false,'Visual Studio 18 2026')` regression suite
passed. Its intentional configuration/compilation/execution/code-generation
errors were expected test cases. The seven-coefficient MATLAB fixture reproduced
the original undefined-variable failure and generated correctly after repair.
The rewritten MATLAB static verifier also passed the fresh Hamner reference.

The Python example artifacts are retained under `examples/F_python*`. Large
temporary parity/build products are not tracked. See `PYTHON_PARITY.md` for
commands that reproduce the checks.

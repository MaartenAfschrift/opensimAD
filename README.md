# OpenSimAD
Windows libraries for OpenSimAD - OpenSim with support for Algorithmic Differentiation.

## How to generate an external function for use with CasADi?
OpenSimAD is used to formulate trajectory optimization problems with OpenSim musculoskeletal models. To leverage the benefits of algorithmic differentiation, we use [CasADi external functions](https://web.casadi.org/docs/#casadi-s-external-function). In our case, the external functions typically take as inputs the multi-body model states (joint positions and speeds) and controls (joint accelerations) and return the joint torques after solving inverse dynamics. The external functions can then be called when formulating trajectory optimization problems (e.g. https://github.com/KULeuvenNeuromechanics/PredSim, https://github.com/antoinefalisse/3dpredictsim and https://github.com/antoinefalisse/predictsim_mtp).


## Getting started
Here we provide code and examples to generate external functions automatically given an OpenSim musculoskeletal model (.osim file).
This workflow is not limited to full body models. [Any OpenSim model](https://user-images.githubusercontent.com/71920801/143950905-9ef6263e-c763-409a-bf7e-905efd8d28b8.png) within the given [limitations](#Limitations) can be used to generate an external function.

### Install requirements (Windows)
  - Third-party software:
    - CMake (make sure cmake.exe is in your path)
    - Visual studio (tested with Visual Studio 2015, 2017, 2019, 2022 Community editions)
    - MATLAB
    - [CasADi](https://web.casadi.org/get/)

### Example
  - Make sure CasADi is in your MATLAB path. Use `addpath(genpath(casadi_path))` with casadi_path the path to the top folder of your CasADi download.
  - Run `main_opensimAD.m`
  - You should get as output a few files in the example folder. Among them: `F_test.cpp` and `F_test.dll`. The .cpp file contains the source code of the external function, whereas the .dll file is the [dynamically linked library](https://web.casadi.org/docs/#casadi-s-external-function) that can be called when formulating your trajectory optimization problem. `F_test_IO.mat` contains a struct `IO` with the indices relating to the different inputs and outputs of `F`.

### Limitations
  - Not all OpenSim models are supported:
    - Your model **should not have locked joints**. Please replace them with weld joints. This can be easily done with [OpenSim Creator](https://www.opensimcreator.com/) by using *Rezero Joint* and *Change Joint Type*.
    - **Constraints will be ignored** (eg, coupling constraints).
    - **SimmSplines are not supported for coordinates**, as their implementation in OpenSim is not really compatible with algorithmic differentiation. See how we replaced the splines of the [LaiArnold_modifed model](https://simtk.org/projects/model-high-flex) with polynomials. SimmSplines in PathPoints etc. are fine, since they do not affect the inverse dynamics.
  - OpenSimAD does not support all features of OpenSim. **Make sure you verify what you are doing**. We have only used OpenSimAD for specific applications.

### Troubleshooting
- The code generates and runs .exe files, which can cause troubles with security policies. On KU Leuven GBW computers, run opensimAD from the C:/GBW_MyPrograms folder to prevent issues with group policy.
- When getting an error message containing "opensim" or "simbody", check your [OpenSim installation](https://simtk-confluence.stanford.edu:8443/display/OpenSim/Scripting+with+Matlab) and path settings are correct.
- Compilation uses unique temporary directories and short internal target names to avoid Visual Studio's 260-character intermediate-path limit. Public filenames, CasADi function names, and input/output metadata remain unchanged. Temporary builds are removed on success or failure, and MATLAB's working directory is restored.
- The build root defaults to MATLAB's `tempdir`. To use a shorter directory or a location where security policies permit executables, create that directory and set `setenv('OPENSIMAD_BUILD_ROOT', 'C:\your\short\buildroot')` before conversion. The directory must already exist. An excessively long build root is rejected before compilation. Model, SDK, and final output paths must still be accessible to the toolchain.
- Failed build commands report their stage, exit code, command, and captured output, even with verbose mode disabled. Resolve the first reported failure before retrying.

## opensimAD workflow


matlab:

1. Use api of regular opensim to read information from model file (\*.osim)
2. Generate code (F_\*.cpp) that constructs and analyzes this model, but the generated code uses the opensimAD api.
	Note: most arguments in the matlab functions are to configure the analysis that needs to be done.

terminal (via matlab `system()`):

3. Compile the generated code (F_\*.cpp) and link with opensimAD binaries to create (F_\*.exe). This uses cmake and visual studio compiler.
4. Run F_\*.exe to generate a function that contains the expression graph (sequence of elementary operations) of the desired analysis (foo.py).

python (GenF.py) or terminal (GenF.exe):

5. Evaluate the expression graph (foo.py) with CasADi symbolic variables to get a symbolic expression.
6. Create symbolic expressions of the partial derivatives of the input/output of the analysis.
7. (newer versions) Serialise the symbolic expressions and save them to a file (F_foo.casadi, F_\*.casadi).
8. Generate code (foo_jac.c) for the symbolic expressions of the analysis and the derivatives.

terminal (via matlab `system()`):

9. Compile foo_jac.c into a shared library (F_\*.dll). Functions in this library can be loaded into matlab via CasADi's `external()` function constructor.

matlab:

10. Use api of regular opensim to perform a dummy analysis for the model to get reference results.
11. Load the analysis function from F_\*.dll (or F_\*.casadi) and evaluate it with the same dummy inputs to verify the created file.
12. Remove all the temporary folders and files.


Notes

- The workflow is hard-coded to use cmake and visual studio compiler. Modifying this to use another compiler (maybe without cmake) is totally fine.
- Steps 5-8 use a compiled python function, because this is more convenient than asking people to set up the python api in matlab. When running or compiling GenF.py, the python environment should have CasADi binaries (with python interface!!).
- Step 7 can be used to skip step 8-9. The CasADi version in python should not be newer than the one in matlab. Loading serialised files is backward compatible, but writing isn't.The included `GenF.exe` uses CasADi 3.6.2.
- Step 10-11 can be skipped.
- PredSim will load the function the same way as in step 11.

## Citation
Please cite this paper in your publications if OpenSimAD helps your research:
  - Falisse A, Serrancolí G, et al. (2019) Algorithmic differentiation improves the computational efficiency of OpenSim-based trajectory optimization of human movement. PLoS ONE 14(10): e0217730. https://doi.org/10.1371/journal.pone.0217730

Please cite this paper in your publications if you used OpenSimAD for simulations of human walking:
  - Falisse A, et al. (2019) Rapid predictive simulations with complex musculoskeletal models suggest that diverse healthy and pathological human gaits can emerge from similar control strategies. J. R. Soc. Interface.162019040220190402. http://doi.org/10.1098/rsif.2019.0402

## Compiling GenF
The included custom OpenSim libraries generate the expression graph as `foo.py`, which cannot be used with CasADi from MATLAB. To avoid the need to set up python, we have compiled the function that uses foo.py (`GenF.py`) into an executable that can be called from MATLAB (`./utilities/GenF/GenF.exe`).

To compile this yourself:
  - Open an Anaconda prompt
  - Create environment: `conda create -n opensimAD pip spyder python=3.8`
  - Activate environment: `conda activate opensimAD`
  - Navigate to the folder where you want to download the code: eg. `cd Documents`
  - Download code: `git clone https://github.com/KULeuvenNeuromechanics/opensimAD.git`
  - Navigate to the folder: `cd opensimAD`
  - Install required packages: `python -m pip install casadi`
  - Install pyinstaller:  `conda install -c conda-forge pyinstaller`
  - Create an executable from GenF: `pyinstaller GenF.py  --distpath C:/.../opensimAD/utilities`

When running opensimAD, you can get an error from genF.exe that it cannot find casadi libraries. You then have to move the files from `./utilities/GenF/casadi/` to `./utilities/GenF/`.

## Source code
The libraries in `./opensimAD-install` were compiled from [here](https://github.com/antoinefalisse/opensim-core/tree/AD-recorder-work-py-install).

## Python workflow (Windows)

The Python implementation supports the MATLAB options, including body loads,
relative orientations, station velocities, per-contact forces/powers, full
3D gravity, multivariate joint polynomials and optional second derivatives.
It uses the same bundled recorder SDK and CMake projects. MATLAB is not required
for Python conversion.

Use Python 3.10 or later with OpenSim's Python bindings, CasADi, NumPy and SciPy.
Install OpenSim through its supported conda distribution; it is intentionally
not declared as a PyPI dependency. This checkout was validated with Python
3.10.20, OpenSim 4.5.2, CasADi 3.7.2 and Visual Studio 2026. CMake and
`opensim-cmd` must be on PATH when inverse-dynamics verification is enabled.

Run `python main.py` from the repository, or install the checkout for use from
other directories: `python -m pip install --no-deps -e .`. Use an **editable**
installation: the SDK and CMake resources remain in this repository and are
not bundled into a Python wheel.

```python
from opensimad import generate_external_function
import casadi as ca

result = generate_external_function(
    r'C:\models\model.osim', r'C:\results',
    output_filename='F_model', no_dll=True,
    export_3d_positions=[dict(body='tibia_l', point_in_body=[0, -.012, 0], name='left_shin')],
)
F = ca.Function.load(result['artifacts']['casadi'])
io = result['io']  # Zero-based indices; input.nInputs remains a count.
# With no_dll=False: F = ca.external('F', result['artifacts']['dll'])
```

A conversion returns `artifacts`, `io`, and `verification`. Both output forms
use a single packed column-vector input. The JSON/Python map uses zero-based
indices; `_IO.mat` preserves MATLAB's one-based IO structure. Set
`verify_id=False` only when intentionally skipping the static independent
OpenSim IDTool check; verification failures raise exceptions.

See [Python parity, limitations and tests](PYTHON_PARITY.md) for exact I/O
ordering, mathematical conventions, deliberate bug corrections, build behavior,
and commands for running the unit/integration and fresh MATLAB parity suites.

If an IDE's PATH omits CMake, the Python runner also checks its standard Windows
installation folders. For a custom installation, set `OPENSIMAD_CMAKE` to the
full path of `cmake.exe`. An explicit override takes precedence over PATH.

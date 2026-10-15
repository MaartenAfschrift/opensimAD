function testBuildPaths(fullModel, compiler)
% Run from the repository: addpath('tests'); testBuildPaths(false)
% Use true for OpenSim/CasADi numerical and inverse-dynamics checks.
if nargin == 0, fullModel = false; end
repo = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repo, 'utilities'));
originalDir = pwd;
originalRoot = getenv('OPENSIMAD_BUILD_ROOT');
testRoot = tempname;
mkdir(testRoot);
cleanup = onCleanup(@() finish(testRoot, originalDir, originalRoot));
buildRoot = fullfile(testRoot, 'build with spaces');
mkdir(buildRoot);
setenv('OPENSIMAD_BUILD_ROOT', buildRoot);
outputDir = fullfile(testRoot, 'output with spaces', repmat('deep_', 1, 12));
mkdir(outputDir);
name = 'F_Falisse_et_al_2022_slope07_torso125pct';
if nargin < 2, compiler = 'Visual Studio 16 2019'; end
writeFile(fullfile(outputDir, [name '.cpp']), ...
    sprintf('#include <fstream>\nint main(){std::ofstream("foo.py") << "graph"; return 0;}\n'));
foo = buildExpressionGraph(name, outputDir, compiler, false);
assert(strcmp(pwd, originalDir));
assert(isfile(fullfile(foo, 'foo.py')));
assert(~contains(foo, name));
assert(~isfile(fullfile(repo, 'OpenSimAD-install', 'bin', 'lockFile.txt')));
writeFile(fullfile(foo, 'foo_jac.c'), ...
    '__declspec(dllexport) double test_value(double x) { return x * 2; }');
buildExternalFunction(foo, name, outputDir, compiler, false);
assert(strcmp(pwd, originalDir));
assert(isfile(fullfile(outputDir, [name '.dll'])));
assert(isfile(fullfile(outputDir, [name '.lib'])));
% Import libraries must refer to the delivered public DLL, not the target.
fid = fopen(fullfile(outputDir, [name '.lib']), 'rb');
bytes = fread(fid, '*uint8')'; fclose(fid);
assert(contains(char(bytes), [name '.dll']));
rmdir(foo, 's');
assert(isempty(dir(fullfile(buildRoot, 'tp*'))));

for verbose = [false true]
    mustFail(@() buildExpressionGraph(name, outputDir, 'Invalid generator', verbose), ...
        'opensimAD:CommandFailed', 'Expression graph configuration');
    assert(strcmp(pwd, originalDir));
    writeFile(fullfile(outputDir, [name '.cpp']), 'this is invalid C++');
    mustFail(@() buildExpressionGraph(name, outputDir, compiler, verbose), ...
        'opensimAD:CommandFailed', 'Expression graph compilation');
    writeFile(fullfile(outputDir, [name '.cpp']), 'int main(){return 7;}');
    mustFail(@() buildExpressionGraph(name, outputDir, compiler, verbose), ...
        'opensimAD:CommandFailed', 'Expression graph execution');
    assert(~isfile(fullfile(repo, 'OpenSimAD-install', 'bin', 'lockFile.txt')));
    writeFile(fullfile(outputDir, [name '.cpp']), 'int main(){return 0;}');
    mustFail(@() buildExpressionGraph(name, outputDir, compiler, verbose), ...
        'opensimAD:MissingArtifact', 'Expression graph execution');
end
assert(strcmp(pwd, originalDir));
assert(isempty(dir(fullfile(buildRoot, 'tp*'))));
badGraph = fullfile(testRoot, 'bad graph'); mkdir(badGraph);
writeFile(fullfile(badGraph, 'foo_jac.c'), 'invalid C code');
mustFail(@() buildExternalFunction(badGraph, name, outputDir, compiler, false), ...
    'opensimAD:CommandFailed', 'External function compilation');
writeFile(fullfile(badGraph, 'foo.py'), 'invalid python syntax !');
mustFail(@() generateF(1, badGraph, false), ...
    'opensimAD:CommandFailed', 'CasADi code generation');
assert(strcmp(pwd, originalDir));
writeFile(fullfile(outputDir, [name '.cpp']), ...
    sprintf('#include <fstream>\nint main(){std::ofstream("foo.py") << "retry"; return 0;}'));
foo = buildExpressionGraph(name, outputDir, compiler, false);
assert(strcmp(fileread(fullfile(foo, 'foo.py')), 'retry'));
rmdir(foo, 's');
longRoot = fullfile(testRoot, repmat('long_', 1, 30)); mkdir(longRoot);
setenv('OPENSIMAD_BUILD_ROOT', longRoot);
mustFail(@() createBuildWorkspace(), 'opensimAD:BuildRootTooLong', 'OPENSIMAD_BUILD_ROOT');
setenv('OPENSIMAD_BUILD_ROOT', buildRoot);

if fullModel
    model = fullfile(repo, 'examples', 'Hamner_modified.osim');
    for currentName = {'F_short', name, 'F_serialized'}
        n = currentName{1};
        generateExternalFunction(model, outputDir, [], [], [], [], [], [], [], [], ...
            true, true, true, true, n, compiler, false, false, false, strcmp(n, 'F_serialized'));
        assert(strcmp(pwd, originalDir));
        for extension = {'.cpp', '_IO.mat', '.casadi'}
            assert(isfile(fullfile(outputDir, [n extension{1}])));
        end
    end
    assert(~isfile(fullfile(outputDir, 'F_serialized.dll')));
    refIO = load(fullfile(outputDir, 'F_short_IO.mat'));
    longIO = load(fullfile(outputDir, [name '_IO.mat']));
    assert(isequaln(refIO.IO, longIO.IO));
    short = casadi.external('F', fullfile(outputDir, 'F_short.dll'));
    long = casadi.external('F', fullfile(outputDir, [name '.dll']));
    serial = casadi.Function.load(fullfile(outputDir, 'F_serialized.casadi'));
    serialLong = casadi.Function.load(fullfile(outputDir, [name '.casadi']));
    js = short.jacobian(); jl = long.jacobian(); jserial = serial.jacobian();
    for k = 0:3
        x = 0.01 * sin((1:refIO.IO.input.nInputs)' + k);
        y = full(short(x));
        assert(max(abs(y-full(long(x)))) < 1e-10);
        assert(max(abs(y-full(serial(x)))) < 1e-10);
        assert(max(abs(y-full(serialLong(x)))) < 1e-10);
        a = full(js(x, y)); b = full(jl(x, y)); c = full(jserial(x, y));
        assert(max(abs(a(:)-b(:))) < 1e-10);
        assert(max(abs(a(:)-c(:))) < 1e-10);
    end
    % The existing verifier warns on disagreement; promote it to a failure.
    lastwarn('');
    VerifyInverseDynamics(model, outputDir, name, false);
    [message, ~] = lastwarn;
    assert(~contains(message, 'does not match') && ~contains(message, 'Unable to load'));
    clear short long serial serialLong js jl jserial;
end
fprintf('Build path regression tests passed (fullModel=%d).\n', fullModel);
end

function writeFile(path, contents)
fid = fopen(path, 'w'); assert(fid >= 0);
fprintf(fid, '%s', contents); fclose(fid);
end

function mustFail(action, identifier, text)
try
    action();
catch err
    assert(strcmp(err.identifier, identifier), '%s', err.message);
    assert(contains(err.message, text), '%s', err.message);
    return;
end
error('Expected failure: %s', identifier);
end

function finish(root, originalDir, originalRoot)
cd(originalDir);
setenv('OPENSIMAD_BUILD_ROOT', originalRoot);
if isfolder(root), rmdir(root, 's'); end
end

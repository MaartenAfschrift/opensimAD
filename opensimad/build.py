"""Windows builds with short targets and recorder coordination."""
import os
import re
import shutil
import subprocess
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parent.parent


def find_cmake(env=None):
    """Find CMake even when an IDE inherited an incomplete PATH."""
    environment = os.environ if env is None else env
    override = environment.get('OPENSIMAD_CMAKE')
    if override:
        executable = Path(override).expanduser()
        if not executable.is_file():
            raise FileNotFoundError(f'OPENSIMAD_CMAKE does not point to a file: {override}')
        return str(executable.resolve())
    executable = shutil.which('cmake', path=environment.get('PATH', ''))
    if executable:
        return executable
    if os.name == 'nt':
        for variable, default in (('ProgramW6432', r'C:\Program Files'),
                                  ('ProgramFiles', r'C:\Program Files'),
                                  ('ProgramFiles(x86)', r'C:\Program Files (x86)')):
            candidate = Path(environment.get(variable, default)) / 'CMake' / 'bin' / 'cmake.exe'
            if candidate.is_file():
                return str(candidate)
    raise FileNotFoundError(
        'CMake was not found on PATH or in the standard Windows installation folders. '
        'Install CMake or set OPENSIMAD_CMAKE to the full path of cmake.exe.'
    )

def run(command, stage, cwd, verbose=False, env=None):
    command = [str(x) for x in command]
    try:
        if command[0].lower() in ('cmake', 'cmake.exe'):
            command[0] = find_cmake(env)
        result = subprocess.run(command, cwd=cwd, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, errors='replace', check=False)
    except OSError as error:
        raise RuntimeError(f'{stage} failed to start: {subprocess.list2cmdline([str(x) for x in command])}\n{error}') from error
    if verbose:
        print(result.stdout, end='', flush=True)
    if result.returncode:
        raise RuntimeError(f'{stage} failed (exit code {result.returncode}).\nCommand: '
                           f'{subprocess.list2cmdline([str(x) for x in command])}\n{result.stdout}')
    return result.stdout


def require_file(path, stage):
    if not Path(path).is_file():
        raise RuntimeError(f'{stage}: missing artifact {path}')


@contextmanager
def workspace():
    root = Path(os.environ.get('OPENSIMAD_BUILD_ROOT', tempfile.gettempdir())).resolve()
    if not root.is_dir():
        raise ValueError(f'Build root does not exist: {root}')
    # mkdtemp adds eight characters to the prefix. Reserve 120 for MSBuild.
    if os.name == 'nt' and len(str(root / ('oa_' + 'x'*8))) + 120 >= 260:
        raise ValueError(f'Build root is too long: {root}. Set OPENSIMAD_BUILD_ROOT to a shorter existing directory.')
    path = Path(tempfile.mkdtemp(prefix='oa_', dir=root))
    try:
        yield path
    finally:
        shutil.rmtree(path)


def generator_arguments(compiler):
    if compiler is None:
        return ['-A', 'x64']
    if not compiler.startswith('Visual Studio '):
        raise ValueError('The bundled Windows SDK requires a Visual Studio generator')
    match = re.match(r'Visual Studio (\d+)', compiler)
    if not match:
        raise ValueError(f'Invalid Visual Studio generator: {compiler}')
    return ['-G', compiler] + (['-A', 'x64'] if int(match.group(1)) >= 16 else [])


@contextmanager
def recorder_lock(bin_dir, timeout=300):
    lock = Path(bin_dir) / 'lockFile.txt'
    deadline = time.monotonic() + timeout
    while True:
        try:
            descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            with os.fdopen(descriptor, 'w') as handle:
                handle.write(f'Python OpenSimAD pid={os.getpid()}')
            break
        except FileExistsError:
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Timed out waiting for {lock}')
            time.sleep(min(1, max(0, deadline-time.monotonic())))
    try:
        yield
    finally:
        (Path(bin_dir) / 'foo.py').unlink(missing_ok=True)
        lock.unlink(missing_ok=True)


def build_graph(source, target, compiler, verbose):
    sdk = REPOSITORY / 'opensimAD-install' / 'sdk'
    bin_dir = sdk.parent / 'bin'
    require_file(sdk / 'lib' / 'osimCommon.lib', 'OpenSimAD SDK')
    shutil.copy2(source, target / 'graph.cpp')
    shutil.copy2(REPOSITORY / 'buildExpressionGraph' / 'CMakeLists.txt', target)
    run(['cmake', '.', *generator_arguments(compiler), f'-DSDK_DIR:PATH={sdk}'],
        'Expression graph configuration', target, verbose)
    run(['cmake', '--build', '.', '--config', 'RelWithDebInfo'], 'Expression graph compilation', target, verbose)
    executable = target / 'RelWithDebInfo' / 'osimad_graph.exe'
    require_file(executable, 'Expression graph compilation')
    with recorder_lock(bin_dir):
        graph = bin_dir / 'foo.py'
        graph.unlink(missing_ok=True)
        run([executable], 'Expression graph execution', bin_dir, verbose)
        require_file(graph, 'Expression graph execution')
        shutil.copy2(graph, target / 'foo.py')


def build_library(graph_dir, output_dir, name, compiler, verbose):
    with workspace() as target:
        shutil.copy2(REPOSITORY / 'buildExternalFunction' / 'CMakeLists.txt', target)
        shutil.copy2(graph_dir / 'foo_jac.c', target)
        install = target / 'install'
        run(['cmake', '.', *generator_arguments(compiler), f'-DOUTPUT_NAME:STRING={name}', f'-DINSTALL_DIR:PATH={install}'],
            'External function configuration', target, verbose)
        run(['cmake', '--build', '.', '--config', 'RelWithDebInfo', '--target', 'install'],
            'External function compilation and installation', target, verbose)
        for folder, extension in (('bin', '.dll'), ('lib', '.lib')):
            artifact = install / folder / (name + extension)
            require_file(artifact, 'External function compilation')
            shutil.copy2(artifact, output_dir / artifact.name)

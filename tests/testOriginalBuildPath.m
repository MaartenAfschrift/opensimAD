function testOriginalBuildPath()
% Demonstrate the original Visual Studio 2019 failure without a model/SDK.
name = 'F_Falisse_et_al_2022_slope07_torso125pct';
oldDir = pwd;
root = tempname;
mkdir(root);
cleanup = onCleanup(@() finish(root, oldDir));
% Keep the project/source paths under 260 but push tracking files over it.
padding = repmat('p', 1, max(1, 121-length(root)-length(name)-2));
build = fullfile(root, padding, name);
mkdir(build);
writeFile(fullfile(build, [name '.cpp']), 'int main(){return 0;}');
writeFile(fullfile(build, 'CMakeLists.txt'), sprintf( ...
    'cmake_minimum_required(VERSION 3.10)\nproject(%s)\nadd_executable(%s %s.cpp)\n', name, name, name));
cd(build);
[status, output] = system('cmake . -G "Visual Studio 16 2019"');
assert(status == 0, '%s', output);
[status, output] = system('cmake --build . --config RelWithDebInfo');
assert(status ~= 0 && (contains(output, 'MSB3491') || contains(output, 'MSB4184')), '%s', output);
assert(contains(output, 'max path limit'), '%s', output);
fprintf('Original path layout reproduced the MSBuild OS max path limit failure.\n');
end

function writeFile(path, contents)
fid = fopen(path, 'w'); assert(fid >= 0);
fprintf(fid, '%s', contents); fclose(fid);
end

function finish(root, oldDir)
cd(oldDir);
if isfolder(root), rmdir(root, 's'); end
end

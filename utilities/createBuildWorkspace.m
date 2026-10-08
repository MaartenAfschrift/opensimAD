function pathBuild = createBuildWorkspace()
% One short, unique directory per conversion, independent of the model name.
root = getenv('OPENSIMAD_BUILD_ROOT');
if isempty(root)
    root = tempdir;
end
if ~isfolder(root)
    error('opensimAD:BuildRoot', 'Build root does not exist: %s', root);
end
oldDir = pwd;
restoreDir = onCleanup(@() cd(oldDir));
cd(root);
root = pwd; % Resolve relative roots before other stages change directory.
pathBuild = tempname(root);
% Reserve space for Visual Studio target, configuration and tracking paths.
if ispc && length(pathBuild) + 120 >= 260
    error('opensimAD:BuildRootTooLong', ...
        'Build root is too long: %s. Set OPENSIMAD_BUILD_ROOT to a shorter existing directory.', root);
end
mkdir(pathBuild);
end

function cleanupBuildWorkspace(pathBuild, originalDir)
% Leave the build directory before removing it, including on failed commands.
cd(originalDir);
if isfolder(pathBuild)
    rmdir(pathBuild, 's');
end
end

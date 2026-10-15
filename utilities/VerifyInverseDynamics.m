function [] = VerifyInverseDynamics(pathOpenSimModel, outputDir, outputFilename, verbose_mode, noDll)
% Compare fresh outputs with ordinary OpenSim IDTool; mismatches are errors.
% Reference: Falisse et al. (2019), doi:10.1371/journal.pone.0217730.
% Port of the original verifier by Lars D'Hondt (Apache-2.0).
if nargin < 5, noDll = false; end
import org.opensim.modeling.*;
import casadi.*;
oldDir = pwd;
work = createBuildWorkspace();
cleanup = onCleanup(@() cleanupBuildWorkspace(work, oldDir));
data = load(fullfile(outputDir, [outputFilename '_IO.mat']), 'IO');
IO = data.IO;
names = fieldnames(IO.coordi);
n = numel(names);
z = zeros(IO.input.nInputs, 1);
z(1:2:2*n) = 0.05;
if isfield(IO.input.Qs, 'pelvis_ty'), z(IO.input.Qs.pelvis_ty) = -0.05; end

% Use the same documented component exclusions as the generator.
model = Model(pathOpenSimModel);
forces = model.updForceSet();
for i = forces.getSize()-1:-1:0
    if ~strcmp(char(forces.get(i).getConcreteClassName()), 'SmoothSphereHalfSpaceForce')
        forces.remove(i);
    end
end
joints = model.updJointSet();
for i = joints.getSize()-1:-1:0
    if ismember(char(joints.get(i).getName()), {'patellofemoral_l','patellofemoral_r'})
        joints.remove(i);
    end
end
bodies = model.updBodySet();
for i = bodies.getSize()-1:-1:0
    if ismember(char(bodies.get(i).getName()), {'patella_l','patella_r'})
        bodies.remove(i);
    end
end
constraints = model.updConstraintSet();
for i = constraints.getSize()-1:-1:0, constraints.remove(i); end
model.finalizeConnections();
modelPath = fullfile(work, 'reference.osim');
model.print(modelPath);
q.labels = [{'time'}, names'];
q.data = [(0.01:0.01:0.1)', repmat(z(1:2:2*n)', 10, 1)];
q.inDeg = 'no';
motion = fullfile(work, 'verify.mot');
write_motionFile_v40(q, motion);
tool = InverseDynamicsTool();
tool.setModelFileName(modelPath);
tool.setCoordinatesFileName(motion);
tool.setResultsDir(work);
tool.setStartTime(0.01);
tool.setEndTime(0.1);
tool.setOutputGenForceFileName('reference.sto');
setup = fullfile(work, 'SetupID.xml');
tool.print(setup);
runBuildCommand(['opensim-cmd run-tool "' setup '"'], 'Inverse dynamics verification', verbose_mode);
resultPath = fullfile(work, 'reference.sto');
requireBuildFile(resultPath, 'Inverse dynamics verification');
reference = importdata(resultPath);
expected = zeros(n, 1);
for i = 1:n
    % Coupled coordinates have an IDTool _force suffix, despite the legacy
    % metadata grouping them with rotational coordinates.
    column = strcmp(reference.colheaders, [names{i} '_force']) | ...
        strcmp(reference.colheaders, [names{i} '_moment']);
    assert(sum(column) == 1, 'Missing or ambiguous IDTool coordinate output.');
    expected(i) = reference.data(1, column);
end
extensions = {'.casadi'};
if ~noDll, extensions{end+1} = '.dll'; end
for i = 1:numel(extensions)
    extension = extensions{i};
    artifact = fullfile(outputDir, [outputFilename extension]);
    requireBuildFile(artifact, 'Inverse dynamics verification');
    if strcmp(extension, '.casadi')
        F = casadi.Function.load(artifact);
    else
        F = casadi.external('F', artifact);
    end
    actual = full(F(z));
    difference = max(abs(actual(1:n) - expected));
    if ~isfinite(difference) || difference >= 1e-6
        error('opensimAD:VerificationFailed', ...
            'Inverse dynamics verification failed for %s (max error %.9g, limit 1e-6).', artifact, difference);
    end
    if verbose_mode, fprintf('Verified %s (max error %.9g).\n', artifact, difference); end
end
end

function R = sweep_ki(cfgs)
%SWEEP_KI  Runs the default scenario with register overrides and reports
%   steady offsets, overshoot and IAE. Results -> sweep_ki.txt
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(fileparts(here), 'simulink'));
if nargin < 1
    cfgs = {struct(), struct('KI',8), struct('KI',13), struct('KI',8,'ILIM',60), ...
            struct('KI',13,'ILIM',60), struct('KI',26,'ILIM',60)};
end
fid = fopen(fullfile(here, 'sweep_ki.txt'), 'w');
R = [];
for k = 1:numel(cfgs)
    scn = bb_scenario(); scn.reg = cfgs{k};
    bdclose('all'); evalc('build_bb_simscape(scn);');
    o = sim('bb_simscape');
    m = bb_metrics(o, scn);
    f = fieldnames(cfgs{k}); lbl = 'defaults';
    if ~isempty(f), lbl = strjoin(cellfun(@(n) sprintf('%s=%d', n, cfgs{k}.(n)), f, 'uni', 0), ' '); end
    line = sprintf('%-18s off330 %+5.1f  off250 %+5.1f  after-push %+5.1f  overshoot %4.1f/%4.1f mm  IAE %5.0f  tau-sat %.1f%%\n', ...
        lbl, m.off1, m.off2, m.off3, m.ov1, m.ov2, m.iae, 100*m.tausat);
    fprintf('%s', line); fprintf(fid, '%s', line);
    R = [R; m]; %#ok<AGROW>
end
fclose(fid);
end

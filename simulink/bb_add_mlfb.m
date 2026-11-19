function s = bb_add_mlfb(mdl, path, src, xy) %#ok<INUSL>
%BB_ADD_MLFB  Add a MATLAB Function block whose code is `src`.
%   Arguments named P_* become non-tunable parameters; known signal names
%   get explicit data types (bb_sig_types). Integer overflow wraps (as in HDL).
%   Shared by build_bb_model (HDL DUT) and simscape/build_bb_simscape.
add_block('simulink/User-Defined Functions/MATLAB Function', path, ...
    'Position', [xy(1) xy(2) xy(1)+150 xy(2)+180]);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', path);
ch.Script = src;
ch.SaturateOnIntegerOverflow = false;
ch.EmlDefaultFimath = 'Other:UserSpecified';
ch.InputFimath = 'hdlfimath';
d = ch.find('-isa', 'Stateflow.Data');
in = {}; out = {};
T = bb_sig_types();
for k = 1:numel(d)
    if startsWith(d(k).Name, 'P_')
        d(k).Scope = 'Parameter';
        d(k).Tunable = false;
    elseif isKey(T, d(k).Name)
        d(k).DataType = T(d(k).Name);   % explicit types: feedback loops cannot inherit
    end
end
for k = 1:numel(ch.Inputs),  in{end+1}  = ch.Inputs(k).Name;  end %#ok<AGROW>
for k = 1:numel(ch.Outputs), out{end+1} = ch.Outputs(k).Name; end %#ok<AGROW>
s = struct('path', path, 'in', {in}, 'out', {out});
end

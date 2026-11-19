function gen_hdl(withTb)
%GEN_HDL  Generate VHDL for BallBeamCore (Spartan-6 XC6SLX9-2TQG144, 24 MHz)
%   into ../hdl. With withTb = true, also generate an HDL Coder testbench
%   (stimulus/expected vectors from a 35 ms Simulink run) into ../hdl/tb.
if nargin < 1, withTb = true; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
mdl  = 'bb_core';
dut  = [mdl '/BallBeamCore'];
build_bb_model(0.035);
hdlDir = fullfile(root, 'hdl');
if exist(hdlDir, 'dir'), rmdir(hdlDir, 's'); end
hdlset_param(mdl, 'TargetDirectory', hdlDir);
makehdl(dut, 'TargetLanguage', 'VHDL', 'TargetDirectory', hdlDir, ...
    'HDLCodingStandard', 'Industry', 'GenerateHDLCode', 'on', ...
    'HDLGenerateWebview', 'off', 'Traceability', 'off', 'ResourceReport', 'on', ...
    'OptimizationReport', 'off', 'HDLCodingStandardCustomizations', ...
    hdlcoder.CodingStandard('Industry'));
if withTb
    makehdltb(dut, 'TargetLanguage', 'VHDL', 'TargetDirectory', hdlDir, ...
        'GenerateHDLTestBench', 'on', 'UseFileIOInTestBench', 'on', ...
        'SimulationTool', 'Custom', 'GenerateCoSimModel', 'None', ...
        'HDLCodingStandard', 'Industry');
end
save_system(mdl);
end

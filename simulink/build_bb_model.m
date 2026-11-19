function mdl = build_bb_model(simTime)
%BUILD_BB_MODEL  Builds bb_core.slx from the MATLAB Function sources in mlfb/.
%   Top level : harness (plant + HC-SR04 + UART host)  <->  BallBeamCore (DUT)
%   Inside the DUT every block is a MATLAB Function block; signals are wired
%   automatically by name (an output named X drives every input named X;
%   DUT inports/outports are created for unmatched names listed below).
%   Arguments named P_* are non-tunable parameters taken from bb_params().
if nargin < 1, simTime = 0.12; end
here = fileparts(mfilename('fullpath'));
mdl  = 'bb_core';
P    = bb_params();

if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']);
if exist(f, 'file'), delete(f); end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '1/24e6', 'StopTime', num2str(simTime, 17), ...
    'SaveOutput', 'on', 'SignalLogging', 'on', 'ReturnWorkspaceOutputs', 'on', ...
    'SingleTaskRateTransMsg', 'error', 'AlgebraicLoopMsg', 'error');

% ---- parameters -> model workspace ----
ws = get_param(mdl, 'ModelWorkspace');
prm = struct('P_CLK_PER_US', P.CLK_PER_US, 'P_UART_DIV', P.UART_DIV, ...
    'P_TRIG_US', P.TRIG_US, 'P_RISE_TMO_US', P.RISE_TMO_US, 'P_ECHO_MAX_US', P.ECHO_MAX_US, ...
    'P_MM_PER_US_Q16', P.MM_PER_US_Q16, 'P_MIN_MM', P.MIN_MM, 'P_MAX_MM', P.MAX_MM, ...
    'P_PERIOD_US', P.PWM_PERIOD_US, 'P_ABS_MIN_US', P.ABS_MIN_US, 'P_ABS_MAX_US', P.ABS_MAX_US, ...
    'P_INIT_US', P.DEF.CENTER, 'P_VERSION', P.VERSION, 'P_DT', 1/P.CLK_HZ, ...
    'P_CMD', bb_test_commands(), 'P_CMD_T0', 0.5e-3);
fn = fieldnames(P.DEF);
for k = 1:numel(fn), prm.(['P_DEF_' fn{k}]) = P.DEF.(fn{k}); end
pn = fieldnames(prm);
for k = 1:numel(pn), assignin(ws, pn{k}, prm.(pn{k})); end

% ---- DUT subsystem ----
dut = [mdl '/BallBeamCore'];
add_block('built-in/Subsystem', dut, 'Position', [400 100 600 400]);
blocks = {'io_sync','timebase','hcsr04_if','pos_filter','pid_ctrl','servo_pwm', ...
          'uart_rx','cmd_regs','telemetry','uart_tx','status_leds'};
dutIn  = {'echo_in','uart_rxd'};
dutOut = {'trig_out','servo_out','uart_txd','led0','led1','led2','led3'};
info = struct('path', {}, 'in', {}, 'out', {});
for k = 1:numel(blocks)
    src = fileread(fullfile(here, 'mlfb', [blocks{k} '.m']));
    info(k) = add_mlfb(mdl, [dut '/' blocks{k}], src, [200*mod(k-1,4)+50, 220*floor((k-1)/4)+50]); %#ok<AGROW>
end
for k = 1:numel(dutIn)
    b = [dut '/' dutIn{k}];
    add_block('simulink/Sources/In1', b, 'Position', [-100 60*k -70 60*k+14], ...
        'OutDataTypeStr', 'boolean');
    info(end+1) = struct('path', b, 'in', {{}}, 'out', {dutIn(k)}); %#ok<AGROW>
end
for k = 1:numel(dutOut)
    b = [dut '/' dutOut{k}];
    add_block('simulink/Sinks/Out1', b, 'Position', [900 60*k 930 60*k+14]);
    info(end+1) = struct('path', b, 'in', {dutOut(k)}, 'out', {{}}); %#ok<AGROW>
end
autowire(dut, info);

% ---- harness ----
hsrc = fileread(fullfile(here, 'harness_plant.m'));
h = add_mlfb(mdl, [mdl '/harness'], hsrc, [100 100]);
set_param(h.path, 'Position', [100 100 300 400]);
dutPH = get_param(dut, 'PortHandles');
hPH   = get_param(h.path, 'PortHandles');
% harness outputs echo_in, uart_rxd -> DUT inputs (order = dutIn)
% (through unit delays: the harness is a direct-feedthrough MATLAB Function)
for k = 1:2
    z = sprintf('%s/zh%d', mdl, k);
    add_block('simulink/Discrete/Unit Delay', z, 'InitialCondition', num2str(k == 2), ...
        'Position', [330 100+60*k 360 130+60*k]);
    zp = get_param(z, 'PortHandles');
    add_line(mdl, hPH.Outport(k), zp.Inport(1), 'autorouting', 'on');
    add_line(mdl, zp.Outport(1), dutPH.Inport(k), 'autorouting', 'on');
end
% DUT trig_out, servo_out -> harness
add_line(mdl, dutPH.Outport(1), hPH.Inport(1), 'autorouting', 'on');
add_line(mdl, dutPH.Outport(2), hPH.Inport(2), 'autorouting', 'on');
% log DUT outputs + harness observables
names = [dutOut, {'ball_mm','pulse_us'}];
srcs  = [num2cell(dutPH.Outport), {hPH.Outport(3), hPH.Outport(4)}];
for k = 1:numel(names)
    b = [mdl '/log_' names{k}];
    add_block('simulink/Sinks/To Workspace', b, 'VariableName', names{k}, ...
        'SaveFormat', 'Array', 'Decimation', '1', 'Position', [750 40*k 830 40*k+20]);
    if strcmp(names{k}, 'ball_mm') || strcmp(names{k}, 'pulse_us')
        set_param(b, 'Decimation', '2400');   % 10 kHz is plenty for plant signals
    end
    p = get_param(b, 'PortHandles');
    add_line(mdl, srcs{k}, p.Inport(1), 'autorouting', 'on');
end

hdl_settings(mdl, dut);
save_system(mdl, f);
fprintf('built %s (%d DUT blocks)\n', f, numel(blocks));
end

% -------------------------------------------------------------------------
function s = add_mlfb(mdl, path, src, xy)
s = bb_add_mlfb(mdl, path, src, xy);
end

function autowire(sys, info)
drivers = containers.Map();
for k = 1:numel(info)
    for j = 1:numel(info(k).out)
        nm = info(k).out{j};
        assert(~isKey(drivers, nm), 'signal %s driven twice', nm);
        drivers(nm) = [k j];
    end
end
for k = 1:numel(info)
    ph = get_param(info(k).path, 'PortHandles');
    for j = 1:numel(info(k).in)
        nm = info(k).in{j};
        assert(isKey(drivers, nm), 'no driver for input %s of %s', nm, info(k).path);
        kj = drivers(nm);
        sph = get_param(info(kj(1)).path, 'PortHandles');
        if strcmp(nm, 'tx_busy')
            % the only feedback loop (telemetry <-> uart_tx): break it with a register
            z = [sys '/z_' nm];
            add_block('simulink/Discrete/Unit Delay', z, 'InitialCondition', '0', ...
                'Position', [700 600 730 630]);
            zp = get_param(z, 'PortHandles');
            add_line(sys, sph.Outport(kj(2)), zp.Inport(1), 'autorouting', 'on');
            l = add_line(sys, zp.Outport(1), ph.Inport(j), 'autorouting', 'on');
        else
            l = add_line(sys, sph.Outport(kj(2)), ph.Inport(j), 'autorouting', 'on');
        end
        set_param(l, 'Name', nm);
    end
end
end

function hdl_settings(mdl, dut)
hdlset_param(mdl, 'HDLSubsystem', dut);
hdlset_param(mdl, 'TargetLanguage', 'VHDL');
hdlset_param(mdl, 'ResetType', 'Synchronous');            % Xilinx-preferred
hdlset_param(mdl, 'ResetAssertedLevel', 'Active-high');
hdlset_param(mdl, 'ClockInputPort', 'clk');
hdlset_param(mdl, 'ResetInputPort', 'reset');
hdlset_param(mdl, 'ClockEnableInputPort', 'clk_enable');
hdlset_param(mdl, 'MinimizeClockEnables', 'on');
hdlset_param(mdl, 'SynthesisTool', 'Xilinx ISE');
hdlset_param(mdl, 'SynthesisToolChipFamily', 'Spartan6');
hdlset_param(mdl, 'SynthesisToolDeviceName', 'xc6slx9');
hdlset_param(mdl, 'SynthesisToolPackageName', 'tqg144');
hdlset_param(mdl, 'SynthesisToolSpeedValue', '-2');
hdlset_param(mdl, 'TargetFrequency', 24);
hdlset_param(mdl, 'GenerateHDLTestBench', 'on');
end

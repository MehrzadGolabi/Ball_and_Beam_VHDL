function test_rolling()
%TEST_ROLLING  Validates the ball model used in bb_simscape on a fixed incline.
%   Ball = Prismatic joint along the track carrying the rolling-equivalent
%   mass m_eff = m + I/r_eff^2, plus a constant world-frame upward force
%   (m_eff - m)*g so that gravity acts only on the real mass:
%       m_eff * s'' = m*g*sin(alpha)   ->   a = g*sin(alpha)/(1 + I/(m*r_eff^2))
%   (Rack-and-Pinion was tried first but does not assemble with a moving pinion.)
M = bb_mech_params();
mdl = 'tst_roll';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl, 'StopTime', '0.5', 'Solver', 'ode23t', 'MaxStep', '1e-3');
alpha = 3;
s = mdl;
smh.add(s, 'world', 'W', [20 100]);
smh.add(s, 'mech', 'Mech', [20 20]);
smh.add(s, 'solver', 'Solv', [20 200]);
smh.rt(s, 'Tilt', [120 100], [0 0 0], 'Y', alpha);   % +x end goes down
smh.rt(s, 'Along', [220 100], [0 0 0], 'Y', 90);     % joint z along the track
smh.add(s, 'prismatic', 'Roll', [320 100], 'SensePosition', 'on');
smh.solid(s, 'sphere', 'Ball', [420 100], M.ball.r, ...
    struct('mc', M.ball.meff, 'I', M.ball.I*[1 1 1]), [1 0.5 0]);
add_comp_force(s, 'Ball', M, [420 200]);
c = @(a, ap, b, bp) smh.c(s, [s '/' a], ap, [s '/' b], bp);
c('W','R1','Mech','R1'); c('W','R1','Solv','R1'); c('W','R1','Tilt','L1');
c('Tilt','R1','Along','L1'); c('Along','R1','Roll','L1'); c('Roll','R1','Ball','R1');
smh.add(s, 'ps2sl', 'PS', [520 100]);
add_block('simulink/Sinks/To Workspace', [s '/x'], 'VariableName', 'x', ...
    'SaveFormat', 'Timeseries', 'Position', [620 100 680 130]);
c('Roll','R2','PS','L1');
p = get_param([s '/PS'], 'PortHandles'); q = get_param([s '/x'], 'PortHandles');
add_line(s, p.Outport(1), q.Inport(1));
out = sim(s);
t = out.x.Time; d = squeeze(out.x.Data);
a_sim = 2*d(end)/t(end)^2;
a_th = 9.81*sind(alpha)*M.ball.m/M.ball.meff;
fprintf('rolling test: a_sim = %.5f, a_theory = %.5f m/s^2 (%.3f %% error)\n', ...
    a_sim, a_th, 100*abs(a_sim/a_th - 1));
assert(abs(a_sim/a_th - 1) < 1e-3, 'rolling model mismatch');
close_system(s, 0);
end

function add_comp_force(s, ballName, M, pos)
% constant world-frame +z force (m_eff - m)*g on the ball
f = smh.add(s, 'extforce', 'Fcomp', pos, 'EnableForce', 'on', 'ForceResolutionFrame', 'World');
k = [s '/Fval'];
add_block('simulink/Sources/Constant', k, 'Value', ...
    mat2str([0 0 (M.ball.meff - M.ball.m)*9.80665], 10), 'Position', [pos(1)-200 pos(2) pos(1)-150 pos(2)+20]);
cv = smh.add(s, 'sl2ps', 'Fconv', [pos(1)-120 pos(2)]);
set_param(cv, 'Unit', 'N');
P = get_param(k, 'PortHandles'); Q = get_param(cv, 'PortHandles');
add_line(s, P.Outport(1), Q.Inport(1));
smh.c(s, cv, 'R1', f, 'L1');
smh.c(s, f, 'R1', [s '/' ballName], 'R1');
end

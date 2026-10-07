%% design_pid_leadlag.m -- inner-loop design, stability margins, lead-lag
%  V2V-Assisted Predictive ACC / BECE302L
%
%  Produces the root locus, Bode plots and margin numbers for the report,
%  and emits the #define block to paste into firmware/common/config.h.
%
%  Run sysid_fit.m first (or edit the plant block below by hand).
%
%  DESIGN METHOD -- lambda / IMC tuning
%  For a first-order plant G = K/(tau*s+1) under PI control, choosing
%      Kp = tau/(K*lambda),   Ki = Kp/tau
%  places the PI zero exactly on the plant pole. The pole cancels, and the
%  nominal closed loop collapses to a single first-order lag with time
%  constant lambda: no overshoot, no oscillation, one knob.
%
%  WHY lambda CANNOT BE ARBITRARILY SMALL
%  Pole cancellation is exact only in the model. The real plant has a
%  transport delay L that cannot be cancelled by anything causal, plus a
%  discrete sample period. Demanding lambda << L just pushes the loop gain
%  up at frequencies where the delay has already eaten the phase, and the
%  loop rings or goes unstable. Hence lambda >= 3L and >= 10*dt below.

clear; clc; close all;

%% ---- plant (edit, or let sysid_fit.m fill these in) -------------------
K        = 0.5492;     % dead-zone-corrected gain [m/s per unit duty]
tau      = 0.3020;     % time constant [s]
L        = 0.0597;     % transport delay [s]
deadzone = 0.150;      % static-friction duty (NOT part of G(s))
dt       = 0.02;       % control period [s] -- must match CTRL_HZ
Kgap     = 0.90;       % outer-loop gap gain [1/s]
PM_target = 60;        % phase-margin target for the lead compensator

s  = tf('s');
G  = K/(tau*s + 1);
Gd = G * exp(-L*s);

fprintf('PLANT\n');
fprintf('  K = %.4f, tau = %.4f s, L = %.4f s\n', K, tau, L);
fprintf('  open-loop bandwidth 1/tau = %.2f rad/s\n', 1/tau);
fprintf('  delay limits useful bandwidth to about %.2f rad/s\n', 1/(3*L));

%% ---- inner loop: lambda tuning ---------------------------------------
lambda = max([3*L, 10*dt, 0.3*tau]);
Kp = tau/(K*lambda);
Ki = Kp/tau;
Kd = 0;

C  = Kp + Ki/s + Kd*s;
Lo = C*Gd;                      % loop gain, delay included
Tcl = feedback(C*G, 1);         % nominal closed loop, delay neglected

fprintf('\nINNER LOOP (lambda = %.4f s)\n', lambda);
fprintf('  chosen as max(3L=%.3f, 10*dt=%.3f, 0.3*tau=%.3f)\n', ...
        3*L, 10*dt, 0.3*tau);
fprintf('  Kp = %.4f\n  Ki = %.4f\n  Kd = %.4f\n', Kp, Ki, Kd);
fprintf('  nominal closed loop: 1st order, tau_cl = %.3f s\n', lambda);
fprintf('    rise time  (10-90%%) = %.3f s\n', 2.2*lambda);
fprintf('    settling   (2%%)     = %.3f s\n', 3.9*lambda);
fprintf('    overshoot           = 0%% (pole-cancelled PI)\n');

[Gm, Pm, Wcg, Wcp] = margin(Lo);
fprintf('\nSTABILITY MARGINS (delay included)\n');
fprintf('  gain margin  : %.1f dB at %.2f rad/s\n', 20*log10(Gm), Wcg);
fprintf('  phase margin : %.1f deg at %.2f rad/s\n', Pm, Wcp);
if Pm >= 50
    fprintf('  verdict      : comfortable\n');
elseif Pm >= 35
    fprintf('  verdict      : acceptable\n');
else
    fprintf('  verdict      : TOO LOW -- expect ringing\n');
end

%% ---- cascade separation check ----------------------------------------
wInner = 1/lambda;
sep = wInner/Kgap;
fprintf('\nOUTER LOOP (constant time-gap)\n');
fprintf('  the gap plant is a pure integrator, so outer crossover = Kgap\n');
fprintf('  inner bandwidth ~ %.2f rad/s, outer ~ %.2f rad/s -> %.1fx apart\n', ...
        wInner, Kgap, sep);
if sep >= 5
    fprintf('  -> good: the loops can be designed independently.\n');
elseif sep >= 3
    fprintf('  -> marginal: expect some interaction between the loops.\n');
else
    fprintf('  -> TOO CLOSE: raise the inner bandwidth or drop Kgap below %.2f.\n', ...
            wInner/5);
end

%% ---- lead compensator -------------------------------------------------
fprintf('\nLEAD COMPENSATOR (target PM = %.0f deg)\n', PM_target);
phi = PM_target - Pm + 8;        % +8 deg for the crossover shift
if phi <= 1
    fprintf('  not needed -- PM is already %.1f deg.\n', Pm);
    fprintf('  Leave ENABLE_LEADLAG = 0 in config.h. Explaining why you do\n');
    fprintf('  not need a compensator is a better answer than adding one.\n');
    useLead = false;
else
    phi = min(phi, 60);
    alpha = (1-sind(phi))/(1+sind(phi));

    % new crossover: where |L| = sqrt(alpha)
    w  = logspace(-2, 3, 4000);
    m  = squeeze(abs(freqresp(Lo, w)));
    wm = interp1(m, w, sqrt(alpha), 'linear');

    z  = wm*sqrt(alpha);
    p  = wm/sqrt(alpha);
    Gc = (p/z) * (s + z)/(s + p);        % unity DC gain

    Gcd = c2d(Gc, dt, 'tustin');
    [bn, an] = tfdata(Gcd, 'v');
    b0 = bn(1)/an(1);  b1 = bn(2)/an(1);  a1 = an(2)/an(1);

    [~, Pm2] = margin(Gc*Lo);
    fprintf('  phase to add : %.1f deg\n', phi);
    fprintf('  alpha        : %.4f\n', alpha);
    fprintf('  zero / pole  : %.3f / %.3f rad/s\n', z, p);
    fprintf('  achieved PM  : %.1f deg\n', Pm2);
    fprintf('\n  paste into firmware/common/config.h:\n');
    fprintf('    #define ENABLE_LEADLAG   1\n');
    fprintf('    #define LL_B0   %+.6ff\n', b0);
    fprintf('    #define LL_B1   %+.6ff\n', b1);
    fprintf('    #define LL_A1   %+.6ff\n', a1);
    useLead = true;
end

%% ---- the paste block --------------------------------------------------
fprintf('\n%s\n', repmat('=',1,62));
fprintf('PASTE INTO config.h, THEN RUN: python tools/sync_common.py\n');
fprintf('%s\n', repmat('=',1,62));
fprintf('  #define PID_KP                 %.4ff\n', Kp);
fprintf('  #define PID_KI                 %.4ff\n', Ki);
fprintf('  #define PID_KD                 %.4ff\n', Kd);
fprintf('  #define K_GAP                  %.4ff\n', Kgap);
fprintf('  #define PLANT_K                %.4ff\n', K);
fprintf('  #define PLANT_TAU              %.4ff\n', tau);
fprintf('  #define PLANT_DELAY_S          %.4ff\n', L);
fprintf('  #define PLANT_DEADZONE         %.4ff\n', deadzone);

%% ---- figures ----------------------------------------------------------
figure('Name','Root locus','Position',[60 60 560 460]);
rlocus(C*G); grid on;
title(sprintf('Root locus, PI (K_p=%.2f, K_i=%.2f), delay neglected', Kp, Ki));

figure('Name','Bode / margins','Position',[640 60 620 560]);
margin(Lo); grid on;
title(sprintf('Loop gain with delay -- GM %.1f dB, PM %.1f deg', ...
      20*log10(Gm), Pm));

figure('Name','Nyquist','Position',[60 560 520 460]);
nyquist(Lo); grid on;
title('Nyquist -- encirclements of -1 determine closed-loop stability');

figure('Name','Step response','Position',[640 640 620 420]);
tsim = 0:dt:4;
step(Tcl, tsim); hold on; grid on;
yline(1,'k--');
title(sprintf('Nominal closed-loop step (predicted \\tau_{cl} = %.3f s)', lambda));
xlabel('time [s]'); ylabel('speed / setpoint');
legend('linear prediction','setpoint','Location','southeast');

if useLead
    figure('Name','Bode with lead','Position',[1280 60 620 560]);
    margin(Gc*Lo); grid on;
    title(sprintf('With lead compensator -- PM %.1f deg', Pm2));
end

fprintf('\nNOTE: every prediction above is from the LINEAR model. The real\n');
fprintf('plant has a %.3f-duty dead-zone and saturates at +/-1, so measured\n', deadzone);
fprintf('overshoot and settling will differ. Compare against the nonlinear\n');
fprintf('simulation to see how much:\n');
fprintf('  python python/scripts/sim_only.py --kp %.3f --ki %.3f --compare --fair\n', Kp, Ki);

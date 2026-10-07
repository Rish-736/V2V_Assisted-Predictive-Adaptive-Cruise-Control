%% sysid_fit.m -- fit the motor transfer function from step-response data
%  V2V-Assisted Predictive ACC / BECE302L
%
%  MATLAB companion to python/scripts/sysid_fit.py. Both are provided on
%  purpose: the Python one runs anywhere and feeds the digital twin; this
%  one uses the campus Control System Toolbox licence and produces the
%  figures the report wants.
%
%  USAGE
%    1. Capture data with firmware/tools/step_response (see the sketch
%       header), save it as data/step.csv
%    2. Run this script from the project root.
%
%  INPUT CSV columns:  t_ms, duty, speed_mps, enc_count, tag
%
%  IMPORTANT -- THE DEAD-ZONE
%  The per-level gain v_ss/duty is the APPARENT gain; it already includes
%  the static-friction loss, so it changes with operating point. The
%  dead-zone-corrected gain used by the simulator is the SLOPE of the
%  steady-state curve, K = slope*(1-dz). Using the apparent gain together
%  with a separate dead-zone subtracts the friction twice. See the
%  comments in python/scripts/sysid_fit.py for the algebra.

clear; clc; close all;

%% ---- configuration ----------------------------------------------------
csvFile      = fullfile('..','data','step.csv');
nominalDuty  = 0.60;      % operating point the nominal model should match
if ~isfile(csvFile)
    csvFile = fullfile('data','step.csv');
end
if ~isfile(csvFile)
    error(['Step data not found. Capture it with ' ...
           'firmware/tools/step_response and save as data/step.csv']);
end

%% ---- load -------------------------------------------------------------
T = readtable(csvFile, 'CommentStyle', '#');
t     = T.t_ms / 1000;
duty  = T.duty;
speed = T.speed_mps;

fprintf('loaded %d samples over %.1f s\n', height(T), t(end)-t(1));

%% ---- split into constant-duty segments --------------------------------
edges   = [1; find(abs(diff(duty)) > 1e-6) + 1; numel(duty)+1];
levels  = struct('duty',{},'K',{},'tau',{},'L',{},'vss',{},'fit',{}, ...
                 't',{},'v',{});

for k = 1:numel(edges)-1
    idx = edges(k):(edges(k+1)-1);
    if numel(idx) < 20, continue; end
    d = duty(idx(1));
    if d <= 1e-6, continue; end              % skip the coast-down phases

    ts = t(idx) - t(idx(1));
    vs = movmean(speed(idx), 7);

    vss = mean(vs(round(0.75*end):end));     % steady state
    if vss <= 1e-4, continue; end

    % dead time: first crossing of 5% of final value
    iL = find(vs >= 0.05*vss, 1, 'first');
    L  = ts(max(iL,1));

    % time constant: 63.2% point, measured after the dead time
    i63 = find(vs >= 0.632*vss, 1, 'first');
    tau = ts(max(i63,1)) - L;
    if tau <= 1e-4, continue; end

    % refine (K, tau, L) by least squares on the whole segment
    cost = @(p) sum((vs - stepModel(ts, d, p(1), p(2), p(3))).^2);
    p0   = [vss/d, tau, L];
    opts = optimset('Display','off','TolX',1e-8,'TolFun',1e-12);
    p    = fminsearch(cost, p0, opts);

    pred = stepModel(ts, d, p(1), p(2), p(3));
    fitPct = 100*(1 - norm(vs-pred)/norm(vs-mean(vs)));

    levels(end+1) = struct('duty',d,'K',p(1),'tau',p(2),'L',p(3), ...
                           'vss',vss,'fit',fitPct,'t',ts,'v',vs); %#ok<SAGROW>
end

if isempty(levels)
    error('No usable rising-step segments found.');
end

%% ---- report per level -------------------------------------------------
fprintf('\n%6s %9s %8s %8s %7s\n','duty','K_app','tau','L','fit%%');
fprintf('%s\n', repmat('-',1,44));
for k = 1:numel(levels)
    fprintf('%6.2f %9.4f %8.4f %8.4f %6.1f%%\n', ...
        levels(k).duty, levels(k).K, levels(k).tau, levels(k).L, levels(k).fit);
end

%% ---- static curve -> dead-zone and the CORRECTED gain -----------------
dutyPts = [levels.duty].';
vssPts  = [levels.vss].';
pfit    = polyfit(dutyPts, vssPts, 1);
slope   = pfit(1);
deadzone = max(0, min(0.9, -pfit(2)/slope));
K_corr   = slope * (1 - deadzone);

[~, iNom] = min(abs(dutyPts - nominalDuty));
tau_nom = levels(iNom).tau;
L_nom   = levels(iNom).L;

fprintf('\nstatic curve : v_ss = %.4f*duty %+.4f\n', slope, pfit(2));
fprintf('dead-zone    : %.4f duty\n', deadzone);
fprintf('CORRECTED K  : %.4f  (= slope*(1-dz); do NOT use the apparent K)\n', K_corr);

%% ---- the model --------------------------------------------------------
s = tf('s');
G = K_corr / (tau_nom*s + 1);
Gd = G * exp(-L_nom*s);                       % with transport delay

fprintf('\nNOMINAL MODEL\n');
fprintf('           %.4f\n', K_corr);
fprintf('  G(s) = --------- * exp(-%.4f s)\n', L_nom);
fprintf('         %.4fs+1\n', tau_nom);

%% ---- save for the Python side ----------------------------------------
params = struct('K',K_corr,'tau',tau_nom,'delay',L_nom, ...
                'deadzone',deadzone,'v_max',max(vssPts));
outDir = fileparts(csvFile);
jsonFile = fullfile(outDir,'plant_params_matlab.json');
fid = fopen(jsonFile,'w');
fprintf(fid,'{\n  "params": {\n');
fprintf(fid,'    "K": %.6f,\n', params.K);
fprintf(fid,'    "tau": %.6f,\n', params.tau);
fprintf(fid,'    "delay": %.6f,\n', params.delay);
fprintf(fid,'    "deadzone": %.6f,\n', params.deadzone);
fprintf(fid,'    "v_max": %.6f\n', params.v_max);
fprintf(fid,'  }\n}\n');
fclose(fid);
fprintf('\nsaved -> %s\n', jsonFile);
fprintf('(the Python tools read this file directly:\n');
fprintf('  python python/scripts/sim_only.py --params %s --compare --fair)\n', jsonFile);

%% ---- figures ----------------------------------------------------------
figure('Name','System identification','Position',[80 80 1200 420]);
n = numel(levels);
for k = 1:n
    subplot(1,n,k);
    plot(levels(k).t, levels(k).v, '.', 'Color',[.6 .6 .6], 'MarkerSize',4); hold on;
    pred = stepModel(levels(k).t, levels(k).duty, levels(k).K, ...
                     levels(k).tau, levels(k).L);
    plot(levels(k).t, pred, 'r-', 'LineWidth',1.8);
    yline(levels(k).vss, 'k:');
    yline(0.632*levels(k).vss, 'b:');
    xline(levels(k).L + levels(k).tau, 'b:');
    grid on; xlabel('time [s]');
    if k==1, ylabel('speed [m/s]'); end
    title(sprintf('duty %.2f\\newlineK=%.3f \\tau=%.3fs (%.0f%%)', ...
          levels(k).duty, levels(k).K, levels(k).tau, levels(k).fit));
    legend('measured','1st-order fit','Location','southeast');
end
sgtitle('Open-loop step response and first-order fit');

figure('Name','Static curve','Position',[120 120 560 420]);
plot(dutyPts, vssPts, 'o', 'MarkerFaceColor','b'); hold on;
dd = linspace(0,1,100);
plot(dd, polyval(pfit,dd), 'r-', 'LineWidth',1.6);
yline(0,'k-'); xline(deadzone,'k--');
text(deadzone, max(vssPts)*0.5, sprintf('  dead-zone = %.3f', deadzone));
grid on; xlabel('duty'); ylabel('steady-state speed [m/s]');
title('Static curve: the dead-zone is the x-intercept');
legend('measured','linear fit','Location','northwest');

%% ---- local function ---------------------------------------------------
function v = stepModel(t, duty, K, tau, L)
%STEPMODEL analytic first-order step response with dead time
    v = zeros(size(t));
    m = t >= L;
    v(m) = K*duty*(1 - exp(-(t(m)-L)/tau));
end

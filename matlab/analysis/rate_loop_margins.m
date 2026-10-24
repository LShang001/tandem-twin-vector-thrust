%% rate_loop_margins.m — 速率/姿态环频域裕度独立复核（MATLAB R2025b）
% 目的：用 Control System Toolbox 对固件速率环做连续域近似，
% 比较 fixed / detune70 / lead / AAF 的增益、相位与附加延迟裕度。
% 注意：这是 L1 线性近似，不含位置环、磁融合和真实振动；不能当实机结论。
%
% 运行：matlab -batch "run('matlab/analysis/rate_loop_margins.m')"
% 输出：sim-data/matlab-2026-10-25/rate_loop_margins.json / .png

clear; clc;
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outdir = fullfile(root, 'simulations', 'three_vehicle_mc', 'sim-data', 'matlab-2026-10-25');
if ~exist(outdir, 'dir'); mkdir(outdir); end

s = tf('s');
tau_act = 0.05;                  % 执行器/分配一阶滞后 [s]（fast_mc.py a_act）
alpha_out = 0.9; dt_ctrl = 0.005;
tau_out = -dt_ctrl / log(1 - alpha_out);  % 输出滤波等效时间常数 [s]
tau_delay = 0.020;               % SIL clean 的标称控制延迟 [s]

% 固件默认速率环增益（FlightCtrlParams.h）
axes = { ...
  struct('name','roll',  'kp',16.042818, 'ki',5.729578, 'att_kp',2.8), ...
  struct('name','pitch', 'kp',16.042818, 'ki',5.729578, 'att_kp',2.8), ...
  struct('name','yaw',   'kp',11.459156, 'ki',4.297184, 'att_kp',5.0)};

variants = { ...
  struct('name','fixed',    'scale',1.00, 'lead',0.0,  'aaf',0.0), ...
  struct('name','detune70', 'scale',0.70, 'lead',0.0,  'aaf',0.0), ...
  struct('name','lead05',   'scale',1.00, 'lead',0.05, 'aaf',0.0), ...
  struct('name','aaf08',    'scale',1.00, 'lead',0.0,  'aaf',0.80)};

results = struct('note', 'L1 continuous approximation; not flight evidence', ...
                 'tau_act_s', tau_act, 'tau_out_s', tau_out, ...
                 'nominal_delay_s', tau_delay, 'axes', {{}});
fig = figure('Visible','off');

for ai = 1:numel(axes)
  ax = axes{ai};
  P = (1/(tau_act*s + 1)) * (1/(tau_out*s + 1)) * (1/s);
  axres = struct('name', ax.name, 'variants', {{}});
  for vi = 1:numel(variants)
    v = variants{vi};
    C = v.scale*ax.kp + ax.ki/s;
    H = 1;
    if v.lead > 0
      H = (1 + v.lead*s) / (1 + 0.002*s);   % 滤波超前，避免 improper
    elseif v.aaf > 0
      H = (1 + v.aaf*s) / (1 + 0.002*s);    % 实测加速度反馈等效阻尼
    end
    L = C * H * P;
    L.InputDelay = tau_delay;
    m = allmargin(L);
    % 额外延迟裕度：allmargin 的 DelayMargin 是相对当前 InputDelay 的余量
    dm = NaN;
    if isfield(m,'DelayMargin') && ~isempty(m.DelayMargin)
      dm = min(m.DelayMargin);
    end
    pm = min(m.PhaseMargin);
    gm = min(m.GainMargin);
    gm_unbounded = isempty(gm) || any(~isfinite(gm)) || any(gm <= 0);
    if gm_unbounded
      gmdB = Inf;                    % 无相位穿越，增益裕度视为无穷
    else
      gmdB = 20*log10(gm);
    end
    % 姿态外环：角度误差 -> 速率指令（P），内环用 rate closed loop
    T = feedback(L, 1);
    Latt = ax.att_kp * T / s;
    ma = allmargin(Latt);
    pma = min(ma.PhaseMargin); gma = min(ma.GainMargin);
    axres.variants{end+1} = struct( ...
      'name', v.name, 'kp', v.scale*ax.kp, 'ki', ax.ki, ...
      'rate_pm_deg', pm, 'rate_gm_db', gmdB, ...
      'rate_gm_unbounded', gm_unbounded, 'rate_delay_margin_s', dm, ...
      'att_pm_deg', pma, 'att_gm_db', 20*log10(min(gma)));
    if strcmp(ax.name,'roll')
      subplot(2,1,1); hold on;
      bode(L, {2*pi*0.1, 2*pi*200});
      subplot(2,1,2); hold on;
    end
  end
  results.axes{end+1} = axres;
end

subplot(2,1,1); legend(variants{1}.name, variants{2}.name, variants{3}.name, variants{4}.name, 'Location','southwest');
title('Roll rate loop: fixed / detune70 / lead05 / aaf08');
grid on;
saveas(fig, fullfile(outdir,'rate_loop_bode.png'));
close(fig);

fid = fopen(fullfile(outdir,'rate_loop_margins.json'),'w');
fprintf(fid, '%s', jsonencode(results));
fclose(fid);
disp(jsonencode(results));

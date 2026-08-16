% =========================================================================
%  SCOTS Go2 — Grid Visualiser
%
%  Loads go2_*.bdd files via the SCOTS SymbolicSet interface and draws
%  the state-space grid, target set, and (projected) input-space grid
%  as overlapping scatter plots so you can visually judge the resolution
%  of each grid relative to the others and the arena.
%
%  No controller simulation is performed — this is purely a diagnostic
%  tool for choosing eta values in test.cpp.
%
%  USAGE
%  -----
%  1. Set BDD_DIR and SCOTS_MFILES below.
%  2. Run.  Use the sliders to change eta values and see the effect live
%     (the grids are redrawn analytically from the parameters you enter,
%     not reloaded from disk — so you can explore without recompiling).
%
%  HARDCODED PARAMETERS (match test.cpp exactly)
%  ----------------------------------------------
%  Edit the "Grid parameters" block below when you change test.cpp.
% =========================================================================

clear; clc; close all;

% =========================================================================
% USER CONFIGURATION
% =========================================================================
BDD_DIR      = '\\wsl.localhost\Ubuntu-20.04\home\cublab\Workspace-Linux\unitree_go2_dev\go2_dev\scots_dev\BDD Files\2026-5-23-1';
SCOTS_MFILES = '\\wsl.localhost\Ubuntu-20.04\home\cublab\Workspace-Linux\unitree_go2_dev\scots\mfiles';    % set if not already on path
addpath('\\wsl.localhost\Ubuntu-20.04\home\cublab\Workspace-Linux\unitree_go2_dev\scots\mfiles\mexfiles')
% =========================================================================

if ~isempty(SCOTS_MFILES), addpath(genpath(SCOTS_MFILES)); end

% =========================================================================
% Grid parameters — copy from test.cpp
% =========================================================================
% State space
s_lb  = [-5;  -5;  0   ];
s_ub  = [ 5;   5;  6.28];

% Target ellipsoid
H_ell = diag([.5, .5, 0.2]);
c_ell = [0; 0; 0];

% Input space
u_lb  = [-2.5; -1.0; -4.0];
u_ub  = [ 3.8;  1.0;  4.0];

% =========================================================================
% Helper: build a uniform grid vector
% =========================================================================
gv = @(lb, ub, eta) lb : eta : ub;

% =========================================================================
% Helper: ellipse patch (2-D projection of ellipsoid)
% =========================================================================
function [ex, ey] = ellipse_proj(c, H, dim1, dim2, n)
    th = linspace(0, 2*pi, n);
    ex = c(dim1) + cos(th)/sqrt(H(dim1,dim1));
    ey = c(dim2) + sin(th)/sqrt(H(dim2,dim2));
end

% =========================================================================
% Load BDD files if available (for target set overlay)
% =========================================================================
have_bdd = false;
target_pts = [];
if exist('SymbolicSet','class') || exist('SymbolicSet','file')
    tgt_path = fullfile(BDD_DIR,'go2_target.bdd');
    if isfile(tgt_path)
        try
            ts = SymbolicSet(tgt_path);
            % SymbolicSet has no direct grid-dump method in this build;
            % we approximate target cells by dense sampling
            have_bdd = true;
            fprintf('Loaded go2_target.bdd\n');
        catch e
            fprintf('Could not load BDD: %s\n', e.message);
        end
    end
end

% =========================================================================
% Interactive figure with eta controls
% =========================================================================
fig = figure('Name','SCOTS Go2 — Grid Visualiser', ...
             'Position',[60 60 1300 820], ...
             'Color',[0.12 0.12 0.15]);

% ---- Control panel on the right -----------------------------------------
panel = uipanel('Parent',fig, ...
    'Position',[0.78 0 0.22 1], ...
    'BackgroundColor',[0.18 0.18 0.22], ...
    'ForegroundColor',[0.9 0.9 0.9], ...
    'Title','Grid Parameters', ...
    'FontSize',12, 'FontWeight','bold');

% Default eta values
default_eta_s = [0.4,0.4, 0.2512];
default_eta_u = [0.63; 0.2; 0.8];

labels_s = {'eta\_x (state)','eta\_y (state)','eta\_yaw (state)'};
labels_u = {'eta\_vx (input)','eta\_vy (input)','eta\_vyaw (input)'};
ranges_s = {[0.1 2.0]; [0.1 2.0]; [0.05 1.0]};
ranges_u = {[0.05 1.0]; [0.01 0.5]; [0.05 1.0]};

sliders_s = gobjects(3,1);
edits_s   = gobjects(3,1);
sliders_u = gobjects(3,1);
edits_u   = gobjects(3,1);
counts_s  = gobjects(3,1);
counts_u  = gobjects(3,1);

y0 = 0.93;
dy = 0.09;

% helper to add a labelled slider row
function [sl, ed, ct] = add_slider_row(panel, label, val, rng, y)
    uicontrol('Parent',panel,'Style','text', ...
        'Units','normalized','Position',[0.03 y 0.94 0.035], ...
        'String',label,'FontSize',9,'FontWeight','bold', ...
        'BackgroundColor',[0.18 0.18 0.22],'ForegroundColor',[0.75 0.85 1.0], ...
        'HorizontalAlignment','left');
    sl = uicontrol('Parent',panel,'Style','slider', ...
        'Units','normalized','Position',[0.03 y-0.03 0.65 0.025], ...
        'Min',rng(1),'Max',rng(2),'Value',val, ...
        'BackgroundColor',[0.3 0.4 0.6]);
    ed = uicontrol('Parent',panel,'Style','edit', ...
        'Units','normalized','Position',[0.70 y-0.03 0.27 0.028], ...
        'String',sprintf('%.3f',val),'FontSize',9, ...
        'BackgroundColor',[0.25 0.25 0.30],'ForegroundColor',[1 1 1]);
    ct = uicontrol('Parent',panel,'Style','text', ...
        'Units','normalized','Position',[0.03 y-0.055 0.94 0.022], ...
        'String','','FontSize',8, ...
        'BackgroundColor',[0.18 0.18 0.22],'ForegroundColor',[0.6 0.8 0.6], ...
        'HorizontalAlignment','left');
end

uicontrol('Parent',panel,'Style','text', ...
    'Units','normalized','Position',[0.03 y0 0.94 0.03], ...
    'String','── STATE SPACE ──','FontSize',10,'FontWeight','bold', ...
    'BackgroundColor',[0.18 0.18 0.22],'ForegroundColor',[0.4 0.7 1.0], ...
    'HorizontalAlignment','center');
y0 = y0 - 0.03;
for i = 1:3
    [sliders_s(i), edits_s(i), counts_s(i)] = add_slider_row( ...
        panel, labels_s{i}, default_eta_s(i), ranges_s{i}, y0);
    y0 = y0 - dy;
end

y0 = y0 - 0.02;
uicontrol('Parent',panel,'Style','text', ...
    'Units','normalized','Position',[0.03 y0 0.94 0.03], ...
    'String','── INPUT SPACE ──','FontSize',10,'FontWeight','bold', ...
    'BackgroundColor',[0.18 0.18 0.22],'ForegroundColor',[1.0 0.7 0.3], ...
    'HorizontalAlignment','center');
y0 = y0 - 0.03;
for i = 1:3
    [sliders_u(i), edits_u(i), counts_u(i)] = add_slider_row( ...
        panel, labels_u{i}, default_eta_u(i), ranges_u{i}, y0);
    y0 = y0 - dy;
end

% Target centre controls
y0 = y0 - 0.02;
uicontrol('Parent',panel,'Style','text', ...
    'Units','normalized','Position',[0.03 y0 0.94 0.03], ...
    'String','── TARGET CENTRE ──','FontSize',10,'FontWeight','bold', ...
    'BackgroundColor',[0.18 0.18 0.22],'ForegroundColor',[0.4 1.0 0.5], ...
    'HorizontalAlignment','center');
y0 = y0 - 0.03;
tgt_labels = {'c\_x','c\_y'};
tgt_defaults = [c_ell(1); c_ell(2)];
tgt_ranges   = {[-5 5]; [-5 5]};
sliders_c = gobjects(2,1);
edits_c   = gobjects(2,1);
for i = 1:2
    [sliders_c(i), edits_c(i), ~] = add_slider_row( ...
        panel, tgt_labels{i}, tgt_defaults(i), tgt_ranges{i}, y0);
    y0 = y0 - dy;
end

% Summary text
summary_txt = uicontrol('Parent',panel,'Style','text', ...
    'Units','normalized','Position',[0.03 0.01 0.94 0.12], ...
    'String','','FontSize',8, ...
    'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',[0.9 0.9 0.7], ...
    'HorizontalAlignment','left');

% ---- Axes ---------------------------------------------------------------
ax1 = axes('Parent',fig,'Position',[0.03 0.52 0.72 0.45], ...
           'Color',[0.08 0.08 0.11],'XColor',[0.7 0.7 0.7], ...
           'YColor',[0.7 0.7 0.7],'GridColor',[0.3 0.3 0.3], ...
           'GridAlpha',0.4);
ax2 = axes('Parent',fig,'Position',[0.03 0.05 0.72 0.42], ...
           'Color',[0.08 0.08 0.11],'XColor',[0.7 0.7 0.7], ...
           'YColor',[0.7 0.7 0.7],'GridColor',[0.3 0.3 0.3], ...
           'GridAlpha',0.4);

% =========================================================================
% Draw function
% =========================================================================
function draw(ax1, ax2, sliders_s, sliders_u, sliders_c, ...
              s_lb, s_ub, u_lb, u_ub, H_ell, ...
              counts_s, counts_u, summary_txt)

    eta_s = arrayfun(@(s) s.Value, sliders_s);
    eta_u = arrayfun(@(s) s.Value, sliders_u);
    cx    = sliders_c(1).Value;
    cy    = sliders_c(2).Value;
    c2    = [cx; cy];

    % Grid vectors
    x_v   = s_lb(1):eta_s(1):s_ub(1);
    y_v   = s_lb(2):eta_s(2):s_ub(2);
    vx_v  = u_lb(1):eta_u(1):u_ub(1);
    vy_v  = u_lb(2):eta_u(2):u_ub(2);

    nx = numel(x_v); ny = numel(y_v);
    nvx = numel(vx_v); nvy = numel(vy_v);

    % Grid point counts
    ns_total = nx * ny * round((s_ub(3)-s_lb(3))/eta_s(3)+1);
    nu_total = nvx * nvy * round((u_ub(3)-u_lb(3))/eta_u(3)+1);

    for i=1:3
        n = round((s_ub(i)-s_lb(i))/eta_s(i)+1);
        counts_s(i).String = sprintf('  %d points', n);
    end
    for i=1:3
        n = round((u_ub(i)-u_lb(i))/eta_u(i)+1);
        counts_u(i).String = sprintf('  %d points', n);
    end

    summary_txt.String = sprintf( ...
        'State pts (XY): %d × %d = %d\nTotal state pts: %s\nInput pts (VxVy): %d × %d = %d\nTotal input pts: %s\nTransition combos: ~%s', ...
        nx, ny, nx*ny, ...
        format_large(ns_total), ...
        nvx, nvy, nvx*nvy, ...
        format_large(nu_total), ...
        format_large(ns_total * nu_total));

    % ---- Top axes: X-Y state space + target -----------------------------
    cla(ax1);
    axes(ax1); %#ok<LAXES>
    hold(ax1,'on');

    % State grid dots
    [GX, GY] = ndgrid(x_v, y_v);
    scatter(ax1, GX(:), GY(:), 8, [0.25 0.35 0.55], 'filled', ...
            'MarkerFaceAlpha', 0.5, 'DisplayName', ...
            sprintf('State grid (%d×%d pts)', nx, ny));

    % Target ellipse
    th = linspace(0,2*pi,300);
    ex = cx + cos(th)/sqrt(H_ell(1,1));
    ey = cy + sin(th)/sqrt(H_ell(2,2));
    fill(ax1, ex, ey, [0.2 0.8 0.4], 'FaceAlpha',0.2, ...
         'EdgeColor',[0.2 0.9 0.4],'LineWidth',2, ...
         'DisplayName','Target ellipsoid');
    plot(ax1, cx, cy, '+', 'Color',[0.2 0.9 0.4], ...
         'MarkerSize',12,'LineWidth',2,'HandleVisibility','off');

    % Arena boundary
    rectangle('Parent',ax1, ...
        'Position',[s_lb(1) s_lb(2) (s_ub(1)-s_lb(1)) (s_ub(2)-s_lb(2))], ...
        'EdgeColor',[0.6 0.6 0.6],'LineWidth',1.5,'LineStyle','--');

    % Eta cell rectangle (one example cell at origin)
    rectangle('Parent',ax1, ...
        'Position',[-eta_s(1)/2 -eta_s(2)/2 eta_s(1) eta_s(2)], ...
        'EdgeColor',[1 0.6 0.1],'LineWidth',1.5,'LineStyle','-', ...
        'FaceColor',[1 0.6 0.1 0.08]);
    text(ax1, eta_s(1)/2+0.05, eta_s(2)/2+0.05, ...
         sprintf('cell: %.2f×%.2f m', eta_s(1), eta_s(2)), ...
         'Color',[1 0.7 0.2],'FontSize',8);

    set(ax1,'FontSize',10,'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7]);
    xlabel(ax1,'x_{pos} [m]','Color',[0.8 0.8 0.8],'FontSize',11);
    ylabel(ax1,'y_{pos} [m]','Color',[0.8 0.8 0.8],'FontSize',11);
    title(ax1, sprintf('State Space X-Y Grid  |  %d × %d = %d points  (eta=[%.2f, %.2f, %.3f])', ...
        nx, ny, nx*ny, eta_s(1), eta_s(2), eta_s(3)), ...
        'Color',[0.85 0.85 0.85],'FontSize',11);
    axis(ax1,'equal');
    xlim(ax1,[s_lb(1)-0.5 s_ub(1)+0.5]);
    ylim(ax1,[s_lb(2)-0.5 s_ub(2)+0.5]);
    grid(ax1,'on'); legend(ax1,'Location','northeast','FontSize',9, ...
        'TextColor',[0.85 0.85 0.85],'Color',[0.15 0.15 0.2]);

    % ---- Bottom axes: Vx-Vy input space ---------------------------------
    cla(ax2);
    axes(ax2); %#ok<LAXES>
    hold(ax2,'on');

    [GVX, GVY] = ndgrid(vx_v, vy_v);
    scatter(ax2, GVX(:), GVY(:), 8, [0.8 0.5 0.15], 'filled', ...
            'MarkerFaceAlpha', 0.5, 'DisplayName', ...
            sprintf('Input grid (%d×%d pts)', nvx, nvy));

    % Input cell rectangle
    rectangle('Parent',ax2, ...
        'Position',[-eta_u(1)/2 -eta_u(2)/2 eta_u(1) eta_u(2)], ...
        'EdgeColor',[0.4 0.8 1.0],'LineWidth',1.5, ...
        'FaceColor',[0.4 0.8 1.0 0.08]);
    text(ax2, eta_u(1)/2+0.02, eta_u(2)/2+0.02, ...
         sprintf('cell: %.3f×%.3f m/s', eta_u(1), eta_u(2)), ...
         'Color',[0.4 0.8 1.0],'FontSize',8);

    % Zero-input marker
    plot(ax2, 0, 0, 'w+','MarkerSize',12,'LineWidth',2, ...
         'DisplayName','Zero input');

    set(ax2,'FontSize',10,'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7]);
    xlabel(ax2,'v_x [m/s]','Color',[0.8 0.8 0.8],'FontSize',11);
    ylabel(ax2,'v_y [m/s]','Color',[0.8 0.8 0.8],'FontSize',11);
    title(ax2, sprintf('Input Space Vx-Vy Grid  |  %d × %d = %d points  (eta=[%.3f, %.3f, %.3f])', ...
        nvx, nvy, nvx*nvy, eta_u(1), eta_u(2), eta_u(3)), ...
        'Color',[0.85 0.85 0.85],'FontSize',11);
    axis(ax2,'equal');
    xlim(ax2,[u_lb(1)-0.2 u_ub(1)+0.2]);
    ylim(ax2,[u_lb(2)-0.1 u_ub(2)+0.1]);
    grid(ax2,'on'); legend(ax2,'Location','northeast','FontSize',9, ...
        'TextColor',[0.85 0.85 0.85],'Color',[0.15 0.15 0.2]);

    drawnow;
end

function s = format_large(n)
    if n >= 1e12
        s = sprintf('%.2f T', n/1e12);
    elseif n >= 1e9
        s = sprintf('%.2f B', n/1e9);
    elseif n >= 1e6
        s = sprintf('%.2f M', n/1e6);
    elseif n >= 1e3
        s = sprintf('%.1f K', n/1e3);
    else
        s = sprintf('%d', round(n));
    end
end

% =========================================================================
% Wire up callbacks
% =========================================================================
cb = @(~,~) draw(ax1, ax2, sliders_s, sliders_u, sliders_c, ...
                  s_lb, s_ub, u_lb, u_ub, H_ell, ...
                  counts_s, counts_u, summary_txt);

for i = 1:3
    addlistener(sliders_s(i), 'Value', 'PostSet', cb);
    addlistener(sliders_u(i), 'Value', 'PostSet', cb);
    % Sync edit box → slider
    edits_s(i).Callback = @(src,~) set_slider_from_edit(src, sliders_s(i), cb);
    edits_u(i).Callback = @(src,~) set_slider_from_edit(src, sliders_u(i), cb);
end
for i = 1:2
    addlistener(sliders_c(i), 'Value', 'PostSet', cb);
    edits_c(i).Callback = @(src,~) set_slider_from_edit(src, sliders_c(i), cb);
end

function set_slider_from_edit(src, sl, cb)
    v = str2double(src.String);
    if ~isnan(v)
        v = max(sl.Min, min(sl.Max, v));
        sl.Value = v;
        src.String = sprintf('%.3f', v);
        cb([], []);
    end
end

% Initial draw
draw(ax1, ax2, sliders_s, sliders_u, sliders_c, ...
     s_lb, s_ub, u_lb, u_ub, H_ell, ...
     counts_s, counts_u, summary_txt);

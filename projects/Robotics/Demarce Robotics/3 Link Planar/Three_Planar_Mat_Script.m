clc; clear; close all;

% --- Figure setup ---
f1 = figure('Name','3-Link Planar Manipulator with Sliders','NumberTitle','off');
axis equal; grid on;
xlim([-8 8]); ylim([-8 8]);
xlabel('X (m)'); ylabel('Y (m)');
title('Interactive 3-Link Planar Manipulator');
hold on;

% --- Circle parameters (for joints) ---
r = 0.2; 
N = 200; 
t = linspace(0,2*pi,N);

% --- Link lengths ---
L1 = 4; L2 = 2; L3 = 1;

% --- Initial plots (horizontal orientation) ---
Link1 = plot([0 L1],[0 0],'g-','LineWidth',3);
Link2 = plot([L1 L1+L2],[0 0],'r-','LineWidth',3);
Link3 = plot([L1+L2 L1+L2+L3],[0 0],'b-','LineWidth',3);

Joint1 = patch(r*cos(t), r*sin(t), 'k'); % Base joint
Joint2 = patch(L1+r*cos(t), r*sin(t), 'k'); % 2nd joint
Joint3 = patch((L1+L2)+r*cos(t), r*sin(t), 'k'); % 3rd joint
EndEff = patch((L1+L2+L3)+r*cos(t), r*sin(t), 'm'); % End-effector

% --- Text box for dispalyingh joint positions  ---
posText = uicontrol('Style','text','Units','normalized',...
    'Position',[0.78 0.35 0.2 0.5],'FontSize',11,...
    'HorizontalAlignment','left','BackgroundColor',[0.95 0.95 0.95],...
    'String','Joint Positions:');

% --- Sliders ---
theta1Slider = uicontrol('Style','slider','Min',-180,'Max',180,'Value',0,...
    'Position',[100 50 150 20],'Callback',@updateArm1);
uicontrol('Style','text','Position',[100 70 150 20],'String','Theta1 (deg)');

theta2Slider = uicontrol('Style','slider','Min',-180,'Max',180,'Value',0,...
    'Position',[300 50 150 20],'Callback',@updateArm1);
uicontrol('Style','text','Position',[300 70 150 20],'String','Theta2 (deg)');

theta3Slider = uicontrol('Style','slider','Min',-180,'Max',180,'Value',0,...
    'Position',[500 50 150 20],'Callback',@updateArm1);
uicontrol('Style','text','Position',[500 70 150 20],'String','Theta3 (deg)');

% --- Store handles in structure ---
data.L1 = L1; data.L2 = L2; data.L3 = L3;
data.r = r; data.t = t;
data.Link1 = Link1; data.Link2 = Link2; data.Link3 = Link3;
data.Joint1 = Joint1; data.Joint2 = Joint2; data.Joint3 = Joint3; data.EndEff = EndEff;
data.theta1Slider = theta1Slider; 
data.theta2Slider = theta2Slider;
data.theta3Slider = theta3Slider;
data.posText = posText;
guidata(f1, data);

% === Update function (called whenever a slider moves) ===
function updateArm1(src,~)
    data = guidata(src);

    % --- Read angles (in radians) ---
    th1 = deg2rad(get(data.theta1Slider,'Value'));
    th2 = deg2rad(get(data.theta2Slider,'Value'));
    th3 = deg2rad(get(data.theta3Slider,'Value'));

    % --- Denavit-Hartenberg matrices (planar case: alpha=0, d=0) ---
    A1 = dh(data.L1, 0, 0, th1);
    A2 = dh(data.L2, 0, 0, th2);
    A3 = dh(data.L3, 0, 0, th3);

    % --- Cumulative transformations ---
    T01 = A1;
    T02 = A1 * A2;
    T03 = A1 * A2 * A3;

    % --- Extract joint positions (from last column) ---
    O0 = [0;0;1];
    O1 = T01(1:3,4);
    O2 = T02(1:3,4);
    O3 = T03(1:3,4); % end-effector position

    % --- Update link lines ---
    set(data.Link1,'XData',[O0(1) O1(1)],'YData',[O0(2) O1(2)]);
    set(data.Link2,'XData',[O1(1) O2(1)],'YData',[O1(2) O2(2)]);
    set(data.Link3,'XData',[O2(1) O3(1)],'YData',[O2(2) O3(2)]);

    % --- Update joint markers ---
    set(data.Joint1,'XData',O0(1)+data.r*cos(data.t),'YData',O0(2)+data.r*sin(data.t));
    set(data.Joint2,'XData',O1(1)+data.r*cos(data.t),'YData',O1(2)+data.r*sin(data.t));
    set(data.Joint3,'XData',O2(1)+data.r*cos(data.t),'YData',O2(2)+data.r*sin(data.t));
    set(data.EndEff,'XData',O3(1)+data.r*cos(data.t),'YData',O3(2)+data.r*sin(data.t));

     % --- Display positions in side panel ---
    infoStr = sprintf(['Joint Positions:\n' ...
                       'O1: (%.2f , %.2f)\n' ...
                       'O2: (%.2f , %.2f)\n' ...
                       'O3: (%.2f , %.2f)\n'],...
                       O1(1),O1(2), O2(1),O2(2), O3(1),O3(2));
    set(data.posText,'String',infoStr);

    % --- Display current end-effector coordinates ---
    title(sprintf('End Effector: (%.2f , %.2f)', O3(1), O3(2)));

    drawnow;
end

close all
clc


%% Drone parameters
MASS = 0.027; % kg
GRAVITY = [0;0;9.81];
INERTIA = [16.6e-6, 0.83e-6, 0.72e-6;
    0.83e-6, 16.6e-6, 1.8e-6;
    0.72e-6, 1.8e-6, 29.3e-6]; %from bitcraze
BODY_FRAME_THRUST = [0;0;1];

%% Simulation parameters
SIM_TIME = 25; % seconds
DELTA_TIME = 0.001;
TIME_STEPS = SIM_TIME/DELTA_TIME;
i = 1;
t = 0;

MAX_TRANS_C = 5;
MAX_ROT_C = 10.0;

%% Initialize state variables
stop = 3;
slide = 0.4;
% position = [slide+cos(3+t);sin(3+t);sin(0.5*t)*.5]/3;
position = [0;0;-0.3];
position_pert = [0;0;0];
orientation = quaternion(1,0,0,0);
orientation = normalize(orientation);
velocity = [0;0;0];
velocity_error = velocity;
angular_velocity = [0;0;0];

%% Control Variables

%Translational control gains
trans_kp_o = [10;10;10];
trans_kd_o = [6;6;6];

trans_kp = trans_kp_o;
trans_kd = trans_kd_o;

trans_kp_dot = [0;0;0];
trans_kd_dot = [0;0;0];

%Rotational control gains
ang_kp_o = [90;90;90];
ang_kd_o = [40;40;40];

ang_kp_dot = [0;0;0];
ang_kd_dot = [0;0;0];

ang_kp = ang_kp_o;
ang_kd = ang_kd_o;

lambda = [50 50 1.5 0.5];
alpha = [3 0.05 1 0.05];

restriction_radius = 0.5;
lambda_max = restriction_radius^2;

%% Desired variables


desired_orientation = quaternion(1,0,0,0);
desired_position = position;
desired_position_unmodified = [0;0;0];

% Error
orientation_error = orientation;
position_error = position - desired_position;


%% Auxiliary variables for plotting
plot_position = zeros(3, TIME_STEPS);
plot_desired_position = zeros(3, TIME_STEPS);
plot_position_norm = zeros(1, TIME_STEPS);
plot_position_norm_error = zeros(1, TIME_STEPS);
plot_constraint = zeros(1, TIME_STEPS);
plot_desired_position_norm = zeros(1, TIME_STEPS);
plot_position_norm_erro = zeros(1, TIME_STEPS);
plot_attitude = zeros(4, TIME_STEPS);
plot_desired_attitude = zeros(4, TIME_STEPS);
plot_attitude_error = zeros(4, TIME_STEPS);
plot_angular_error = zeros(1, TIME_STEPS);
plot_trans_gains = zeros(3, TIME_STEPS);
plot_trans_kd = zeros(3, TIME_STEPS);
plot_torques = zeros(3, TIME_STEPS);
plot_thrust = zeros(1, TIME_STEPS);
time = zeros(1, TIME_STEPS);

discretization_v = 0.002;
step = 0;
size = 0.8;
%% Simulation loop
while t < SIM_TIME
    
    slide = 0.15;
    step = step+0.0005;
    % step = step+0.001;
    if t < 20
        size = size+0.00004;
    else
        size = size-0.00004;
    end

    desired_position = [slide+cos(3+step)*size;slide+sin(3+step)*size;sin(0.5*step)*size*.5]/3;

    position_pert(1) = 0;
    position_pert(2) = 0;
    
    

    position_error_prev  = position_error; 
    r = sqrt(position(1)^2 + position(2)^2 + position(3)^2); %position relative to the origin
    rd = sqrt(desired_position(1)^2 + desired_position(2)^2 + desired_position(3)^2);
    
    plot_desired_position(:, i) = desired_position;
    plot_desired_position_norm(:, i) = rd;
    
    reduction_v = 0.999;
    increment_v = 1.2;
    theta = atan2(desired_position(2),desired_position(1));
    beta = acos(desired_position(3)/rd);
    
    if r >= restriction_radius*reduction_v
        position(1) = restriction_radius*reduction_v*sin(beta)*cos(theta);
        position(2) = restriction_radius*reduction_v*sin(beta)*sin(theta);
        position(3) = restriction_radius*reduction_v*cos(beta);
        r = sqrt(position(1)^2 + position(2)^2 + position(3)^2);
    end
    if abs(desired_position(1)) > abs(restriction_radius*increment_v*sin(beta)*cos(theta))
        desired_position(1) = restriction_radius*increment_v*sin(beta)*cos(theta);
    end
    if abs(desired_position(2)) > abs(restriction_radius*increment_v*sin(beta)*sin(theta))
        desired_position(2) = restriction_radius*increment_v*sin(beta)*sin(theta);
    end
    if abs(desired_position(3)) > abs(restriction_radius*increment_v*cos(beta))
        desired_position(3) = restriction_radius*increment_v*cos(beta);
    end
    position_error = position - desired_position + position_pert;
    velocity_error = ((position_error - position_error_prev)/DELTA_TIME);
    
    t = t + DELTA_TIME;
    
    approx_v = 1;

    % Estimate translational control gains
    for j = 1:3    
        trans_kp_dot(j) = ( -lambda(1)/(restriction_radius-r))*abs(position_error(j)) + lambda(2)*(trans_kp_o(j) - trans_kp(j));
        trans_kd_dot(j) = lambda(3)*(velocity_error(j)) + lambda(4)*(trans_kd_o(j) - trans_kd(j));
        trans_kp(j) = trans_kp(j)*approx_v + trans_kp_dot(j) * DELTA_TIME;
        trans_kd(j) = trans_kd(j)*approx_v + trans_kd_dot(j) * DELTA_TIME;
    end
    
    % Translational Control
    trans_control = -MASS*(trans_kp.*position_error + trans_kd.*(velocity_error) + (-GRAVITY));
    
    if norm(trans_control)~=0
        trans_control = -MASS*(trans_kp.*position_error + trans_kd.*(velocity_error) + (-GRAVITY));
        trans_control = MAX_TRANS_C*tanh(norm(trans_control)/MAX_TRANS_C)*trans_control/norm(trans_control);
    end
    
    norm_trans_control = norm(trans_control);
    control_direction = [0;0;0];
    if norm_trans_control ~= 0
        control_direction = trans_control/norm_trans_control; %Fu
    end
    
    [~,frx,fry,frz] = parts(orientation*quaternion(0,0,0,1)*(orientation')); %Fth
    control_thrust = trans_control(3)/dot([0;0;1],[frx;fry;frz]);
    
    
    % Orientation Control
    desired_orientation = exp(0.5*log(quaternion([dot([0;0;1],control_direction);[cross([0;0;1],control_direction)]]')));
    desired_orientation = normalize(desired_orientation);
    orientation_error_prev =  orientation_error;
    orientation_error = orientation*(desired_orientation');
    orientation_error_vector = rotvec(orientation_error)'; %Euler angle representation
    angular_velocity_error = rotvec(orientation_error*orientation_error_prev')'/DELTA_TIME;
    
    if norm(rotvec(orientation_error)) > pi || norm(rotvec(orientation_error))< -pi
        disp('Inverting reference')
        desired_orientation = -desired_orientation;
        orientation_error = (desired_orientation)*(orientation');
    end
    
    % Estimate rotational control gains
    for k = 1:3
        ang_kp_dot(k) = alpha(1)*abs(orientation_error_vector(k)) + alpha(2)*(ang_kp_o(k) - ang_kp(k));
        ang_kd_dot(k) = alpha(3)*(angular_velocity_error(k)) + alpha(4)*(ang_kd_o(k) - ang_kd(k));
        
        ang_kp(k) = ang_kp(k)*approx_v + ang_kp_dot(k) * DELTA_TIME;
        ang_kd(k) = ang_kd(k)*approx_v + ang_kd_dot(k) * DELTA_TIME;
    end
    
    orientation_control = (-ang_kp.*orientation_error_vector)-(ang_kd.*angular_velocity_error);
    torque = INERTIA*(orientation_control + cross(angular_velocity,INERTIA*angular_velocity));
    
    % Update orientation and position
    [~,ax,ay,az] = parts(orientation*quaternion(0,0,0,norm(control_thrust)/MASS)*(orientation'));
    az = az + (-GRAVITY(3));
    velocity =  velocity + ([ax;ay;az])* DELTA_TIME;
    position = position + velocity * DELTA_TIME;
    orientation_dot = 0.5*quaternion([0,angular_velocity'])*orientation;
    orientation = orientation + orientation_dot*DELTA_TIME;
    orientation = normalize(orientation);
    dot_w = (INERTIA)\(torque- cross(angular_velocity,INERTIA*angular_velocity));
    angular_velocity = angular_velocity + dot_w * DELTA_TIME;
    
    % Update plotting matrix
    plot_position(:, i) = position;
    plot_position_norm(:, i) = r;
    plot_position_norm_error(:, i) = r - rd;
    plot_constraint(:,i) = restriction_radius;
    [plot_attitude(1,i),plot_attitude(2,i),plot_attitude(3,i),plot_attitude(4,i)] = parts(orientation);
    [plot_desired_attitude(1,i),plot_desired_attitude(2,i),plot_desired_attitude(3,i),plot_desired_attitude(4,i)] = parts(desired_orientation);
    [plot_attitude_error(1,i),plot_attitude_error(2,i),plot_attitude_error(3,i),plot_attitude_error(4,i)] = parts(orientation_error);
    plot_angular_error(i) = norm(rotvec(orientation_error));
    plot_trans_gains(:,i) = trans_kp;
    plot_trans_kd(:,i) = trans_kd;
    plot_torques(:, i) = torque;
    if control_thrust < 0
        plot_thrust(:, i) = 0;
    else
        plot_thrust(:, i) = control_thrust;
    end
    
    time(i) = t;
    i = i + 1;
    
end


%% Plotting
marker_space = 1 :400: 23000;
figure;
set(gcf,"Color",[1,1,1])
set(gcf,"Position",[238,182,1074,780])
% set(gcf,"Renderer","painters")
plot3(plot_position(1, :), plot_position(2, :), plot_position(3, :),'LineWidth',5);
hold on
% plot3(plot_desired_position(1, :), plot_desired_position(2, :), plot_desired_position(3, :), 'LineStyle', 'none' , 'Marker', 'square', MarkerSize=10, MarkerFaceColor='r');
plot3(plot_desired_position(1, :), plot_desired_position(2, :), plot_desired_position(3, :), 'r-.', 'LineWidth',5);
[sx,sy,sz] = sphere(60);
s = surf(sx*restriction_radius,sy*restriction_radius,sz*restriction_radius);
s.FaceAlpha = 0.5;
s.EdgeColor = 'none';
s.FaceColor = 'interp';
xlabel('Coord. X [m]');
ylabel('Coord. Y [m]');
zlabel('Coord. Z [m]');
legend('Position ', 'Reference ', 'FontSize', 24, Box="off", Position=[0.790122593906473,0.78076923076923,0.201505277736402,0.115384615384616]);
set(gca, "XLim", [-0.65 0.65])
set(gca, "YLim", [-0.65 0.65])
set(gca, "ZLim", [-0.65 0.65])
set(gca, "OuterPosition", [0,0.089918404228575,1,1.108031170807595])
set(gca, "InnerPosition", [0.13,0.21180183301741,0.775,0.90304540420819])
set(gca, "Position", [0.13,0.21180183301741,0.775,0.90304540420819])
set(gca, "DataAspectRatio", [1 1 1])
set(gca, "PlotBoxAspectRatio", [1 1 1])
set(gca,"LineWidth", 2)
set(gca,"FontSize", 20)
set(gca,"LabelFontSizeMultiplier", 1.5)
view([45,40])
grid off;
print('./plots/restricted_control/3D_Trajectory', '-depsc2')

% figure;
% set(gcf,"Color",[1,1,1])
% set(gcf,"Position",[80,220,1850,525])
% set(gcf,"Renderer","painters")
% plot(time, plot_attitude(1, :), 'k','LineWidth',2.5);
% hold on
% plot(time, plot_attitude(2, :), 'r','LineWidth',2.5);
% plot(time, plot_attitude(3, :), 'g','LineWidth',2.5);
% plot(time, plot_attitude(4, :), 'b','LineWidth',2.5);
% plot(time, plot_desired_attitude(1, :), 'k-.','LineWidth',2.5);
% plot(time, plot_desired_attitude(2, :), 'r-.','LineWidth',2.5);
% plot(time, plot_desired_attitude(3, :), 'g-.','LineWidth',2.5);
% plot(time, plot_desired_attitude(4, :), 'b-.','LineWidth',2.5);
% xlim([0 SIM_TIME])
% legend('$q_0$','$q_1$','$q_2$','$q_3$','$q_{r0}$','$q_{r1}$','$q_{r2}$','$q_{r3}$', ...
%     'FontSize',24, 'interpreter', 'latex', ...
%     'Color', [1,1,1], 'EdgeColor', [1,1,1], ...
%     'Position', [0.92,0.45,0.05,0.35]);
% set(gca,"FontSize", 15);
% xlabel('Time [s]', 'FontSize',24);
% ylabel('Attitude Quaternion', 'FontSize',24);
% box("off")
% grid off;
% print('./plots/restricted_control/Attitude', '-depsc2')
% hold off


% figure;
% set(gcf,"Color",[1,1,1])
% set(gcf,"Position",[80,220,1850,525])
% set(gcf,"Renderer","painters")
% plot(time,plot_desired_position(1, :), 'r','LineWidth',5);
% hold on
% plot(time,plot_position(1, :), 'r-.','LineWidth',0.5, 'Marker','o','MarkerEdgeColor', 'k', 'MarkerSize', 12, 'MarkerFaceColor',[1,0.5,0],'MarkerIndices',marker_space);
% % plot(time,plot_position_adaptive(1, :), 'b','LineWidth',2.5);
% % plot(time,plot_desired_position_adaptive(1, :), 'b-.','LineWidth',2.5);
% ylim([-1 1])
% xlim([0 SIM_TIME])
% legend('$p_x$', '$p_{xr}$', ...
%     'FontSize',24, 'interpreter', 'latex', ...
%     'Color', [1,1,1], 'EdgeColor', [1,1,1], ...
%     'Position', [0.92,0.75,0.052,0.1548]);
% set(gca,"FontSize", 15);
% xlabel('Time [s]','FontSize',24);
% ylabel('Position [m]','FontSize',24);
% grid off;
% box("off")
% print('./plots/restricted_control/Position1', '-depsc2')
% hold off

% figure;
% set(gcf,"Color",[1,1,1])
% set(gcf,"Position",[80,220,1850,525])
% set(gcf,"Renderer","painters")
% plot(time,plot_desired_position(2, :), 'g','LineWidth',2.5);
% hold on
% plot(time,plot_position(2, :), 'g-.','LineWidth',.5, 'Marker','o','MarkerEdgeColor', 'k','MarkerFaceColor',[1.00,0.41,0.16],'MarkerIndices',marker_space);
% ylim([-1 1])
% xlim([0 SIM_TIME])
% legend('$p_y$','$p_{yr}$', ...
%     'FontSize',24, 'interpreter', 'latex', ...
%     'Color', [1,1,1], 'EdgeColor', [1,1,1], ...
%     'Position', [0.92,0.75,0.052,0.1548]);
% set(gca,"FontSize", 15);
% xlabel('Time [s]','FontSize',24);
% ylabel('Position [m]','FontSize',24);
% grid off;
% box("off")
% print('./plots/restricted_control/Position2', '-depsc2')
% hold off

% figure;
% set(gcf,"Color",[1,1,1])
% set(gcf,"Position",[80,220,1850,525])
% set(gcf,"Renderer","painters")
% plot(time,plot_desired_position(3, :), 'b','LineWidth',2.5);
% hold on
% plot(time,plot_position(3, :), 'b-.','LineWidth',.5, 'Marker','o','MarkerEdgeColor', 'k','MarkerFaceColor',[0.40,0.90,0.05],'MarkerIndices',marker_space);
% ylim([-1 1])
% xlim([0 SIM_TIME])
% legend('$p_z$','$p_{zr}$', ...
%     'FontSize',24, 'interpreter', 'latex', ...
%     'Color', [1,1,1], 'EdgeColor', [1,1,1], ...
%     'Position', [0.92,0.75,0.052,0.1548]);
% set(gca,"FontSize", 15);
% xlabel('Time [s]','FontSize',24);
% ylabel('Position [m]','FontSize',24);
% grid off;
% box("off")
% print('./plots/restricted_control/Position3', '-depsc2')
% hold off


% figure;
% set(gcf,"Color",[1,1,1])
% set(gcf,"Position",[80,220,1850,525])
% set(gcf,"Renderer","painters")
% plot(time,plot_desired_position_norm(1, :), 'r', 'LineWidth',5)
% hold on
% plot(time,plot_position_norm(1, :), 'k-.','LineWidth',.5 ,'Marker','o','MarkerEdgeColor', 'k','MarkerSize', 12,'MarkerFaceColor',[0,0.45,0.75],'MarkerIndices',marker_space);
% plot(time,plot_constraint(1, :), 'Color' , [0.3,0.8,0.2] ,'LineWidth',5);
% ylim([0 0.75])
% xlim([0 SIM_TIME])
% legend('$\left\Vert \vec{p_d} \right\Vert$', '$\left\Vert \vec{p} \right\Vert$', '$\left\Vert r \right\Vert$', ...
%     'FontSize',48, 'interpreter', 'latex', ...
%     'Color', [1,1,1], 'EdgeColor', [1,1,1], ...
%     'Position', [0.863448876523099,0.490904758635022,0.094920428771982,0.452952387491862]);
% set(gca,"FontSize", 30);
% set(gca,"OuterPosition", [-0.05,0,1,1])
% xlabel('Time [s]','FontSize',40);
% ylabel('Position [m]','FontSize',40);
% grid off;
% box("off")
% print('./plots/restricted_control/Constraint', '-depsc2')
% hold off
% %
% figure;
% set(gcf,"Color",[1,1,1])
% set(gcf,"Position",[50,50,1920,1080])
% set(gcf,"Renderer","painters")
% plot(time,plot_trans_gains(1, :), 'k','LineWidth',5);
% hold on
% plot(time,plot_trans_gains(2, :), 'b','LineWidth',5);
% plot(time,plot_trans_gains(3, :), 'r','LineWidth',5);
% ylim([-140 20])
% xlim([0 25])
% legend('$kp_{x}$','$kp_{y}$','$kp_{z}$', ...
%     'FontSize',48, 'interpreter', 'latex', ...
%     'Color', [1,1,1], 'EdgeColor', [1,1,1], ...
%     'Position', [0.92,0.65,0.05,0.25]);
% set(gca,"FontSize", 30);
% xlabel('Time [s]','FontSize',40);
% ylabel('Translational kp','FontSize',40);
% grid off;
% box("off")
% print('./plots/restricted_control/tkp', '-depsc2')
% hold off

% figure
% set(gcf,"Color",[1,1,1])
% set(gcf,"Position",[80,220,1850,525])
% set(gcf,"Renderer","painters")
% plot(time,plot_trans_kd(1, :), 'k','LineWidth',5);
% hold on
% plot(time,plot_trans_kd(2, :), 'b','LineWidth',5);
% plot(time,plot_trans_kd(3, :), 'r','LineWidth',5);
% ylim([5.5 6.5])
% xlim([0 SIM_TIME])
% legend('$kd_{x}$','$kd_{y}$','$kd_{z}$', ...
%     'FontSize',48, 'interpreter', 'latex', ...
%     'Color', [1,1,1], 'EdgeColor', [1,1,1], ...
%     'Position', [0.92,0.65,0.05,0.25]);
% set(gca,"FontSize", 30);
% xlabel('Time [s]','FontSize',40);
% ylabel('Trans kd [m]','FontSize',40);
% grid off;
% box("off")
% print('./plots/restricted_control/tkd', '-depsc2')
% hold off

% figure
% set(gcf,"Color",[1,1,1])
% set(gcf,"Position",[80,220,1850,525])
% plot(time,plot_torques(1, :), 'k','LineWidth',5);
% hold on
% plot(time,plot_torques(2, :), 'b','LineWidth',5);
% plot(time,plot_torques(3, :), 'r','LineWidth',5);
% ylim([-0.05 0.05])
% xlim([0 SIM_TIME])
% legend('$\tau_x$','$\tau_y$','$\tau_z$', ...
%     'FontSize',48, 'interpreter', 'latex', ...
%     'Color', [1,1,1], 'EdgeColor', [1,1,1], ...
%     'Position', [0.92,0.65,0.05,0.25]);
% set(gca,"FontSize", 30);
% set(gca,"LineWidth", 1.5);
% xlabel('Time [s]','FontSize',40);
% ylabel('Torques [N/m]','FontSize',40);
% grid off;
% box("off")
% print('./plots/restricted_control/torques', '-depsc2')
% hold off

% figure
% set(gcf,"Color",[1,1,1])
% set(gcf,"Position",[80,220,1850,525])
% plot(time,plot_thrust(1, :), 'k','LineWidth',5);
% hold on
% ylim([0 1])
% xlim([0 SIM_TIME])
% legend('$F_{th}$', ...
%     'FontSize',48, 'interpreter', 'latex', ...
%     'Color', [1,1,1], 'EdgeColor', [1,1,1], ...
%     'Position', [0.92,0.65,0.05,0.25]);
% set(gca,"FontSize", 30);
% xlabel('Time [s]','FontSize',40);
% ylabel('Thrust [N]','FontSize',40);
% grid off;
% box("off")
% print('./plots/restricted_control/thrust', '-depsc2')
% hold off
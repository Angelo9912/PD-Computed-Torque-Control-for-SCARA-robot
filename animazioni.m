close all
clc
robot = importrobot('robot_description.urdf');
robot.DataFormat = 'column';
robot.Gravity = [0;0;-9.81];
% figure("Name","Plot SCARA 3D")
% set(gca, 'ZLim',[0 0.5])
% show(robot)
q1 = out.q1;
q2 = out.q2;
q3 = out.q3;
q4 = out.q4;
error_q1 = out.q1_error;
error_q2 = out.q2_error;
error_q3 = out.q3_error;
error_q4 = out.q4_error;
tau1 = out.tau1;
tau2 = out.tau2;
tau3 = out.tau3;
tau4 = out.tau4;
eta_plot = [];
x = out.x;
y = out.y;
z = out.z;
psi = out.psi;

l1 = 0.2;
l2 = 0.25;


% a = figure("Name","End effector trajectory")
% hold on
% 
% j3 = plot3(0,l1+l2,l1,"Marker","o","MarkerSize",6,"Color","black")
% j4 = plot3(0,l1+l2,l1/2,"Marker","+","MarkerSize",6,"Color","black")
% curve = animatedline('LineWidth',2)
figure("Name", "Animation Plot")
for i = 1:length(x)
    base = [0,0,0]';
    j1 = [0,0,l1]';
    R21 = [cos(q1(i)) -sin(q1(i)) 0; sin(q1(i)) cos(q1(i)) 0; 0 0 1];
    R32 = [cos(q2(i)+q1(i)) -sin(q2(i)+q1(i)) 0; sin(q2(i)+q1(i)) cos(q2(i)+q1(i)) 0; 0 0 1];
    j2 = R21 * [0 l1 0]' + j1;
    j3 = R32 * [0 l2 0]' + j2;
    j4 = [x(i) y(i) z(i)]';
    Re4 = [cos(q3(i)+q2(i)+q1(i)) -sin(q3(i)+q2(i)+q1(i)) 0; sin(q3(i)+q2(i)+q1(i)) cos(q3(i)+q2(i)+q1(i)) 0; 0 0 1];
    ee1 = Re4*[-0.025 0 0]' + j4;
    ee2 = Re4*[0.025 0 0]' + j4;
    ee3 = [0 0 -0.025]' + ee2;
    ee4 = [0 0 -0.025]' + ee1;
%     addpoints(curve,x(i),y(i),z(i));
    plot3([base(1),j1(1);j1(1),j2(1);j2(1),j3(1);j3(1),j4(1)], ...
        [base(2),j1(2); j1(2),j2(2);j2(2),j3(2);j3(2),j4(2)], ...
        [base(3),j1(3);j1(3),j2(3);j2(3),j3(3);j3(3),j4(3)], ...
        "Marker",".","MarkerSize",20,"LineWidth",2,"Color","Blue")
    hold on
    plot3([ee4(1),ee1(1),ee2(1),ee3(1)], [ee4(2),ee1(2),ee2(2),ee3(2)], ...
        [ee4(3),ee1(3),ee2(3),ee3(3)],"MarkerSize",20,"LineWidth",2,"Color","Blue")
    axis equal
    xlim([-0.5,+0.5])
    ylim([-0.5,+0.5])
    zlim([-0.03,+0.5])
    title("Robot Animation")
    xlabel("x [m]")
    ylabel("y [m]")
    zlabel("z [m]")
    hold off

    drawnow;
    pause(dt_animation)
end

%% Torques Plot

figure("Name","Joint Position Error Plot")

t = tiledlayout(2,2);
title(t,"Torques")
time = 0:dt_animation:times_way(end);

nexttile
plot(time,squeeze(tau1))
xlabel("Time [s]")
ylabel("tau 1 [N]")
title("tau 1")

nexttile
plot(time,squeeze(tau2))
xlabel("Time [s]")
ylabel("tau 2 [N]")
title("tau 2")

nexttile
plot(time,squeeze(tau3))
xlabel("Time [s]")
ylabel("tau 3 [N]")
title("tau 3")

nexttile
plot(time,squeeze(tau4))
xlabel("Time [s]")
ylabel("tau 4 [N]")
title("tau 4")

if controller == 1
    %% PD Error Plot
    error_x = out.error_x;
    error_y = out.error_y;
    error_z = out.error_z;
    error_psi = out.error_psi;
    figure("Name","Position Error Plot")
    
    t = tiledlayout(2,2);
    title(t,"Position Error")
    time = 0:dt_animation:times_way(end);
    
    nexttile
    plot(time,squeeze(error_x))
    xlabel("Time [s]")
    ylabel("x error [m]")
    title("x error")
    
    nexttile
    plot(time,squeeze(error_y))
    xlabel("Time [s]")
    ylabel("y error [m]")
    title("y error")
    
    nexttile
    plot(time,squeeze(error_z))
    xlabel("Time [s]")
    ylabel("z error [m]")
    title("z error")
    
    nexttile
    plot(time,squeeze(error_psi))
    xlabel("Time [s]")
    ylabel("psi error [degrees]")
    title("psi error")
else
    %% Joint Position Error Plot
    
    figure("Name","Joint Position Error Plot")
    
    t = tiledlayout(2,2);
    title(t,"Joint Position Error")
    time = 0:dt_animation:times_way(end);
    
    nexttile
    plot(time,squeeze(error_q1))
    xlabel("Time [s]")
    ylabel("q1 error [degrees]")
    title("q1 error")
    
    nexttile
    plot(time,squeeze(error_q2))
    xlabel("Time [s]")
    ylabel("q2 error [degrees]")
    title("q2 error")
    
    nexttile
    plot(time,squeeze(error_q3))
    xlabel("Time [s]")
    ylabel("q3 error [degrees]")
    title("q3 error")
    
    nexttile
    plot(time,squeeze(error_q4))
    xlabel("Time [s]")
    ylabel("q4 error [m]")
    title("q4 error")
    
    
    %% Joint Velocity Error Plot
    
    error_q1_dot = out.q1_error_dot;
    error_q2_dot = out.q2_error_dot;
    error_q3_dot = out.q3_error_dot;
    error_q4_dot = out.q4_error_dot;
    
    figure("Name","Joint Position Error Plot")
    
    t = tiledlayout(2,2);
    title(t,"Joint Velocity Error")
    time = 0:dt_animation:times_way(end);
    
    nexttile
    plot(time,squeeze(error_q1_dot))
    xlabel("Time [s]")
    ylabel("q_1 dot error [degrees/s]")
    title("q_1 dot error")
    
    nexttile
    plot(time,squeeze(error_q2_dot))
    xlabel("Time [s]")
    ylabel("q_2 dot error [degrees/s]")
    title("q_2 dot error")
    
    nexttile
    plot(time,squeeze(error_q3_dot))
    xlabel("Time [s]")
    ylabel("q_3 dot error [degrees/s]")
    title("q_3 dot error")
    
    nexttile
    plot(time,squeeze(error_q4_dot))
    xlabel("Time [s]")
    ylabel("q_4 dot error [m/s]")
    title("q_4 dot error")
end


clear;close;
%% 2v2 air combat
% 用于 2v2 air combat
% 假定红蓝双方各两架无人机，记为R1,R2;B1,B2
% 其初始状态记为R1_0,R2_0;B1_0,B2_0;整个过程状态量记为R1_s,R2_s;B1_s,B2_s
% 机动决策库为{左偏置；稳定飞行；最大加速；最大减速；右偏置；爬升；俯冲}

%% 参数设置
dt = 0.25;                        % 仿真步长
dT = 10 * dt;                      % 决策步长
sim_t = 120;                      % 总仿真时长
odeSolver = @ode45;               % 微分求解器

R1_0 = [0; 1000; 5000; 500; 0; -pi/2];
R2_0 = [0; 0; 5000; 500; 0; -pi/2];
B1_0 = [5000; 1000; 6000; 500; 0; -pi/2];
B2_0 = [4000; 0; 6000; 500; 0; -pi/2];        % 红蓝两方初始状态

global g nx_max nf_max sigma1 sigma2 R_max R_min V_min V_max h sigmah 
global sigmah1 sigmah2 omega_A omega_R omega_v omega_h

g = 9.81;                         % 重力系数
nx_max = 8 ;                      % 最大切向过载
nf_max = 3 ;                      % 最大法向过载

sigma1 = 100;                     % 距离优势函数参数 m
sigma2 = 1000;                    % 距离优势函数参数 m
R_max = 1000;                     % 最大攻击距离 m
R_min = 500;                      % 最小攻击距离 m
V_max = 800;                      % 最大速度 m/s
V_min = 300;                      % 最小速度 m/s
h = 200;                          % 最佳高度差 m
sigmah = 100;                     % 高度优势函数参数 m
sigmah1 = 100;                    % 高度优势函数参数 m
sigmah2 = 1000;                   % 高度优势函数参数 m
omega_A = 1;                   % 角度优势函数权重
omega_R = 1;                   % 距离优势函数权重
omega_v = 1;                   % 速度优势函数权重
omega_h = 1;                   % 高度优势函数权重

%%

total_k = ceil(sim_t / dt);     % 总步数
t = 0;
R1 = R1_0;
R2 = R2_0;
B1 = B1_0;
B2 = B2_0;

% 设置变量预存
AR11_s = zeros(total_k,1);
AR12_s = zeros(total_k,1);
AR21_s = zeros(total_k,1);
AR22_s = zeros(total_k,1);
AB11_s = zeros(total_k,1);
AB12_s = zeros(total_k,1);
AB21_s = zeros(total_k,1);
AB22_s = zeros(total_k,1);

R1_s = zeros(total_k,6);
R2_s = zeros(total_k,6);
B1_s = zeros(total_k,6);
B2_s = zeros(total_k,6);
R1_T = zeros(total_k,1);
R2_T = zeros(total_k,1);
B1_T = zeros(total_k,1);
B2_T = zeros(total_k,1);


R1_u_s = zeros(total_k,3);
R2_u_s = zeros(total_k,3);
B1_u_s = zeros(total_k,3);
B2_u_s = zeros(total_k,3);

t_s = zeros(total_k,1);

% 设置初值
t_s(1) = t;
AR11_s(1) = advantage(R1,B1);
AR12_s(1) = advantage(R1,B2);
AR21_s(1) = advantage(R2,B1);
AR22_s(1) = advantage(R2,B2);
AB11_s(1) = advantage(B1,R1);
AB12_s(1) = advantage(B1,R2);
AB21_s(1) = advantage(B2,R1);
AB22_s(1) = advantage(B2,R2);

R1_s(1,:) = R1';
R2_s(1,:) = R2';
B1_s(1,:) = B1';
B2_s(1,:) = B2';

R1_T(1) = 2;
R2_T(1) = 2;
B1_T(1) = 2;
B2_T(1) = 2;

% B1_u = [0 1 0];
% B2_u = [0 1 0];               % 假设蓝方为匀速直线

for k = 1:total_k-1
    t;
    
    [R1_t,tr1] = target(R1,B1,B2);
    R1_T(k+1) = tr1;
    R1_u = controller(R1,R1_t,dT);
    R1_u_s(k, :) = R1_u;
    
    [R2_t,tr2] = target(R2,B1,B2);
    R2_T(k+1) = tr2;
    R2_u = controller(R2,R2_t,dT);
    R2_u_s(k, :) = R2_u;
    
    [B1_t,tb1] = target(B1,R1,R2);
    B1_T(k+1) = tb1;
    B1_u = controller(B1,B1_t,dT);
    B1_u_s(k, :) = B1_u;
    
    [B2_t,tb2] = target(B2,R1,R2);
    B2_T(k+1) = tb2;
    B2_u = controller(B2,B2_t,dT);
    B2_u_s(k, :) = B2_u;  

%     B1_t = target(B1,R1,R2);
%     B1_u = controllerD(B1,B1_t,dT);
%     B1_u_s(k, :) = B1_u;
%     
%     B2_t = target(B2,R1,R2);
%     B2_u = controllerD(B2,B2_t,dT);
%     B2_u_s(k, :) = B2_u;                     % 蓝方采用目标优势函数最小原则
    
    [~, R1_s_temp] = odeSolver(@(t,y) dynamics(t,y, R1_u), [t t+dt], R1);
    R1 = R1_s_temp(end, :)';
    [~, R2_s_temp] = odeSolver(@(t,y) dynamics(t,y, R2_u), [t t+dt], R2);
    R2 = R2_s_temp(end, :)';
    [~, B1_s_temp] = odeSolver(@(t,y) dynamics(t,y, B1_u), [t t+dt], B1);
    B1 = B1_s_temp(end, :)';  
    [t_s_temp, B2_s_temp] = odeSolver(@(t,y) dynamics(t,y, B2_u), [t t+dt], B2);
    B2 = B2_s_temp(end, :)';
    
    AR11_s(k+1) = advantage(R1,B1);
    AR12_s(k+1) = advantage(R1,B2);
    AR21_s(k+1) = advantage(R2,B1);
    AR22_s(k+1) = advantage(R2,B2);
    AB11_s(k+1) = advantage(B1,R1);
    AB12_s(k+1) = advantage(B1,R2);
    AB21_s(k+1) = advantage(B2,R1);
    AB22_s(k+1) = advantage(B2,R2);
    
    R1_s(k+1, :) = R1';
    R2_s(k+1, :) = R2';
    B1_s(k+1, :) = B1';
    B2_s(k+1, :) = B2';
    t_s(k+1) = t_s_temp(end);
    
    t = t + dt;
    
end

%% plot

figure(1)

plot3(R1_s(:,1),R1_s(:,2),R1_s(:,3));
hold on 
plot3(R2_s(:,1),R2_s(:,2),R2_s(:,3));
plot3(B1_s(:,1),B1_s(:,2),B1_s(:,3));
plot3(B2_s(:,1),B2_s(:,2),B2_s(:,3));
% comet3(R1_s(:,1),R1_s(:,2),R1_s(:,3));
% hold on
% comet3(R2_s(:,1),R2_s(:,2),R2_s(:,3));
% comet3(B1_s(:,1),B1_s(:,2),B1_s(:,3));
% comet3(B2_s(:,1),B2_s(:,2),B2_s(:,3));
grid on 
xlabel('x轴/m')
ylabel('y轴/m')
zlabel('z轴/m')
legend('R_1轨迹','R_2轨迹','B_1轨迹','B_2轨迹')
%axis([0,10000,0,10000,0,30000])

figure(2)

plot(t_s,AR11_s);
hold on 
plot(t_s,AR12_s);
plot(t_s,AR21_s);
plot(t_s,AR22_s);
grid on
xlabel('时间/s')
ylabel('优势函数值')
legend('R_1对B_1','R_1对B_2','R_2对B_1','R_2对B_2')

figure(3)

plot(t_s,AB11_s);
hold on 
plot(t_s,AB12_s);
plot(t_s,AB21_s);
plot(t_s,AB22_s);
grid on
xlabel('时间/s')
ylabel('优势函数值')
legend('B_1对R_1','B_1对R_2','B_2对R_1','B_2对R_2')

figure(4)

subplot(2,2,1)
plot(t_s,R1_T)
xlabel('时间/s')
ylabel('目标编号')
title('R_1目标')
yticks([1 2])
yticklabels({'B_1','B_2'})

subplot(2,2,2)
plot(t_s,R2_T)
xlabel('时间/s')
ylabel('目标编号')
title('R_2目标')
yticks([1 2])
yticklabels({'B_1','B_2'})

subplot(2,2,3)
plot(t_s,B1_T)
xlabel('时间/s')
ylabel('目标编号')
title('B_1目标')
yticks([1 2])
yticklabels({'R_1','R_2'})

subplot(2,2,4)
plot(t_s,B2_T)
xlabel('时间/s')
ylabel('目标编号')
title('B_2目标')
yticks([1 2])
yticklabels({'R_1','R_2'})


clear;
% 系统参数
M = 0.5;   % 车身质量 (kg)
m = 0.145;   % 每个车轮的质量 (kg)
J_w = 0.000081;  % 每个车轮的转动惯性 (kg·m^2)
R = 0.0425;    % 车轮半径 (m)
J = 0.0019;   % 车身转动惯性 (kg·m^2)
l = 0.05;   % 车身质心到车轮轴的距离 (m)
g = 9.81;    % 重力加速度 (m/s^2)
J_y = 0.0081;
d = 0.25;  %轮距
temp1 = 2*m+M+2*J_w/R^2;
temp2 = J+M*l^2;
% 状态空间矩阵 x,Vx,pitch,Wy,yaw,Wz
A = [0 1 0 0 0 0;
     0 0 (M^2*l^2*g)/(temp1*temp2) 0 0 0;
     0 0 0 1 0 0;
     0 0 l*M*g/temp2 0 0 0
     0 0 0 0 0 1
     0 0 0 0 0 0];

B = [0 0;
     1/temp1*(M*l/temp2+1/R) 1/temp1*(M*l/temp2+1/R);
     0 0;
     1/temp2 1/temp2
     0 0
     -d/(R*J_y) d/(R*J_y)];
 
C=[0,1,0,0,0,0;
   0,0,1,0,0,0;
   0,0,0,0,0,1];

D=zeros(3,2);

% 系统状态空间模型
sys = ss(A, B, C, D);
sysd=c2d(sys,0.01,'zoh');

Ad=sysd.A;
Bd=sysd.B;
Cd=sysd.C;
Dd=sysd.D;
% 输出系统的状态空间矩阵
disp('状态空间模型的矩阵A:');
disp(A);
disp('状态空间模型的矩阵B:');
disp(B);

% 选择一个初始状态
initial_conditions = [0; 0; 0.1; 0;0;0];  % 初始状态 [位置, 速度, 倾角, 角速度]
t = 0:0.01:10;  % 时间向量

% 使用 LQR 控制器生成控制输入
Q = diag([1, 10, 10, 1,1,10]);  % 状态的权重矩阵
R = [1 0;0 1];  % 控制输入的权重
[K, S, e] = lqr(A, B, Q, R);
[Kd, Sd, ed] = dlqr(Ad, Bd, Q, R);
% 使用 LQR 控制器计算控制输入
u = @(x) -K * x;

% 仿真系统响应
[t, x] = ode45(@(t, x) (A - B*K) * x, t, initial_conditions);


% 绘制结果
figure;
subplot(2,1,1);
plot(t, x(:,1), 'r', 'LineWidth', 2);
title('小车位置 x(t)');
xlabel('时间 (秒)');
ylabel('位置 (米)');

subplot(2,1,2);
plot(t, x(:,3), 'b', 'LineWidth', 2);
title('车身倾角 θ(t)');
xlabel('时间 (秒)');
ylabel('倾角 (弧度)');



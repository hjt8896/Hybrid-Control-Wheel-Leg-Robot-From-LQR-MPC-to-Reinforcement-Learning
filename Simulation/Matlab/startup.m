clear;
% 系统参数
global Psi Theta Np Nc Cd pitch_0;
pitch_0=45;
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
   0,0,0,1,0,0
   0,0,0,0,0,1];%y:speed pitch wy wz

D=zeros(4,2);

% 系统状态空间模型
sys = ss(A, B, C, D);
sysd=c2d(sys,0.01,'zoh');

Ad=sysd.A;
Bd=sysd.B;
Cd=sysd.C;
Dd=sysd.D;

%mpc控制器
Np=20;
Nc=20;
% 构建Psi和Theta矩阵 Y=Psi*x_k + Theta*U
Psi = zeros(4*Np, size(Ad,1)); % 4输出(pitch)
Theta = zeros(4*Np, 2*Nc); % 2输入(τ)

for i = 1:Np
    Psi(4*i-3:4*i, :) = Cd*Ad^i;
    for j = 1:min(i,Nc)
        Theta(4*i-3:4*i, 2*j-1:2*j) = Cd*Ad^(i-j)*Bd;
    end
end


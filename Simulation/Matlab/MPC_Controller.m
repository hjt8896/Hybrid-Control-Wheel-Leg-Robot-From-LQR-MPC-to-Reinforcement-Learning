function T = MPC_Controller(states)
global Psi Theta Np Nc Cd;
tic;
% states:
T=zeros(2,1);%Tl Tr
X_1=zeros(6,1);
X_1 = states; % x dx q pitch wy wz
Y_1 = Cd*X_1;
% desired commands:
Y_des = [0.3;-0.18;0;0]; 
% desired reference:
Y_ref = generateReference(Y_1,Y_des,Np);

Q_speed = 10000;    % speed权重
Q_pitch = 3000;         % pitch重
Q_wy = 10;       % wy权重
Q_wz = 50;       % wz权重
Q = kron(eye(Np), diag([Q_speed, Q_pitch, Q_wy,Q_wz]));
R = 0.001;     % 扭矩权重

u_min = -2; u_max = 2;
% 控制约束
M_u = [eye(2*Nc); -eye(2*Nc)];
gamma_u = [u_max*ones(2*Nc,1); -u_min*ones(2*Nc,1)];
T=solve_mpc(X_1, Psi, Theta, Q, R, M_u, gamma_u, Y_ref);
end

function u = solve_mpc(x_k, Psi, theta, Q, R, M, gamma, y_ref)
    u=zeros(2,1);
    % 构建目标函数 
    H = theta'*Q*theta + R;
    f = (Psi*x_k - y_ref)'*Q*theta;
    
    % 求解QP问题
    options = optimoptions('quadprog', 'Display', 'none');
    U = quadprog(H, f', M, gamma, [], [], [], [], [], options);
    
    % 返回第一个控制量
    u(1) = U(1); 
    u(2) = U(2);
end

function x_ref = generateReference(x,x_des,h)
    x_ref=zeros(4*h,1);
    temp = zeros(4, h);
    for i = 1 : 4
        temp(i,:) = linspace(x(i), x_des(i), h);
    end
    for i = 1 : h
        x_ref(4*i-3:4*i,:) = temp(:,i);
    end
end
clc
clear all
close all
%% disturbance observer based event trigger MPC (PID-DEMPC)
% 基于扰动观测器和 PID 型事件触发的模型预测控制
%
% 系统模型 (已参数化):
%   x1_dot = x2
%   x2_dot = -(tao/M)*exp(-x1)*x1 - (hd/M)*x2 + (1/M)*u + (1/M)*w
%
% 其中: tao, M, hd 为物理参数； w 为外部扰动
% 控制目标: 在存在扰动时稳定系统到原点，同时减少 MPC 求解次数

%% ========== 参数设置 ==========
tao = 0.9;       % 非线性刚度系数
M   = 1.25;      % 质量/惯性
hd  = 0.42;      % 阻尼系数
k0  = 0.33;      % (未使用，可能是原始参数)
Ts  = 0.1;       % 采样时间 / 离散周期 (s)
Tp  = 10;        % 仿真总时间 (s)
N   = Tp/Ts;     % 总仿真步数 (100步)

% 预生成外部扰动序列 (论文: w(t) = 0.1*sin(1.4*t))
for i = 1:1:N
    w(i) = 0.1*sin(i*Ts*1.4);
end

%% 系统状态空间矩阵 (用于观测器和补偿)
B = [0; 1/M];          % 输入矩阵
D = [0; 1/M];          % 扰动输入矩阵 (与 B 相同，表示匹配扰动)

%% 扩张状态观测器 (ESO) 设计 (替代论文中的非线性 DOB)
% 原论文 DOB 设计为: d(x_hat)/dt = f + g u + h w_hat + L (x - x_hat)
%                  w_hat_dot = Gamma h^T (x - x_hat)
% 本程序采用线性 ESO 估计状态 x2 和总扰动 d:
L     = [0, 0.5];      % 观测器增益 (未直接在 ESO 中使用，ESO 使用带宽参数)
w0    = 12;            % ESO 带宽
beta1 = 2*w0;          % ESO 增益 1
beta2 = w0^2;          % ESO 增益 2

%% MPC 控制器参数
P = [0.1692, 0.0572; 0.0572, 0.1391];  % 终端代价矩阵 (Lyapunov 矩阵)
K = [-0.4454, -1.0932];                 % 终端区域内的线性反馈增益
Q = 0.2*[1,0; 0,1];                    % 状态阶段代价权重
R = 0.1;                               % 控制输入阶段代价权重
r = 0.1508;                            % 终端不变集水平 (x'*P*x <= r)
x0 = [1.2; -2];                        % 初始状态
T = 2.5/Ts;                            % 预测时域长度 (25 步)

%% PID 型事件触发参数
Kp = 0.14;     % 比例增益 (在触发条件中)
Ki = 0.9;      % 积分增益
Kd = 0.7;      % 微分增益

%% 初始化状态/控制/观测变量
xc   = zeros(2, N);   % 实际系统状态 (2 x N)
uc   = zeros(1, N);   % 实际控制输入 (1 x N)
xb   = zeros(2, N);   % MPC 预测状态序列 (用于触发比较)
z    = zeros(1, N);   % 原 DOB 辅助变量 (未使用)
xhat = zeros(1, N);   % ESO 对 x2 的估计 (1 x N)
deso = zeros(1, N);   % ESO 估计的总扰动 d_hat (1 x N)
udo  = zeros(1, N);   % 扰动补偿控制 u_dob = -d_hat (1 x N)
JiF  = 0;             % 触发条件中的积分项累加值

% 设置初始值
xhat(1) = x0(2,1);    % x2 初始估计
xc(:,1) = x0;         % 实际状态初值
xb(:,1) = x0;         % 预测状态初值

%% 事件触发相关变量
tk = 1;               % 当前使用的预计算控制序列索引 (1~T)
et = 1;               % 触发时刻记录向量 (第一个元素为1)
tt = 0;               % (未使用)

%% ========== 主仿真循环 ==========
for i = 1 : N-1
    % 检查当前状态是否在终端不变集外 (需要激活 MPC)
    if xc(:,i)' * P * xc(:,i) >= r   % 条件: x'*P*x >= 0.001508
        
        % --- 分支 1: 初始时刻或重新触发 MPC ---
        if i == 1  % 第一步必须求解 MPC
            yalmip('clear')
            % 定义优化变量: x (2xT), u (1xT)
            x = sdpvar(2, T);
            u = sdpvar(1, T);
            x(:,1) = x0;  % 初始状态约束
            
            % 控制输入约束
            const = [u <= 1, u >= -1];
            
            % 系统动力学等式约束 (前向欧拉离散)
            for k = 1 : T-1
                const = [const, x(1,k+1) == x(1,k) + Ts*(x(2, k))];
                const = [const, x(2,k+1) == x(2,k) + Ts*(- (tao/M)*(exp(-x(1,k)))*x(1,k) - hd/M*x(2,k) + u(k)/M)];
            end
            
            % 目标函数: 阶段代价 + 终端代价
            obj = 0;
            for j = 1 : T-1
                obj = obj + (x(:, j))' * Q * (x(:, j)) + u(j)' * R * u(j);
            end
            obj = obj + x(:, T)' * P * x(:, T);
            
            % 求解优化问题
            ops = sdpsettings('verbose', 0);
            optimize(const, obj, ops);
            
            % 保存最优解
            f(i) = double(obj);        % 最优代价 (调试用)
            xb   = value(x);           % 最优状态轨迹 (2xT)
            ucb  = value(u);           % 最优控制序列 (1xT)
            uc(i) = ucb(1) - udo(i);   % 实际控制 = MPC输出 - 扰动补偿
            
            % 状态更新 (欧拉积分，包含扰动 w(i) 和 DOB 补偿已在 ucb 中体现? 注意: ucb 未包含补偿，这里减去 udo)
            xc(1, i+1) = xc(1,i) + Ts*xc(2, i);
            xc(2, i+1) = xc(2,i) + Ts*(- (tao/M)*(exp(-xc(1,i)))*xc(1,i) - hd/M*xc(2,i) + (ucb(1) - udo(i))/M + w(i)/M);
            
        else
            % --- 分支 2: 非初始时刻，检查 PID 触发条件 ---
            % 触发条件: 基于李雅普诺夫函数误差 + PID 项
            % J = exp(Kp)*e'*P*e + exp(Ki)*∫e'*P*e dt + exp(Kd)*Δ(e'*P*e) < 0.01
            % 这里用 (xc - xb(:,tk+1)) 作为预测误差 e
            % 积分项 JiF 已在上一步累加
            % 微分项近似为 (e'*P*e) / (tk+1)
            if exp(Kp)*(xc(:,i)-xb(:,tk+1))'*P*(xc(:,i)-xb(:,tk+1)) + exp(Ki)*JiF + exp(Kd)*(xc(:,i)-xb(:,tk+1))'*P*(xc(:,i)-xb(:,tk+1))/(tk+1) < 0.01 && tk+1 < T
                % 未触发：继续使用预计算的控制序列
                JiF = JiF + (xc(:,i)-xb(:,tk+1))'*P*(xc(:,i)-xb(:,tk+1))*Ts;  % 积分累加
                tk = tk + 1;  % 序列索引前移
                
                % 应用预计算序列的下一个控制量
                uc(i) = ucb(tk+1) - udo(i);
                
                % 状态更新
                xc(1, i+1) = xc(1,i) + Ts*xc(2, i);
                xc(2, i+1) = xc(2,i) + Ts*(- (tao/M)*(exp(-xc(1,i)))*xc(1,i) - hd/M*xc(2,i) + (ucb(tk+1) - udo(i))/M + w(i)/M);
                
                x0 = xc(:, i+1);  % 更新初始状态用于下次可能的重优化
            else 
                % --- 触发重新求解 MPC ---
                et = [et, i];     % 记录触发时刻索引
                tk = 1;          % 重置序列索引
                JiF = 0;         % 清空积分项
                
                yalmip('clear')
                % 重新定义 MPC 优化问题 (同初始时刻)
                x = sdpvar(2, T);
                u = sdpvar(1, T);
                x(:,1) = x0;      % 以当前实际状态为初始状态
                
                const = [u <= 1, u >= -1];
                for k = 1 : T-1
                    const = [const, x(1,k+1) == x(1,k) + Ts*(x(2, k))];
                    const = [const, x(2,k+1) == x(2,k) + Ts*(- (tao/M)*(exp(-x(1,k)))*x(1,k) - hd/M*x(2,k) + u(k)/M)];
                end
                obj = 0;
                for j = 1 : T-1
                    obj = obj + (x(:, j))' * Q * (x(:, j)) + u(j)' * R * u(j);
                end
                obj = obj + x(:, T)' * P * x(:, T);
                
                ops = sdpsettings('verbose', 0);
                optimize(const, obj, ops);
                
                xb  = value(x);
                ucb = value(u);
                
                % 使用新序列的第一个元素
                uc(i) = ucb(1) - udo(i);
                
                % 状态更新
                xc(1, i+1) = xc(1,i) + Ts*xc(2, i);
                xc(2, i+1) = xc(2,i) + Ts*(- (tao/M)*(exp(-xc(1,i)))*xc(1,i) - hd/M*xc(2,i) + (ucb(1) - udo(i))/M + w(i)/M);
            end
        end
        
    else
        % --- 分支 3: 状态已进入终端不变集 ---
        % 切换为线性状态反馈控制 u = K*x
        uc(i) = K * xc(:,i) - udo(i);   % 线性反馈 - 扰动补偿
        
        % 状态更新
        xc(1, i+1) = xc(1,i) + Ts*xc(2, i);
        xc(2, i+1) = xc(2,i) + Ts*(- (tao/M)*(exp(-xc(1,i)))*xc(1,i) - hd/M*xc(2,i) + uc(i)/M + w(i)/M);
        
        x0 = xc(:, i+1);  % 更新状态记录
    end
    
    % ===== 扩张状态观测器 (ESO) 更新 =====
    % 估计 x2 和总扰动 d
    % xhat: 对 x2 的估计
    % deso: 对总扰动 d 的估计 (包含外部扰动和模型不确定性)
    xhat(i+1) = xhat(i) + Ts * (- (tao/M)*(exp(-xc(1,i)))*xc(1,i) - hd/M*xc(2,i) + uc(i)/M + deso(i)/M + beta1*(xc(2,i) - xhat(i)));
    deso(i+1) = deso(i) + Ts * (beta2 * (xc(2,i) - xhat(i)));
    udo(i+1)  = deso(i+1);   % 扰动补偿量 = 估计的扰动值
    
    % 以下为另一种 DOB 实现 (已注释)
    % z(i+1) = z(i) + Ts * (-L * ([0,1; -(tao/M)*exp(-xc(1,i)), -hd/M*xc(2,i)] * xc(:,i) + [0;1/M]*uc(i) + D*(z(i) + L*xc(:,i))));
    % dhat(i+1) = z(i+1) + L*xc(:,i+1);
    % udo(i+1) = -inv(B'*B)*B'*D*dhat(i+1);
end

%% ========== 结果可视化 ==========
dt   = 0.1;
t    = 0 : dt : (N-1)*dt;
t_com = 0 : dt : (N-1)*dt;

% 图1: 状态响应 x1(黑色), x2(红色)
figure(1)
plot(t, xc(1,:), 'k', t, xc(2,:), 'r')
xlabel('Time (s)'); ylabel('States'); legend('x_1', 'x_2');
title('PID-DEMPC 状态轨迹')

% 图2: 控制输入 uc
figure(2)
plot(t, uc, 'k')
xlabel('Time (s)'); ylabel('Control input u');
title('控制输入')

% 图3: 事件触发时刻 (用竖线标记)
figure(3)
xline(et);
xlabel('Simulation step'); ylabel('Trigger events');
title('事件触发时刻')

% 图4: 扰动真实值 w (红色) 和 ESO 估计值 deso (蓝色)
figure(4)
plot(t, w, 'r', t, deso, 'b')
xlabel('Time (s)'); ylabel('Disturbance');
legend('True w', 'Estimated d');
title('扰动估计效果')
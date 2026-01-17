clear;
clc;
syms phi phi_d phi_dd x x_d x_dd theta theta_d theta_dd;
syms N P Nm Pm;
syms R L l lp lm; % 一些长度常量
syms T Tp;
syms alpha beta gama;
syms Iw Ip Im mw mp M g;

syms phi_d2 theta_d2 sin_phi_alpha cos_phi_alpha sin_theta cos_theta sin_phi_gama cos_phi_gama; % 方便用

phi_d2 = phi_d ^ 2;
theta_d2 = theta_d ^ 2;
sin_phi_alpha = sin(phi - alpha);
cos_phi_alpha = cos(phi - alpha);
sin_theta = sin(theta);
cos_theta = cos(theta);
% sin_phi_gama = sin(phi - gama);
% cos_phi_gama = cos(phi - gama);
sin_phi_gama = sin(phi + gama);
cos_phi_gama = cos(phi + gama);

const_x_matrix = {Iw, Im, mw, mp, M, g, R, lm};
const_matrix = {0.000074, 0.00921, 0.200, 0.85, 2.145, 9.8, 0.050, 0.00939}; % 对应 Iw Im mw mp M g R lm共八个

x_matrix = {phi, phi_d, x, x_d, theta, theta_d, T, Tp};
solution_matrix = {0, 0, 0, 0, 0, 0, 0, 0}; % 刚开始用

MatQ = [10 0 0 0 0 0; 0 1 0 0 0 0; 0 0 5 0 0 0; 0 0 0 10 0 0; 0 0 0 0 1000 0; 0 0 0 0 0 1];
MatR = [20 0; 0 0.25]; %权重矩阵 R 的设计

% 表达式
% Nm = -M * (x_dd - l * (sin_phi_alpha * phi_d2 - cos_phi_alpha * phi_dd) + lm * (- sin_theta * theta_d2 + cos_theta * theta_dd));
Nm = -M * (x_dd - L * (-sin(phi) * phi_d2 + cos(phi) * phi_dd) + lm * (- sin_theta * theta_d2 + cos_theta * theta_dd));
% Pm = M * g + M * (l * (-cos_phi_alpha * phi_d2 - sin_phi_alpha * phi_dd) + lm * (-cos_theta * theta_d2 - sin_theta * theta_dd));
Pm = M * g + M * (L * (-cos(phi) * phi_d2 - sin(phi) * phi_dd) + lm * (-cos_theta * theta_d2 - sin_theta * theta_dd));

eq_N = Nm - N == mp * (x_dd - l * (sin_phi_alpha * phi_d2 - cos_phi_alpha * phi_dd));
eq_P = P - Pm == mp * g + mp * l * (-cos_phi_alpha * phi_d2 - sin_phi_alpha * phi_dd);
N = solve(eq_N, N);
P = solve(eq_P, P); % 得到N和P的表达式

% eq_phi_dd = Ip * phi_dd == Tp - T + l * cos_phi_alpha * N + l * sin_phi_alpha * P - lp * sin_phi_gama * Pm + lp * cos_phi_gama * Nm;
eq_phi_dd = Ip * phi_dd == Tp - T + l * cos_phi_alpha * N + l * sin_phi_alpha * P + lp * sin_phi_gama * Pm + lp * cos_phi_gama * Nm;
eq_x_dd = x_dd == (N * R + T) / (Iw / R + mw * R);
eq_theta_dd = Im * theta_dd == -Tp + Pm * lm * sin_theta + Nm * lm * cos_theta;

phi_x_theta = solve([eq_phi_dd,eq_x_dd,eq_theta_dd],[phi_dd,x_dd,theta_dd]);

f1 = phi_x_theta.phi_dd;
f2 = phi_x_theta.x_dd;
f3 = phi_x_theta.theta_dd;

%%考虑不同摆角时
data_set = readmatrix("data_set.csv");
data_num = length(data_set(:,1));
phi_0_matrix = data_set(:, 7);

solution = zeros(data_num, 8);
solution(:,1) = phi_0_matrix;

L_const_x_matrix = {L, l, lp, alpha, gama, Ip};

for i = 1 : 2
    for j = 1 : 6
        folder = 'K_matrix_data';
        filename = sprintf('k%d%d.csv', i, j);
        delete(fullfile(folder, filename));
    end
end

for i = 1 : data_num
    L_const_matrix = [data_set(i, 1:6)];

    A1 = subs(diff(f1, phi), x_matrix, solution(i, :));
    A2 = subs(diff(f1, theta), x_matrix, solution(i, :));
    A3 = subs(diff(f2, phi), x_matrix, solution(i, :));
    A4 = subs(diff(f2, theta), x_matrix, solution(i, :));
    A5 = subs(diff(f3, phi), x_matrix, solution(i, :));
    A6 = subs(diff(f3, theta), x_matrix, solution(i, :));
    
    B1 = subs(diff(f1, T), x_matrix, solution(i, :));
    B2 = subs(diff(f1, Tp), x_matrix, solution(i, :));
    B3 = subs(diff(f2, T), x_matrix, solution(i, :));
    B4 = subs(diff(f2, Tp), x_matrix, solution(i, :));
    B5 = subs(diff(f3, T), x_matrix, solution(i, :));
    B6 = subs(diff(f3, Tp), x_matrix, solution(i, :));

    A = [0 1 0 0 0 0; A1 0 0 0 A2 0; 0 0 0 1 0 0; A3 0 0 0 A4 0; 0 0 0 0 0 1; A5 0 0 0 A6 0];
    B = [0 0; B1 B2; 0 0; B3 B4; 0 0; B5 B6];

    A = subs(A, const_x_matrix, const_matrix);
    A = subs(A, L_const_x_matrix, L_const_matrix);

    B = subs(B, const_x_matrix, const_matrix);
    B = subs(B, L_const_x_matrix, L_const_matrix);

    A_double = double(A);
    B_double = double(B);

    % %%判断可控性
    % [v,d]=eig(A);
    % D=[B,A*B,A*A*B,A*A*A*B,A*A*A*A*B,A*A*A*A*A*B];
    % r1=rank(D)
    % %%判断观测性
    % C = eye(6,6);
    % D = eye(6,2);
    % E=[C;C*A;C*A*A;C*A*A*A;C*A*A*A*A;C*A*A*A*A*A];
    % r2=rank(E)

    K = lqr(A_double,B_double,MatQ,MatR); %调用 lqr 函数用以求解状态反馈矩阵K(需要具体数值)
    K = -K;
    %%添加到表格中
    for row = 1 : 2
        for column = 1 : 6
            folder = 'K_matrix_data';
            filename = sprintf('k%d%d.csv', row, column);
            writematrix(K(row, column), fullfile(folder, filename), 'WriteMode', 'append');
        end
    end
end

fprintf("K矩阵已保存\n")

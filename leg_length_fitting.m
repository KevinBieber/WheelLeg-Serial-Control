clear
clc

data_set = readmatrix("data_set.csv");

leg_length_set = data_set(:,1);
phi_set = data_set(:,7);
p = polyfit(leg_length_set,phi_set,3);

phi_pre = polyval(p, leg_length_set); % 拟合后的值
R = corrcoef(phi_set, phi_pre); % 计算拟合相关系数
% fprintf("拟合的相关系数为：%.3f\n",R(1,2));

fprintf("phi_target = %.4ff * L_3 + %.4ff * L_2 + %.4ff * L_1 + %.4ff;\n", p(1), p(2), p(3), p(4));
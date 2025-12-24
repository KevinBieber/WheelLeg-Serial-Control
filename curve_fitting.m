clear
clc

delete('curve_p.csv');
data_set = readmatrix("data_set.csv");
leg_length = data_set(:, 1);
folder = 'K_matrix_data';

figure;
t = tiledlayout(2,6);
for i = 1:2
    for j = 1:6
        filename = sprintf('K%d%d.csv',i,j);
        Kij_vec = readmatrix(fullfile(folder, filename));
        
        p = polyfit(leg_length, Kij_vec, 3);
        Kij_pre = polyval(p, leg_length); % 20个样本对应的拟合后的值
        R = corrcoef(Kij_vec, Kij_pre); % 计算拟合相关系数
        fprintf("K%d%d拟合的相关系数为：%.3f\n",i,j,R(1,2));
        writematrix(p, 'curve_p.csv', 'WriteMode', 'append');
        
        %% 画图
        % 数据准备
        x_fit = linspace(min(leg_length), max(leg_length), 100); % 拟合曲线用的连续点
        y_fit = polyval(p, x_fit);

        % 一张张画
        % figure;
        % plot(leg_length, Kij_vec, 'o', 'MarkerFaceColor','b'); % 样本点
        % hold on;
        % plot(x_fit, y_fit, '-r', 'LineWidth',2); % 拟合曲线
        % hold off;
        % xlabel('x');
        % ylabel('y');
        % title('样本点与拟合曲线');
        % legend('样本点','拟合曲线','Location','best');
        % grid on;

        % 画在一起
        ax = nexttile;
        plot(ax, leg_length, Kij_vec,'-r');        % 画第一条曲线
        hold(ax,'on');                 % 保持在 ax 上
        plot(ax, x_fit, y_fit, '--b');       % 在同一个子图继续画第二条曲线
        hold(ax,'off');
        legend('OriginData','FitData');
        grid on;
    end
end
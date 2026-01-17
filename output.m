clear
clc

p_matrix = readmatrix("curve_p.csv");

for i = 1:12
    p = p_matrix(i,:);
    fprintf("K_data[%d] = %.4ff * L_3 + %.4ff * L_2 + %.4ff * L_1 + %.4ff;\n", i - 1, p(1), p(2), p(3), p(4));
    
end
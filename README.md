# myLQR
author:江sir
## 文件结构描述
```
project-name/
│
├── K_matrix_data/ # 这个文件夹放的是不同腿长下K矩阵每个元素的具体值
│   ├── k11.csv
│   ├── k12.csv
|   └── ...
│
├── curve_p.csv # 这个表格存的是K矩阵12个元素关于腿长的3阶多项式拟合的系数（降幂排列）
|
├── data_set.csv # 不同腿长下各参数的值
│
├── curve_fitting.m # 做拟合的脚本
│
├── LQR_function.m # 算K矩阵的脚本
│
├── mytry.m # 试验控制效果的脚本
│
├── output.m # 输出固定格式代码的脚本
│
└── README.md
```
## 注意事项
在运行curve_fitting.m做拟合获得多项式系数之前需要先运行LQR_function.m计算不同腿长下K矩阵
# FRP 端口约定

服务器: 118.190.106.114
控制口: 7000（token 见 nuc_ros/frp/frpc.toml）

数据方向：

  视频:  NUC → 服务器:6004 → PC
  状态:  NUC → 服务器:6005 → PC
  指令:  PC  → 服务器:6006 → NUC

NUC 本机: 8081 / 8082 / 8083（由 NUC 上的 frpc 映射出去）
服务器对外（PC 只连这些）: 6004 / 6005 / 6006

Windows default.yaml: host=118.190.106.114, ports=6004/6005/6006
PC: 只跑上位机（start.bat），不跑 frpc
Ubuntu NUC: 跑 nuc_ros/frp/frpc.toml（注册三口）
云主机: 跑 frps，放行 7000 与 6004-6006

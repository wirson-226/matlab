%%% 整体逻辑 %%%
% 路径规划--参考轨迹--拿出想要的跟踪状态--比如该案例：时刻相关的位置与偏航角--利用视线导引或者其他方法（后续可改）计算期望状态--速度，加速度，角度，角速度，角加速度
% 其中，一般六自由度划分横航向xoy与纵航向yoz，四旋翼中可称为高度控制与姿态控制，固定翼利用速度以及滚转与偏航角（考虑风速是航迹角）控制xoy运动（视线导引），旋翼利用滚转与俯仰角控制xoy运动
% 此外，参考轨迹拿出来的状态是那一时刻参考轨迹上的或者说虚拟轨迹上的状态，并非我们当前智能体的控制输入状态比如说加速度，我们的控制输入状态是当前智能体的控制期望不是跟踪期望，我们是利用加速度控制期望用以跟踪速度或者最终的位置层。
% 并不一定对于跟踪轨迹的物理参考意义，要看利用的参考状态要看控制最上层的输出，
% todo --- 修改俯仰正方向，现在为向下 -y 为正 
% 什么是底层：忽略延迟的瞬间变化量；往往是执行器，针对actuator的命令来源，有两个渠道---其一为控制算法得出，其二是利用控制输入解算得出，简言之，控出来的与算出来的；
% 控出来：固定翼--delta_ele = kp(pitch - pitch_c) - kd*q; pitch_c = kp(h - hc)+ki/s(h - hc)); 
% 算出来：旋翼--acc_des = kp*e_pos..., att_des = asin(acc_des(1)/g) etc; 
% 其中需要区分耦合与耦合，一对一还是多对一，分方向分平面分轴讨论然后 其他定零求解，因为不定零也是个des_from_control 的已知值流程相同
% 比如拿到所有的控制xy位置得到的期望姿态再次pid控制得到的期望力矩，结合控制期望z高度得到的力，解算出四个电机的油门推力

% xyz 右前上，abc电机分别 位置右左尾 转向顺逆逆 带来反扭 逆顺顺 其中逆是朝z正方向 所以是 + - -
% 右手定则坐标系 右前上 对应 东北天
% --- todo 校正所有坐标正向对应

%% 参考轨迹需要 旋翼下 pos， yaw----  固定翼下 es_state 需要添加 des_state.mode， 添加des_state.Va 


close all;

clear;


addpath('utils');
addpath('traj');
addpath('controller');
addpath('test_airplane');

%% copter mode 1-- pre-calculated trajectories 顶层-位置与偏航期望

% trajhandle = @traj_helix;
% trajhandle = @traj_line; 
% trajhandle = @traj_circle; 
% trajhandle = @traj_4point_step;
trajhandle = @traj_vtol_global;
% trajhandle = @traj_cruise;



%% cruise mode 3-- pre-calculated trajectories--dubin based 顶层-偏航角 高度 空速
%% You need to implement this
% trajhandle = @traj_generator;
% trajhandle = @traj_dubin; 
% trajhandle = @traj_4point_cruise;



%% controller selection

% controlhandle = @copter_controller_Done;
controlhandle = @global_controller_main_testing;
% controlhandle = @global_controller_transition;
% controlhandle = @cruise_controller_needtodo;



% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state] = simulation_3d_ttr(trajhandle, controlhandle); % 




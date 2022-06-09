function dY = dynamics(~,Y,u)
%%  2v2 空战动力学模型
% phi为航迹倾角，gamma为航迹滚转角，psi为航迹偏转角
% 机动决策库为{左偏置；稳定飞行；最大加速；最大减速；右偏置；爬升；俯冲}
% u = [ nx nf gamma]        切向过载 法向过载 滚转角
% 左偏置：u = [0 nf_max -arccos(1/nf_max)]
% 稳定平飞：u = [0 1 0]
% 最大加速：u = [nx_max 1 0]
% 最大减速：u = [-nx_max 1 0]
% 右偏置；u = [0 nf_max arccos(1/nf_max)]
% 爬升：u = [ 0 nf_max 0]
% 俯冲：u = [ 0 -nf_max 0]

%%
global g 

x = Y(1); y = Y(2); z = Y(3);
v = Y(4); phi = Y(5); psi =Y(6);

dx = v * cos(phi) * cos(psi);
dy = v * cos(phi) * sin(psi);
dz = v * sin(phi);


% nx = (P - D) / (M * g);
% ny = Y * cos(gamma) / (M * g);
% nz = Y * sin(gamma) / (M * g);
% nf = sqrt(nz^2 + ny^2);

nx = u(:,1);
nf = u(:,2);
gamma = u(:,3);

dv = g * (nx - sin(gamma));
dphi = g / v * (nf * cos(gamma) - cos(phi));
dpsi =  - g * nf * sin(gamma) / (v * cos(phi));

dY = [dx; dy; dz;
    dv; dphi; dpsi];




end
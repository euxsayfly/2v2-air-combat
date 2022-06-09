function a = predictD(y1,y2,u,dt)
%% 状态预测
% 在当前作动下，一段时间后的目标优势函数值
%%
U = [0 1 0];

[~,y1s] = ode45(@(t,y) dynamics(t,y,u),[0,dt],y1);
[~,y2s] = ode45(@(t,y) dynamics(t,y,U),[0,dt],y2);

y1_ = y1s(end,:)';
y2_ = y2s(end,:)';

a = advantage(y2_,y1_);

end
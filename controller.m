function u = controller(y1,y2,dt)
%% 作动选择
%   2v2 控制机动函数
%   U 飞机机动决策库
%   以己方优势最大选
%%
global nx_max nf_max

A = zeros(7, 1);

U = [0, nf_max, -acos(1/nf_max);        % 左偏置
    0, 1, 0;                            % 稳定平飞
    nx_max, 1, 0;                       % 最大加速
    -nx_max, 1, 0;                      % 最大减速
    0, nf_max, acos(1/nf_max);          % 右偏置
    0, nf_max, 0;                       % 爬升
    0, -nf_max, 0];                     % 俯冲

for k = 1:7
    A(k) = predict(y1,y2,U(k,:),dt);
end



A_m = max(A);

switch A_m
    case A(1)
        u = U(1,:);
    case A(2)
        u = U(2,:);
    case A(3)
        u = U(3,:);
    case A(4)
        u = U(4,:);
    case A(5)
        u = U(5,:);
    case A(6)
        u = U(6,:);
    case A(7)
        u = U(7,:);
end

end


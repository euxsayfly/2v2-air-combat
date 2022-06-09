function [y,t] = target(y1,y2,y3)
%% 目标选取函数
% 二选一，一直选取优势函数大的
%%
y12 = advantage(y1,y2);
y13 = advantage(y1,y3);

if y12 > y13
    y = y2;
 else
    y = y3;
    
end
if y == y2
    t = 1;
else
    t = 2;
end


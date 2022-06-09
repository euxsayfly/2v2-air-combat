function G = advantage(Y1, Y2)
%% 态势函数
%%
global sigma1 sigma2 R_max R_min V_min V_max h sigmah sigmah1 sigmah2 omega_A omega_R omega_v omega_h


x1 = Y1(1); y1 = Y1(2); z1 = Y1(3); v1 = Y1(4); phi1 = Y1(5); psi1 = Y1(6);
x2 = Y2(1); y2 = Y2(2); z2 = Y2(3); v2 = Y2(4); phi2 = Y2(5); psi2 = Y2(6);

H1 = Y1(3); H2 = Y2(3);

d_vector = [x2 - x1; y2 - y1; z2 - z1;];
d = norm(d_vector);

v1_vector = [v1 * cos(phi1) * cos(psi1); v1 * cos(phi1) * sin(psi1); v1 * sin(phi1)];
v2_vector = [v2 * cos(phi2) * cos(psi2); v2 * cos(phi2) * sin(psi2); v2 * sin(phi2)];

alpha1 = acos(dot(v1_vector, d_vector) / (v1 * d));
alpha2 = acos(dot(v2_vector, d_vector) / (v2 * d));

% mu_A = abs(alpha1) * abs(alpha2) / pi^2;                        % 角度优势函数
  mu_A = 1 - (abs(alpha1) + abs(alpha2)) / (2 * pi);

if d < R_min
    mu_R = exp(-(d - R_min)^2 / (2 * sigma1^2));
    V_ = v1 - (v1 - V_min) * (1 - exp((d - R_min) / R_min));
else
    if d <= R_max
        mu_R = 1;
        V_ = v2;
    else
        mu_R = exp(-(d - R_max)^2 / (2 * sigma2^2));
        V_ = v1 + (V_max - v1) * (1 - exp((R_max - d) / R_max));
    end
end                                                  %  mu_R为距离优势函数，V_为最佳攻击速度

mu_v = exp(- (v1 - V_)^2 / (2 * V_^2));               % 速度优势函数

dz = z1 - z2;                                        % 高度差

if dz <= 0
    mu_h = 0;
else
    if dz < h
        mu_h = exp(-(dz - h)^2 / (2 * sigmah1^2));
    else
        if dz <= h + sigmah
            mu_h = 1;
        else
            mu_h = exp(-(dz - h - sigmah)^2 / (2 * sigmah2^2));
        end
    end
end                                                 % 高度优势函数

G = omega_A * mu_A + omega_R * mu_R + omega_v * mu_v + omega_h * mu_h;


end
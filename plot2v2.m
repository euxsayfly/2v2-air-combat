clf;
figure(1)
box on 

plot3(R1_s(:,1),R1_s(:,2),R1_s(:,3));
plot3(R2_s(:,1),R2_s(:,2),R2_s(:,3));
plot3(B1_s(:,1),B1_s(:,2),B1_s(:,3));
plot3(B2_s(:,1),B2_s(:,2),B2_s(:,3));
xlabel('x轴/m')
ylabel('y轴/m')
zlabel('z轴/m')

grid on 
for i = 1:5:total_k
    cla;
    hold on;
    plot3(R1_s(:,1),R1_s(:,2),R1_s(:,3));
    plot3(R2_s(:,1),R2_s(:,2),R2_s(:,3));
    plot3(B1_s(:,1),B1_s(:,2),B1_s(:,3));
    plot3(B2_s(:,1),B2_s(:,2),B2_s(:,3));
    plot3(R1_s(i,1),R1_s(i,2),R1_s(i,3),'o');
    plot3(R2_s(i,1),R2_s(i,2),R2_s(i,3),'o');
    plot3(B1_s(i,1),B1_s(i,2),B1_s(i,3),'*');
    plot3(B2_s(i,1),B2_s(i,2),B2_s(i,3),'*');
    legend('R_1轨迹','R_2轨迹','B_1轨迹','B_2轨迹','R_1','R_2','B_1','B_2')
    frame=getframe(gcf);
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if i==1
         imwrite(imind,cm,'test1.gif','gif', 'Loopcount',inf,'DelayTime',1e-4);
    else
         imwrite(imind,cm,'test1.gif','gif','WriteMode','append','DelayTime',1e-4);
    end
end

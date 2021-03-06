% close all;
imshow(~bw1)
% figure(2),
hold on;

[col,~]=size(route1);


global r_max bw1
r_max=15;
route1(1:col,3)=r_max;
plot(route1(:,1),route1(:,2),'o');axis equal;

n_order=6;
total_time=50;
[f,ts,ts_k]=demo1_minimum_snap_simple_tube(route1',total_time,n_order);
plot(f(1,:),f(2,:),'-r');

tangent=[f(3,:);
    f(4,:)];
rot=[0 -1;1 0];
normal=rot*tangent;
normal=normal./sum(normal.^2,1).^0.5;
% 固定半径
fo1=f(1:2,:)-r_max.*normal;
plot(fo1(1,:),fo1(2,:),'-b');
fo2=f(1:2,:)+r_max.*normal;
plot(fo2(1,:),fo2(2,:),'-b');
% 变半径
fo1=f(1:2,:)-1./f(5,:).*normal;
plot(fo1(1,:),fo1(2,:),'-k','linewidth',2);
fo2=f(1:2,:)+1./f(6,:).*normal;
plot(fo2(1,:),fo2(2,:),'-k','linewidth',2);
axis equal;
% legend('o','*','generating curve',...
%     'offset curve 1','offset curve 2',...
%     'virtual curve 1','virtual curve 2');
% plot(fo1(1,ts_k),fo1(2,ts_k),'or');
% plot(fo2(1,ts_k),fo2(2,ts_k),'or');





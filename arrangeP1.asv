function [ts_out,route_l_out,route_r_out,ts_k1]=arrangeP1(fx1,fx2,fy1,fy2,ft)
% 输入曲线，输出半径节点以及相应的时间规划
% load('arrangeP.mat');
global r_max

rot=[0 -1;
    1 0];
ff1=[fx1;
    fy1];
ff2=[fx2;
    fy2];
n_o=rot*ff1;%定向法向量
ff1_unit=ff1./sum(ff1.^2,1).^0.5;
n=diff(ff1_unit,1,2);% 单位法向量，指向曲线内侧
n=[n n(:,end)];
flag_dir=sum(n.*n_o,1);

ts_k=1;
for k=2:size(flag_dir,2)-1
    if flag_dir(k)*flag_dir(k+1)<0
%         判断曲线弯曲方向是否反向
       ts_k=[ts_k k];
    end
end
ts_k=[ts_k k+1];%记录转折点处下标

ts_k1=[];%记录曲率最大处下标
route_r=[];%记录曲率最大处曲率大小
route_l=[];%记录曲率最大处曲率大小
ts=[];
for k=1:length(ts_k)-1
    curvature=1/r_max;
    tmp_k1=ts_k(k);
%     在分段中找到曲率最大的地方的k和曲率大小
   for k1=ts_k(k):ts_k(k+1)
       tmp=norm(cross([ff1(:,k1);0],[ff2(:,k1);0]))/norm(ff1(:,k1))^3;
       if tmp>curvature
          curvature=tmp; 
          tmp_k1=k1;
       end 
   end
   ts_k1=[ts_k1 tmp_k1];
   ts=[ts ft(tmp_k1)];
   if flag_dir(tmp_k1)>0
      route_r=[route_r curvature]; 
      route_l=[route_l 1/r_max];
   else
       route_l=[route_l curvature];
       route_r=[route_r 1/r_max];
   end
end

if ts_k1(1)>1
   ts_k1=[1 ts_k1]; 
   ts=[0 ts];
   route_r=[1/r_max route_r];
   route_l=[1/r_max route_l];
end
if ts_k1(end)< length(flag_dir)
    ts_k1=[ts_k1 length(flag_dir)];
    ts=[ts ft(length(flag_dir))];
    route_l=[route_l 1/r_max];
    route_r=[route_r 1/r_max];
end



route_l_out=1./route_l;
route_r_out=1./route_r;
ts_out=ts;
end
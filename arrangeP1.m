function [ts_out,route_l_out,route_r_out,ts_k1,mindis_r,mindis_l]=arrangeP1(fx,fx1,fx2,fy,fy1,fy2,ft)
% 输入曲线，输出半径节点以及相应的时间规划
% load('arrangeP.mat');
global r_max bw1 

rot=[0 -1;
    1 0];
ff=[fx;
    fy];
ff1=[fx1;
    fy1];
ff2=[fx2;
    fy2];
n_o=rot*ff1;%定向法向量
ff1_unit=ff1./sum(ff1.^2,1).^0.5;
n=diff(ff1_unit,1,2);% 单位法向量，指向曲线内侧
n=[n n(:,end)];
flag_dir=sum(n.*n_o,1);

%% 查询曲率最大处 
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

%% 查询与管道冲突的障碍物
mindis_k=zeros(length(ts_k1)-1,2);
for k=1:length(ts_k1)-1%遍历所有段
    mindis=r_max;
    for k1=ts_k1(k):ts_k1(k+1)
       xp=round(ff(1,k1));
       yp=round(ff(2,k1));
       bili=n_o(2,k1)/n_o(1,k1);
       tmp=0;
       for k2=1:round(r_max/(1+bili^2)^0.5)
          tmp=tmp+bw1(round(yp+k2*bili*sign(n_o(1,k1))),xp+k2*sign(n_o(1,k1)));
       end
%        plot([xp,xp+k2*sign(n_o(1,k1))],[yp,round(yp+k2*bili*sign(n_o(1,k1)))],'k');
       mindis_tmp=r_max-tmp;
       if mindis_tmp<mindis
          mindis_k(k,1)=k1;
          mindis_k(k,2)=mindis_tmp;
          mindis_k(k,3)=ft(k1);
          mindis=mindis_tmp;
       end
    end
    
    if mindis_k(k,1)~=0
        tmp_k1=mindis_k(k,1);
        tmp_dis=mindis_k(k,2);
        plot([ff(1,tmp_k1),ff(1,tmp_k1)+tmp_dis*n_o(1,tmp_k1)/norm(n_o(:,tmp_k1))],...
            [ff(2,tmp_k1),ff(2,tmp_k1)+tmp_dis*n_o(2,tmp_k1)/norm(n_o(:,tmp_k1))],'--r');
    end
end
mindis_r=mindis_k;

n_o=-n_o;
mindis_k=zeros(length(ts_k1)-1,2);
for k=1:length(ts_k1)-1%遍历所有段
    mindis=r_max;
    for k1=ts_k1(k):ts_k1(k+1)
       xp=round(ff(1,k1));
       yp=round(ff(2,k1));
       bili=n_o(2,k1)/n_o(1,k1);
       tmp=0;
       for k2=1:round(r_max/(1+bili^2)^0.5)
          tmp=tmp+bw1(round(yp+k2*bili*sign(n_o(1,k1))),xp+k2*sign(n_o(1,k1)));
       end
%        plot([xp,xp+k2*sign(n_o(1,k1))],[yp,round(yp+k2*bili*sign(n_o(1,k1)))],'k');
       mindis_tmp=r_max-tmp;
       if mindis_tmp<mindis
          mindis_k(k,1)=k1;
          mindis_k(k,2)=mindis_tmp;
          mindis_k(k,3)=ft(k1);
          mindis=mindis_tmp;
       end
    end
    
    if mindis_k(k,1)~=0
        tmp_k1=mindis_k(k,1);
        tmp_dis=mindis_k(k,2);
        plot([ff(1,tmp_k1),ff(1,tmp_k1)+tmp_dis*n_o(1,tmp_k1)/norm(n_o(:,tmp_k1))],...
            [ff(2,tmp_k1),ff(2,tmp_k1)+tmp_dis*n_o(2,tmp_k1)/norm(n_o(:,tmp_k1))],'--r');
    end
end
mindis_l=mindis_k;

route_l_out=1./route_l;
route_r_out=1./route_r;
ts_out=ts;
end
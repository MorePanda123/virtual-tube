function [curvature_max1_out,t_max1_out,curvature_max2_out,t_max2_out]=find_max_curvature(xx1,xx2,yy1,yy2,tt,r_max,dir)
% load('find_max_curvature.mat');

rot=[0 -dir;
    dir 0];
ff1=[xx1;
    yy1];
ff2=[xx2;
    yy2];
n_o=rot*ff1;%定向法向量
ff1_unit=ff1./sum(ff1.^2,1).^0.5;
% 单位法向量，指向曲线内侧
n=diff(ff1_unit,1,2);
n=[n n(:,end)];
flag_dir=sum(n.*n_o,1);

col=size(ff1,2);
curvature_max1=1/r_max;
% curvature_max2=1/r_max;
t_max1=tt(1);
% t_max2=tt(1);
for k=1:floor(col/5):col
    curvature=min(norm(cross([ff1(:,k);0],[ff2(:,k);0]))/norm(ff1(:,k))^3,0.1);
%     if flag_dir(k)>0
%        if  curvature>curvature_max1
%           curvature_max1=curvature;
%           t_max1=tt(k);
%        end
%     elseif flag_dir(k)<0
%         if  curvature>curvature_max2
%             curvature_max2=curvature;
%             t_max2=tt(k);
%         end
%     end
   if flag_dir(k)>0
       if  curvature>curvature_max1
          curvature_max1=curvature;
          t_max1=tt(k);
       end
   else 
       if  curvature>curvature_max1
           curvature_max1=curvature;
           t_max1=tt(k);
       end
   end
end
if flag_dir(k)>0
    curvature_max1_out=curvature_max1;
    t_max1=tt(k);
else
    curvature_max1_out=1/r_max;
    t_max1=tt(k);
end
% curvature_max2_out=curvature_max2;
t_max1_out=t_max1;
% t_max2_out=t_max2;
end
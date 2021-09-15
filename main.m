close all;clear all;
%% ������ɵ�ͼ
map_x=32;
map_y=18;
obstacle.num=100;
obstacle.rmax=0.8;
obstacle.obs=draw_obstacle(map_x,map_y,obstacle.num,obstacle.rmax);
%% ץȡͼ�������߼�����
f=getframe(gcf);
img=f.cdata;
bw1=~im2bw(img);

%% �ָ�ͼƬ
bili=20;
bw1=double(bw1);
[row,col]=size(bw1);
itv=floor(min(row,col)/bili);
count_r=0;count_c=0;
for k=1:itv:row-itv+1
    count_r=count_r+1;
   for m=1:itv:col-itv+1
       count_c=count_c+1;
      if sum(sum(bw1(k:k+itv-1,m:m+itv-1))) >itv*itv/4
          bw2(count_r,count_c)=1;
      else
          bw2(count_r,count_c)=0;    
      end
   end
   count_c=0;
end
% figure,imshow(bw2);
[row2,col2]=size(bw2);
for k=1:row2
   for m=1:col2
      if bw2(k,m)==1
         bw2(k,m)=-inf;
      else
          bw2(k,m)=inf;
      end
   end
end
hold off;
imshow(~bw1);
hold on;
%% ѡȡ�����յ�
[start_pos(1,1),start_pos(1,2)]=ginput(1);
% plot(start_pos(1,1),start_pos(1,2),'rx');
text(start_pos(1,1)+0.1,start_pos(1,2),'start point');
[end_pos(1,1),end_pos(1,2)]=ginput(1);
% plot(end_pos(1,1),end_pos(1,2),'bo');
text(end_pos(1,1)+0.1,end_pos(1,2),'end point');
start_pos=start_pos/itv;
end_pos=end_pos/itv;

%% ·������
[route]=AStar(bw2',ceil(start_pos),ceil(end_pos));
route1=(route-0.5)*itv;
figure(1),
plot(route1(1:end,1),route1(1:end,2),'--');
% figure,image(bw2,'CDataMapping','scaled');hold on;
% plot(route(:,1),route(:,2),'.r');
%% ˳��������
% Untitled3
% figure(2),hold on;
% [col,~]=size(route1);
% route1(1:col,3)=50;
% route1(1:col,4)=rand(col,1)*0.1;
% plot(route1(:,1),route1(:,2),'o');axis equal;
% 
% n_order=5;
% total_time=50;
% [f,ts,polys_x,polys_y,polys_r]=demo1_minimum_snap_simple(route1',total_time,n_order);
% 
% plot(f(1,:),f(2,:),'-r');
% 
% tangent=diff(f(1:2,:),1,2);
% tangent=[tangent tangent(:,end)];
% rot=[0 -1;1 0];
% normal=rot*tangent;
% normal=normal./sum(normal.^2,1).^0.5;
% fo1=f(1:2,:)+f(3,:).*normal;
% plot(fo1(1,:),fo1(2,:),'-b');
% fo2=f(1:2,:)-f(3,:).*normal;
% plot(fo2(1,:),fo2(2,:),'-b');
% quiver3(f(1,:),f(2,:),f(3,:),f(1,:)+0.05*cos(f(4,:)),f(2,:)+0.05*sin(f(4,:)),f(3,:));
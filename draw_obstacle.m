function [pos]=draw_obstacle(map_x,map_y,num,rmax)

figure,
hold on;axis equal;axis off;
axis([0 map_x 0 map_y]);
for k=1:num
   obstacle.obs(k).pos=[rand(1,1)*map_x,rand(1,1)*map_y];
   obstacle.obs(k).r=rand(1,1)*rmax;
   for k1=1:20
       draw_x(k1)=obstacle.obs(k).r*cos(k1/10*pi)+obstacle.obs(k).pos(1,1);
       draw_y(k1)=obstacle.obs(k).r*sin(k1/10*pi)+obstacle.obs(k).pos(1,2);
   end
   fill(draw_x,draw_y,'k');
end
pos=obstacle.obs;
end
clear;
clc;

imu = csvread('imu.csv',1,0);
t = (imu(:,1)-imu(1,1)).*10^(-9);
w_x = imu(:,2);
w_y = imu(:,3);
w_z = imu(:,4);
a_x = imu(:,5);
a_y = imu(:,6);
a_z = imu(:,7);
out = [t,w_x,w_y,w_z,a_x,a_y,a_z];

fid = fopen('imu.txt','wt');
matrix = out;
[m,n]=size(matrix);                      
 for i=1:1:m
   for j=1:1:n
      if j==n
        fprintf(fid,'%f\n',matrix(i,j));
     else
       fprintf(fid,'%f\t',matrix(i,j));
      end
   end
end
fclose(fid);

gt = csvread('gt.csv',1,0);
t2 = (gt(:,1)-gt(1,1)).*10^(-9);
gt_x = gt(:,2);
gt_y = gt(:,3);
gt_z = gt(:,4);
gt_qw = gt(:,5);
gt_qx = gt(:,6);
gt_qy = gt(:,7);
gt_qz = gt(:,8);
gt_vx = gt(:,9);
gt_vy = gt(:,10);
gt_vz = gt(:,11);

out2 = [t2,gt_x,gt_y,gt_z,gt_qw,gt_qx,gt_qy,gt_qz,gt_vx,gt_vy,gt_vz];

fid2 = fopen('gt.txt','wt');
matrix2 = out2;
[m,n]=size(matrix2);                      
 for i=1:1:m
   for j=1:1:n
      if j==n
        fprintf(fid,'%f\n',matrix2(i,j));
     else
       fprintf(fid,'%f\t',matrix2(i,j));
      end
   end
end
fclose(fid2);
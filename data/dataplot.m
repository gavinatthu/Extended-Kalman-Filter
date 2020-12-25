clear;
clc;
imu = load('imu_noise.txt');
gt = load('traj_gt.txt');
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

t = imu(:, 1);
plot(t,gt_vx,t,gt_vy,t,gt_vz)

p_x = imu(:, 2);
p_y = imu(:, 3);
p_z = imu(:, 4);

% 初值设定
Fs = 50;                                                % 采样频率                    
T = 1/Fs;                                               % 采样周期      
L = 12000;                                              % 信号长度
%t = (0:L-1)*T;                                          % 时间向量

% 对输入信号尾部补零，并进行FFT变换
input = p_x;
input = [input,;zeros(L-length(input),1)]; 

output = fft(input);

% 计算双侧频谱 P2。然后基于 P2 和偶数信号长度 L 计算单侧频谱 P1。
P2 = abs(output/L);                                     
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

% 在时域绘制原始输入信号
figure(1)
plot(t,input,t,gt_x)
title('Input signal')
xlabel('t(seconds)')
ylabel('X(t)')

%定义频域 f 并绘制单侧幅值频谱 P1
figure(2)
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('FFT of input signal')
xlabel('f (Hz)')
ylabel('|P1(f)|')




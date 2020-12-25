clear;
clc;

imu = csvread('imu.csv',1,0);
t = (imu(:,1)-imu(1,1)).*10^(-9);
a_x = imu(:,5);
a_y = imu(:,6);
a_z = imu(:,7);
w_x = imu(:,2);
w_y = imu(:,3);
w_z = imu(:,4);

% 对输入信号进行FFT
Fs = 50;
input = w_z;
L = length(input); 
y = fft(input);

% Yule-Walker方法估计
ywxx = pyulear(input,16);

P2 = abs(y/L);                                     
P1 = P2(1:(L/2+1));
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:L/2)/L;

[output,f2]= pwelch(input,rectwin(1000),200,500,Fs);

figure(1)
plot(t,a_x,t,a_y)
legend('竖直方向加速度','水平方向加速度')
title('Input signal')
xlabel('t(seconds)')
ylabel('X(t)')

figure(2)
plot(t,w_x,t,w_y)
legend('滚转角方向角速度','俯仰角方向角速度')
title('Input signal')
xlabel('t(seconds)')
ylabel('X(t)')

figure(3)
plot(f,P1,'color','#D95319') 
title('FFT of input signal')
xlabel('f (Hz)')
ylabel('|P1(f)|')

figure(4)
plot((0:1/128:1),ywxx,'color','#D95319') 
title('Yule-Walker方法估计'); 
xlabel('归一化频率 \omega / \pi')
ylabel('幅度 dB/rad');

F=1;
M=0.6;
m=0.065;
l=0.178;
I=0.0027;
b=0.1;
g=9.81;
Sim_Time=10;

%PID CONTROLLER
Kp=120;
Ki=1;
Kd=25;
sim('Pendelum_Model_2016')

%Plot
figure
plot(IN.time, IN.data)
hold all
plot (OUT.time, OUT.data)
axis([0, 5, -0.2, 0.2]);
xlabel('Time (Seconds)')
ylabel('Amplitude')
title({'Case 4';' PID Gains: Kp = 120, Ki = 1, Kd = 25'});
lsiminfo(OUT.data, OUT.time)

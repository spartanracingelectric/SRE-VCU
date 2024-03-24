clear
clc
close all

%% TorqueVectoring Simulink Results

slipTarget = .2; %0.2*100

%Slip Control Gains
Kp_slip = 250;%250;%100000;%1500;%1500%20000; %can be larger to start with less steady state error
%% 
Ki_slip = 50;%50;%1000;%700%1000;
Kd_slip = 0;
slip_windupUpper = inf;
slip_windupLower = -inf;

%Yaw Control Gains
Kp_yaw = 100;%10;%25000;%25000;%35000 %155 set a constant yaw rate target goes. goes unstable at 70000. Divide by 2 %tested at 20% steer w/ fixed signal of 10 yaw rate setpoint
Ki_yaw = 10;%10;
Kd_yaw = 0;%100

friction = readtable('LookupTables.xlsx','Sheet','LongFriction');
frictionTable = friction(1:31,3:6);
frictionSlipRatio = friction(1:31,1:1);
frictionLoads = [50, 150, 250, 350];
slipFriction = table2array(frictionTable);
slipRatio = table2array(frictionSlipRatio);

Actualfriction = readtable('LookupTables.xlsx','Sheet','LongFrictionActual');
ActualfrictionTable = Actualfriction(1:31,3:6);
ActualFriction = table2array(ActualfrictionTable);


yawrate = readtable('LookupTables.xlsx','Sheet','YawRate');
yawrateTable = yawrate(1:13,3:10);
yawrateSteering = yawrate(1:13,1:1);
yawrateVelocity = [0, 5, 10, 15, 20, 25, 30, 35];
yawRate = table2array(yawrateTable);
steering = table2array(yawrateSteering);
yaw_prev_error = 0; % yaw previous error for PID function
%%

% open('TorqueVectoring_6_1.slx');
% sim('TorqueVectoring_6_1.slx');

%% Plot
% TorqueVectoringPlotter
% 
% autoArrangeFigures(4,3,3);

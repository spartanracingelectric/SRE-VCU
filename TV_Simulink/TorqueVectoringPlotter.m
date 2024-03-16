%% TorqueVecotoringPlotter


clc
close all

%% Professional Plotting

% The standard values for colors saved in PLOT_STANDARDS() will be accessed from the variable PS
PS = PLOT_STANDARDS();

% Figure 1
%========================================================
rows = 2; % # of rows for sublplot
columns = 1; % # of columns for subplot
Throtplot= columns+1;

figure(1);

subplot(rows,columns,1);
driverInput.fig = gcf;
driverInput.p1 = plot(ans.steeringAngle); % , 'o'
title('Steering Angle');
xlabel('Time (s)');
ylabel('Steering Angle');
ylim([-60 60]);
%STANDARDIZE_FIGURE(driverInput);

subplot(rows,columns,2);
driverInput.fig = gcf;
driverInput.p2 = plot(ans.TorquePer*100);
title('Torque Percentage');
xlabel('Time (s)');
ylabel('Percentage');
ylim([-100 100]);

%legend([driverInput.p1, driverInput.p2], 'Vx', 'Vy');
%legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
%driverInput.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
set(driverInput.p1, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', PS.Blue1, 'MarkerEdgeColor', PS.Blue3);    % SET PLOT PROPERTIES
set(driverInput.p2, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', PS.Blue1, 'MarkerEdgeColor', PS.Blue3)
%set(driverInput.p2, 'LineStyle', '--', 'LineWidth', 2, 'Color', PS.MyRed);                                                                             % Choices for COLORS can be found in ColorPalette.png
%STANDARDIZE_FIGURE(driverInput);

% Figure 2 Velocity
%========================================================
figure(2);
Velocity.fig = gcf;
hold on
Velocity.p1 = plot(ans.Vx_2); 
Velocity.p2 = plot(ans.Vy_2);
Velocity.p3 = plot(ans.V_Resultant);
hold off
%========================================================
% ADD LABELS, TITLE, LEGEND
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend([Velocity.p1, Velocity.p2, Velocity.p3], 'Vx', 'Vy','Resultant Velocity');
legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
Velocity.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
set(Velocity.p1, 'LineStyle', '--', 'LineWidth', 3, 'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', PS.Blue1, 'MarkerEdgeColor', PS.Blue3);    % SET PLOT PROPERTIES
set(Velocity.p2, 'LineStyle', '--', 'LineWidth', 3, 'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', PS.MyRed, 'MarkerEdgeColor', PS.MyRed); 
set(Velocity.p3, 'LineStyle', '--', 'LineWidth', 5, 'Color', PS.MyOrange);   
%STANDARDIZE_FIGURE(Velocity);

% Figure 3 Yaw
%========================================================
figure(3);
subplot(rows,columns,1);
Yaw.fig = gcf;
hold on
Yaw.p1 = plot(ans.tout,ans.YawRate); % , 'o'
Yaw.p2 = plot(ans.YawRateSetpoint);
%Yaw.p3 = plot(ans.YawRateError);
hold off

title('Yaw Rate');
xlabel('Time (s)');
ylabel('Yaw Rate (Degrees/S)');
%xlim([1.9 3])
%ylim([9.9 10.4]);
legend([Yaw.p1,Yaw.p2], 'Yaw Rate', 'Yaw Rate Setpoint');%,Yaw.p3 ,'Yaw Rate Error'
legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
Yaw.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
set(Yaw.p1, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Blue1);    % SET PLOT PROPERTIES
set(Yaw.p2, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Red1);    % SET PLOT PROPERTIES
%set(Yaw.p3, 'LineStyle', '--', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', PS.Orange1, 'MarkerEdgeColor', PS.Orange3); 
%STANDARDIZE_FIGURE(Yaw);

subplot(rows,columns,2);
Yaw.fig = gcf;
Yaw.p4 = plot(ans.YawMomentTarget);
title('Yaw Moment Target');
xlabel('Time (s)');
ylabel('Yaw Moment (Nm');
%ylim([-100 100]);

legend([Yaw.p4], 'Yaw Moment Target');
legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
Yaw.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
set(Yaw.p4, 'LineStyle', '--', 'LineWidth', 3, 'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', PS.Blue1, 'MarkerEdgeColor', PS.Blue3);    % SET PLOT PROPERTIES
%STANDARDIZE_FIGURE(Yaw);

% Figure 4 Slip Ratio
%========================================================
figure(4);


Slip.fig = gcf;
hold on
Slip.p1 = plot(ans.tout,ans.FLsr1); % , 'o'
Slip.p2 = plot(ans.tout,ans.FRsr1);
Slip.p3 = plot(ans.tout,ans.RLsr1);
Slip.p4 = plot(ans.tout,ans.RRsr1);
Slip.p5 = plot(ans.slipTarget);
%Slip.p6 = plot(ans.slipTarget*-1);
hold off
title('Slip Ratio');
xlabel('Time (s)');
ylabel('Slip Ratio');
ylim([0 0.5]);
%xlim([0 2]);


legend([Slip.p1, Slip.p2, Slip.p3, Slip.p4, Slip.p5], 'FL Slip Ratio', 'FR Slip Ratio','RL Slip Ratio','RR Slip Ratio', 'Slip Target'); %, Slip.p6
%legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
%driverInput.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
set(Slip.p1, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Blue1)%, 'MarkerSize', 5, 'MarkerFaceColor', PS.Blue1, 'MarkerEdgeColor', PS.Blue1);    % SET PLOT PROPERTIES
set(Slip.p2, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Red1)%, 'MarkerSize', 5, 'MarkerFaceColor', PS.Red1, 'MarkerEdgeColor', PS.Red1);
set(Slip.p3, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Green1)%, 'MarkerSize', 5, 'MarkerFaceColor', PS.Green1, 'MarkerEdgeColor', PS.Green1);
set(Slip.p4, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Orange1)%, 'MarkerSize', 5, 'MarkerFaceColor', PS.Orange1, 'MarkerEdgeColor', PS.Orange1);
set(Slip.p5, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.MyBlack)%, 'MarkerSize', 5, 'MarkerFaceColor', PS.MyBlack, 'MarkerEdgeColor', PS.MyBlack);      
%set(Slip.p6, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Red5, 'MarkerEdgeColor', PS.Red5);    % Choices for COLORS can be found in ColorPalette.png
%STANDARDIZE_FIGURE(Slip);

% Figure 5 Slip Error
%========================================================
figure(5);


Slip.fig = gcf;
hold on
Slip.p1 = plot(ans.FLslipError); 
Slip.p2 = plot(ans.FRslipError);
Slip.p3 = plot(ans.RLslipError);
Slip.p4 = plot(ans.RRslipError);
Slip.p5 = plot(ans.slipTarget);
Slip.p6 = plot(ans.slipTarget*-1);
hold off
title('Slip Error');
xlabel('Time (s)');
ylabel('Slip Error');
%ylim([-1 1]);


legend([Slip.p1, Slip.p2, Slip.p3, Slip.p4, Slip.p5, Slip.p6], 'FL Slip Ratio', 'FR Slip Ratio','RL Slip Ratio','RR Slip Ratio', 'Slip Target', 'Negative Slip Target');
%legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
%driverInput.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
set(Slip.p1, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Blue1, 'MarkerEdgeColor', PS.Blue3);    % SET PLOT PROPERTIES
set(Slip.p2, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Red1, 'MarkerEdgeColor', PS.Red1);
set(Slip.p3, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Blue5, 'MarkerEdgeColor', PS.Blue5);
set(Slip.p4, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 4, 'MarkerFaceColor', PS.Red5, 'MarkerEdgeColor', PS.Red5);
set(Slip.p5, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Red5, 'MarkerEdgeColor', PS.Blue5);      
set(Slip.p6, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Red5, 'MarkerEdgeColor', PS.Red5);    % Choices for COLORS can be found in ColorPalette.png
%STANDARDIZE_FIGURE(Slip);

% Figure 6 X & Y Position
%========================================================
figure(6);
Position.fig = gcf;
hold on
Position.p1 = plot(ans.xPosition, ans.yPosition); % , 'o'
%Position.p2 = plot(ans.Vy_2);
hold off
%========================================================
% ADD LABELS, TITLE, LEGEND
title('Position');
xlabel('X Position (m)');
ylabel('Y Position (m)');
ylim([-20 20]);
%legend([Position.p1, Position.p2], 'Vx', 'Vy','Resultant Velocity');
%legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
%Velocity.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
set(Position.p1, 'LineStyle', '--', 'LineWidth', 3, 'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', PS.Blue1, 'MarkerEdgeColor', PS.Blue3);    % SET PLOT PROPERTIES
%set(Velocity.p2, 'LineStyle', '--', 'LineWidth', 3, 'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', PS.MyRed, 'MarkerEdgeColor', PS.MyRed); 
%set(Velocity.p3, 'LineStyle', '--', 'LineWidth', 5, 'Color', PS.MyOrange);   
%STANDARDIZE_FIGURE(Position);



% Figure 7 Wheel Torques
%========================================================
figure(7);

Torque.fig = gcf;
%subplot(rows,columns,1);
%Torque.fig = gcf;
hold on
Torque.p1 = plot(ans.tout,ans.T_FL); % , 'o'
Torque.p2 = plot(ans.tout,ans.T_FR);
Torque.p3 = plot(ans.tout,ans.T_RL);
Torque.p4 = plot(ans.tout,ans.T_RR);
hold off

title('Wheel Torques');
xlabel('Time (s)');
ylabel('Torque (Nm)');

figure(8);
hold on
Torque.p5 = plot(ans.tout,ans.distributedtorqueFL);
Torque.p6 = plot(ans.tout,ans.distributedtorqueFR);
Torque.p7 = plot(ans.tout,ans.distributedtorqueRL);
Torque.p8 = plot(ans.tout,ans.distributedtorqueRR);
hold off
title('Distributed Torques');
xlabel('Time (s)');
ylabel('Torque (Nm)');
%ylim([-60 60]);


legend([Torque.p1, Torque.p2, Torque.p3, Torque.p4], 'FL Tq Demand', 'FR Tq Demand','RL Tq Demand','RR Tq Demand');%, 'FL Tq Max', 'FR Tq Max', 'RL Tq Max', 'RR Tq Max'); %, Torque.p5, Torque.p6, Torque.p7, Torque.p8
%legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
%driverInput.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
set(Torque.p1, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Blue1);    % SET PLOT PROPERTIES
set(Torque.p2, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Red1);
set(Torque.p3, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Green1);
set(Torque.p4, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Orange1);
set(Torque.p5, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Blue1);    % SET PLOT PROPERTIES
set(Torque.p6, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Red1);
set(Torque.p7, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Green1);
set(Torque.p8, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Orange1);
%STANDARDIZE_FIGURE(Torque);

% Figure 8 Friction Trim
%========================================================
figure(9);

Trim.fig = gcf;
hold on
Trim.p1 = plot(ans.tout,ans.FLtrim); % , 'o'
Trim.p2 = plot(ans.tout,ans.FRtrim);
Trim.p3 = plot(ans.tout,ans.RLtrim);
Trim.p4 = plot(ans.tout,ans.RRtrim);
hold off
title('Friction Trim');
xlabel('Time (s)');
ylabel('Trim value');
%legend([Trim.p1, Trim.p2, Trim.p3, Trim.p4], 'FL Trim', 'FR Trim','RL Trim','RR Trim');
%STANDARDIZE_FIGURE(Trim);

figure(10);
hold on
Trim.p5 = plot(ans.tout,ans.FLtrimUpdate);
Trim.p6 = plot(ans.tout,ans.FRtrimUpdate);
Trim.p7 = plot(ans.tout,ans.RLtrimUpdate);
Trim.p8 = plot(ans.tout,ans.RRtrimUpdate);
hold off

title('Trim Update - Friction Map');
xlabel('Time (s)');
ylabel('Trim Update value');

%legend([Trim.p1, Trim.p2, Trim.p3, Trim.p4, Trim.p5, Trim.p6, Trim.p7, Trim.p8], 'FL Trim', 'FR Trim','RL Trim','RR Trim', 'FL Trim Update', 'FL Trim Update', 'RL Trim Update', 'RR Trim Update');
%legend([Trim.p1, Trim.p2, Trim.p3, Trim.p4, Trim.p5, Trim.p6, Trim.p7, Trim.p8], 'FL Trim', 'FR Trim','RL Trim','RR Trim','FL Trim Update', 'FR Trim Update','RL Trim Update','RR Trim Update');
%legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
%driverInput.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
set(Trim.p1, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Blue1);    % SET PLOT PROPERTIES
set(Trim.p2, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Red1);
set(Trim.p3, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Green1);
set(Trim.p4, 'LineStyle', '-', 'LineWidth', 2, 'Marker', '_', 'Color', PS.Orange1);

set(Trim.p5, 'LineStyle', '-', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Red5, 'MarkerEdgeColor', PS.Blue5);
set(Trim.p6, 'LineStyle', '-', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Red5, 'MarkerEdgeColor', PS.Red5);
set(Trim.p7, 'LineStyle', '-', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Red5, 'MarkerEdgeColor', PS.Purple5);
set(Trim.p8, 'LineStyle', '-', 'LineWidth', 1, 'Marker', 'x', 'MarkerSize', 2, 'MarkerFaceColor', PS.Red5, 'MarkerEdgeColor', PS.Green5);
%STANDARDIZE_FIGURE(Trim);

%Wheel Torques
% Initial Friction + Current Friction Trim
% Accelerations



%========================================================
% ADD LABELS, TITLE, LEGEND
%title('Driver Input');
%xlabel('x axis data');
%ylabel('y axis Data');
%legend([driverInput.p1, driverInput.p2], 'Vx', 'Vy');
%legendX = .82; legendY = .87; legendWidth = 0.02; legendHeight = 0.02;
%driverInput.legendPosition = [legendX, legendY, legendWidth, legendHeight];                                                                            % If you want the tightest box set width and height values very low matlab automatically sets the tightest box
%set(driverInput.p1, 'LineStyle', 'none', 'LineWidth', 1, 'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', PS.Blue1, 'MarkerEdgeColor', PS.Blue3);    % SET PLOT PROPERTIES
%set(driverInput.p2, 'LineStyle', '--', 'LineWidth', 2, 'Color', PS.MyRed);                                                                             % Choices for COLORS can be found in ColorPalette.png
%STANDARDIZE_FIGURE(driverInput);


%figure(9);
%plot(ans.minT);
%title('minT');

%figure(10);
%plot(ans.T_min);
%title('T_min');

%%
autoArrangeFigures(4,3,3);
%========================================================
% SAVE FIGURE AS AN IMAGE: SAVE_MY_FIGURE(fig1_comps, 'Myfigure.png', 'small');
%SAVE_MY_FIGURE(fig1_comps, 'Figures/PlottingTemplate_Quick_big.png', 'big');
%SAVE_MY_FIGURE(fig1_comps, 'Figures/PlottingTemplate_Quick_small.png', 'small');
% Or if you do not like the default settings used to save the figure
% Step1: Set the properties of your figure
% Step2: Maybe move around the legend location manually on the figure that opens up. Then save or
% Step3: Use the following code

%figure_resolution = 600;
%fig_filename = 'Figures/PlottingTemplate_Quick_Separate.png';
%exportgraphics(fig1_comps.fig, fig_filename, 'Resolution', figure_resolution);







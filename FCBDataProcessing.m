%{
DataProcessing
A script for interpreting the data output from the FCB's serial monitor
nzambid1@umbc.edu
Created: 4/30/25
%}

clear;clc; close all;

rawData = readtable("550cc_1_Hz_50-50_Syringe_Air_Test_2.csv");
headerNames = ["runTime", "cmDesired", "cmActual", "pressure"];
%                 (ms)       (cm)          (cm)        (-)

rawData = renamevars(rawData, 1:width(rawData), headerNames);
dxs = diff(rawData.cmActual);
dts = diff(rawData.runTime)./1000;
speeds = dxs./dts;


figure(1)
hold on
plot(rawData.runTime/1000, rawData.cmDesired);
plot(rawData.runTime/1000, rawData.cmActual);
% plot(rawData.runTime(2:end)/1000, speeds);
hold off
xlabel("Run Time (s)")
ylabel("Displacement (cm)")




avgSpeed = -3.12; % cm/s 
syringeDiameter = 6.49; % cm
flowRate = avgSpeed*pi/4*syringeDiameter^2;

% Test 1: Air, 0.5 Hz, 50-50, No PID.
% Average actuator speed: 3.24 cm/s during extension
% Average actuator speed: -3.12 cm/s during extension
% Average flow rate during extension: 107 cc/s
% Average flow rate during retraction: -103 cc/s

% The draining process significantly slows down when the pocket is almost
% empty. Try leaving a bit of fluid left in there.

% Test 2: Air, 1 Hz, 50-50, No PID.
% Average % error: 8.67%
%% check goal dynamixel positions versus measured positions
clear all;
close all;
clc;
addpath(genpath('matlab_helpers'));
%% import data
% load dat
data_ = importdata('to_train/E8_top.txt');

%temporary input data format
% [1,    2,  3,  4,  5-12,    13-16,    17,      18,    19,     20,      21,        22,        23,              24,         25,           26,          27,                28,         29,   30,      31,    32,     33,        34,       35,     36,       37,     38,    39,       40   ]
% [time, Fx, Fy, Fz, s1 - s8,   tof,  x_des,  y_des,  z_des, contact, pitch_des, roll_des, x_act pulse , y1_act pulse, y2_act pulse, z_act pulse, pitch_act pulse, roll_act pulse, x_act, y1_act, y2_act, z_act, pitch_act, roll_act,  
%% extract neccessary data
x_des = data_(:,23);
y1_des = data_(:,24);
y2_des = data_(:,25);
z_des = data_(:,26);
pitch_des = data_(:,27);
roll_des = data_(:,28);


x_act = data_(:,35);
y1_act = data_(:,36);
y2_act = data_(:,37);
z_act = data_(:,38);
pitch_act = data_(:,39);
roll_act = data_(:,40);
 
x_diff = x_des - x_act;
y1_diff = y1_des - y1_act;
y2_diff = y2_des - y2_act;
z_diff = z_des - z_act;
pitch_diff = pitch_des - pitch_act;
roll_diff = roll_des - roll_act;
%% plot differences in pulse counts
clf

plot(z_diff)
hold on
plot(x_diff)
plot(y1_diff)
plot(y2_diff)
plot(pitch_diff)
plot(roll_diff)
hold off
legend('z','y1','y2','x','pitch','roll')
ylabel('error (pulse counts)')
improvePlot()
%% plot differences in mm
clf
pulse_to_mm = pi/4095*28.01;
pulse_to_deg = 180/2048;

figure(1)
plot(z_diff.*pulse_to_mm)
hold on
plot(y1_diff.*pulse_to_mm)
plot(y2_diff.*pulse_to_mm)
plot(x_diff.*pulse_to_mm)
legend('z','y1','y2','x')
hold off
ylabel('error (mm)')
improvePlot()

figure(2)
plot(pitch_diff.*pulse_to_deg)
hold on
plot(roll_diff.*pulse_to_deg)
hold off
legend('pitch','roll')
hold off
ylabel('error (deg)')
improvePlot()
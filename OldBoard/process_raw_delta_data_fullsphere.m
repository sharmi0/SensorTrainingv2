% processing raw data from test gantry
% modified 2/26/24, Sharmi

clear all;
close all;
clc;
addpath(genpath('matlab_helpers'));

%% import data and set output filename
% change this for each sensor!

% load dat
data_top = importdata('to_train/E8_top.txt'); % 90 deg mount
data_45 = importdata('to_train/E8_45.txt'); % 45 deg mount
data_front = importdata('to_train/E8_front.txt'); % flat mount

% sensor identifier to be included in output filenames
sensor_name = 'E8_3_10';
% should data be saved?
save_data = 0;

%temporary input data format
% [1,    2,  3,  4,  5-12,    13-16,    17,      18,    19,     20,      21,        22,        23,              24,         25,           26,          27,                28,         29,   30,      31,    32,     33,        34   ]
% [time, Fx, Fy, Fz, s1 - s8,   tof,  x_des,  y_des,  z_des, contact, pitch_des, roll_des, x_act pulse , y1_act pulse, y2_act pulse, z_act pulse, pitch_act pulse, roll_act pulse, x_act, y1_act, y2_act, z_act, pitch_act, roll_act]


% input data format
%(x, y1, y2, z, pitch, roll) = dxls (1, 2, 3, 4, 5, 6)
% [1,    2,  3,  4,  5-12,    13-16,    17,      18,    19,     20,      21,        22,        23,   24,     25,   26,        27,        28  ]
% [time, Fx, Fy, Fz, s1 - s8,   tof,  x_des,  y_des,  z_des, contact, pitch_des, roll_des, x_act , y1_act, y2_act, z_act, pitch_act, roll_act]

% Fx - Fz from ATI
% s1 - s8 are pressure sensors
% x, y, z are desired end-effector position
% binary flag for sensor contact
% desired pitch and roll
%x, y, z are in mm. pitch and roll are in rads
% pitch angle corresponds to rotx for sensor (theta)
% roll angle corresponds to roty for sensor (phi)


%% preliminary filtering and force transforms for each dataset

% filter out non-contact points
contact_inds_top = data_top(:,20)==1;
contact_transitions_top = diff(data_top(:,20));
ATI_offsets_inds_top = contact_transitions_top==1;

contact_inds_45 = data_45(:,20)==1;
contact_transitions_45 = diff(data_45(:,20));
ATI_offsets_inds_45 = contact_transitions_45==1;

contact_inds_front = data_front(:,20)==1;
contact_transitions_front = diff(data_front(:,20));
ATI_offsets_inds_front = contact_transitions_front==1;

% iterate through data and offset ATI forces
%group length = 10x group length in ellipsoid_training_traj code (bc 10
%timesteps at every point from generated traj)
transition_inds = find(ATI_offsets_inds_top==1,2);
gl = transition_inds(2) - transition_inds(1); 


gl_new_top = length(ATI_offsets_inds_top) - gl + 20 + 1;
gl_new_45 = length(ATI_offsets_inds_45) - gl + 20 + 1;
gl_new_front = length(ATI_offsets_inds_front) - gl + 20 + 1;

for ii=1:length(ATI_offsets_inds_top)
    %*************special case for new data*****************%
    if (ii == gl_new_top)
        data_top(ii+1:ii+gl-20-1,2:4) = data_top(ii+1:ii+gl-20-1,2:4) - data_top(ii,2:4);
    
    %***********************************************************%
    elseif (ATI_offsets_inds_top(ii)==1)
        data_top(ii+1:ii+gl-1,2:4) = data_top(ii+1:ii+gl-1,2:4) - data_top(ii,2:4);
    end
end
for ii=1:length(ATI_offsets_inds_45)
    %*************special case for new data*****************%
    if (ii == gl_new_45)
        data_45(ii+1:ii+gl-20-1,2:4) = data_45(ii+1:ii+gl-20-1,2:4) - data_45(ii,2:4);
    
    %***********************************************************%
    elseif (ATI_offsets_inds_45(ii)==1)
        data_45(ii+1:ii+gl-1,2:4) = data_45(ii+1:ii+gl-1,2:4) - data_45(ii,2:4);
    end
end
for ii=1:length(ATI_offsets_inds_front)
    %*************special case for new data*****************%
    if (ii == gl_new_front)
        data_front(ii+1:ii+gl-20-1,2:4) = data_front(ii+1:ii+gl-20-1,2:4) - data_front(ii,2:4);
    
    %***********************************************************%
    elseif (ATI_offsets_inds_front(ii)==1)
        aa = data_front(ii+1:ii+gl-1,2:4) - data_front(ii,2:4);
        data_front(ii+1:ii+gl-1,2:4) = data_front(ii+1:ii+gl-1,2:4) - data_front(ii,2:4);
    end
end

% contact_inds_top = data_top(1:120000,20)==1;
contact_data_top = data_top(contact_inds_top,:);
contact_data_45 = data_45(contact_inds_45,:);
% contact_inds_front = data_front(1:120000,20)==1;
contact_data_front = data_front(contact_inds_front,:);

% process data
pressure_offset_top = zeros(1,8); % mean(data_top(1:10,5:12),1); % average first 10 samples
pressure_readings_top = contact_data_top(:,5:12) - pressure_offset_top;
sensor_angles_top = [contact_data_top(:,33), contact_data_top(:,34)] + [deg2rad(-90),0]; % CHOOSE
sensor_angles_top = rad2deg(sensor_angles_top);
% ATI_offset_top = mean(data_top(1:10,2:4),1); % average first 10 samples
ATI_forces_top = contact_data_top(:,2:4); % - ATI_offset_top;
% ATI_forces_top = [ATI_forces_top(:,1) ATI_forces_top(:,2) -ATI_forces_top(:,3)]; %negate z for correct coordinate transform later on

pressure_offset_45 = zeros(1,8); % mean(data_45(1:10,5:12),1); % average first 10 samples
pressure_readings_45 = contact_data_45(:,5:12) - pressure_offset_45;
sensor_angles_45 = [contact_data_45(:,33), contact_data_45(:,34)] + [deg2rad(-45),0]; % CHOOSE
sensor_angles_45 = rad2deg(sensor_angles_45);
% ATI_offset_45 = mean(data_45(1:10,2:4),1); % average first 10 samples
ATI_forces_45 = contact_data_45(:,2:4); % - ATI_offset_45;
% ATI_forces_45 = [ATI_forces_45(:,1) ATI_forces_45(:,2) -ATI_forces_45(:,3)]; %negate z for correct coordinate transform later on

pressure_offset_front = zeros(1,8); % mean(data_front(1:10,5:12),1); % average first 10 samples
pressure_readings_front = contact_data_front(:,5:12) - pressure_offset_front;
sensor_angles_front = [contact_data_front(:,33), contact_data_front(:,34)] + [0, 0]; % CHOOSE
sensor_angles_front = rad2deg(sensor_angles_front);
% ATI_offset_front = mean(data_front(1:10,2:4),1); % average first 10 samples
ATI_forces_front = contact_data_front(:,2:4); % - ATI_offset_front;
% ATI_forces_front = [ATI_forces_front(:,1) ATI_forces_front(:,2) -ATI_forces_front(:,3)]; %negate z for correct coordinate transform later on


% transform forces for neural network ground truth
fingertip_forces_top = zeros(size(ATI_forces_top));
for ii=1:size(fingertip_forces_top,1)
%     R = roty(sensor_angles(ii,2),'deg')*rotx(sensor_angles(ii,1),'deg')*rotx(180,'deg');

%     R_sensorbase_contact_top = roty(sensor_angles_top(ii,2),'deg')*rotx(sensor_angles_top(ii,1),'deg');
    R_W_sensorbase_top = rotz(180,'deg'); %*rotz(180,'deg'); %*roty(90,'deg');
    R = R_W_sensorbase_top; %*R_sensorbase_contact_top;

    fingertip_forces_top(ii,:) = -(R*ATI_forces_top(ii,:)')'; % flip sign to get forces applied to sensor, not forces applied to bowl
end

fingertip_forces_45 = zeros(size(ATI_forces_45));
for ii=1:size(fingertip_forces_45,1)
%     R = roty(sensor_angles(ii,2),'deg')*rotx(sensor_angles(ii,1),'deg')*rotx(180,'deg');

%     R_sensorbase_contact_45 = roty(sensor_angles_45(ii,2),'deg')*rotx(sensor_angles_45(ii,1),'deg');
    R_W_sensorbase_45 = rotz(180,'deg'); %*rotz(180,'deg'); %*roty(45,'deg');
    R = R_W_sensorbase_45; %*R_sensorbase_contact_45;

    fingertip_forces_45(ii,:) = -(R*ATI_forces_45(ii,:)')'; % flip sign to get forces applied to sensor, not forces applied to bowl
end

fingertip_forces_front = zeros(size(ATI_forces_front));
for ii=1:size(fingertip_forces_front,1)
%     R = roty(sensor_angles(ii,2),'deg')*rotx(sensor_angles(ii,1),'deg')*rotx(180,'deg');

%     R_sensorbase_contact_front = roty(sensor_angles_front(ii,2),'deg')*rotx(sensor_angles_front(ii,1),'deg');
    R_W_sensorbase_front = rotz(180,'deg'); %*rotz(180,'deg')*roty(0,'deg');
    R = R_W_sensorbase_front; %*R_sensorbase_contact_front;

    fingertip_forces_front(ii,:) = -(R*ATI_forces_front(ii,:)')'; % flip sign to get forces applied to sensor, not forces applied to bowl
end

%% filter out edge angles on phi

% idxs_front = [];
% for ii=1:length(sensor_angles_front)
%     if (sensor_angles_front(ii,2)>30) %||(sensor_angles_front(ii,2)<-30)
%         idxs_front = [idxs_front, ii];
% %     elseif (sensor_angles_front(ii,1)>2)||(sensor_angles_front(ii,1)<-2)
% %         idxs_front = [idxs_front, ii];
%     end
% end
% 
% idxs_top = [];
% for ii=1:length(sensor_angles_top)
%     if (sensor_angles_top(ii,2)<-120) %||(sensor_angles_top(ii,2)>-60)
%         idxs_top = [idxs_top, ii];
% %     elseif (sensor_angles_top(ii,1)>2)||(sensor_angles_top(ii,1)<-2)
% %         idxs_top = [idxs_top, ii];
%     end
% end

% idxs_45 = [];
% for ii=1:length(sensor_angles_45)
%     if (sensor_angles_45(ii,1)>2)||(sensor_angles_45(ii,1)<-2)
%         idxs_45 = [idxs_45, ii];
%     end
% end

% pressure_readings_front(idxs_front,:) = [];
% contact_data_front(idxs_front,:) = [];
% sensor_angles_front(idxs_front,:) = [];
% ATI_forces_front(idxs_front,:) = [];
% fingertip_forces_front(idxs_front,:) = [];
% 
% pressure_readings_top(idxs_top,:) = [];
% contact_data_top(idxs_top,:) = [];
% sensor_angles_top(idxs_top,:) = [];
% ATI_forces_top(idxs_top,:) = [];
% fingertip_forces_top(idxs_top,:) = [];

% pressure_readings_45(idxs_45,:) = [];
% contact_data_45(idxs_45,:) = [];
% sensor_angles_45(idxs_45,:) = [];
% ATI_forces_45(idxs_45,:) = [];
% fingertip_forces_45(idxs_45,:) = [];


%% concatenate all data matrices

pressure_readings = [pressure_readings_top; pressure_readings_45; pressure_readings_front];
contact_data = [contact_data_top; contact_data_45; contact_data_front];
sensor_angles = [sensor_angles_top; sensor_angles_45; sensor_angles_front];
ATI_forces = [ATI_forces_top; ATI_forces_45; ATI_forces_front];
fingertip_forces = [fingertip_forces_top; fingertip_forces_45; fingertip_forces_front];



%% Initial plot of data
figure(1); clf; 
subplot(4,1,1); hold on;
plot(sensor_angles(:,1)');
plot(sensor_angles(:,2)'); 
xline(size(sensor_angles_top,1),'r:'); 
xline(size(sensor_angles_top,1)+size(sensor_angles_45,1),'r:'); hold off;
legend('Pitch','Roll'); xlim([1,size(contact_data,1)]);
title('Sensor Angles'); xlabel('Samples'); ylabel('Angle (deg)');

subplot(4,1,2); hold on; 
plot(ATI_forces(:,1)'); 
plot(ATI_forces(:,2)'); 
plot(ATI_forces(:,3)');
plot(zeros(1,size(fingertip_forces,1)),'k--'); 
xline(size(sensor_angles_top,1),'r:'); 
xline(size(sensor_angles_top,1)+size(sensor_angles_45,1),'r:'); 
hold off;
legend('Fx','Fy','Fz'); xlim([1,size(contact_data,1)]); ylim([-35,25]);
title('ATI Forces'); xlabel('Samples'); ylabel('Force (N)');

subplot(4,1,3); hold on; 
plot(fingertip_forces(:,1)'); 
plot(fingertip_forces(:,2)'); 
plot(fingertip_forces(:,3)');
plot(zeros(1,size(fingertip_forces,1)),'k--'); 
xline(size(sensor_angles_top,1),'r:'); 
xline(size(sensor_angles_top,1)+size(sensor_angles_45,1),'r:'); 
hold off;
legend('Ftx','Fty','Fnz'); xlim([1,size(contact_data,1)]); ylim([-35,25]);
title('Fingertip Forces'); xlabel('Samples'); ylabel('Force (N)');

subplot(4,1,4);
plot(pressure_readings); hold on;
xline(size(sensor_angles_top,1),'r:'); 
xline(size(sensor_angles_top,1)+size(sensor_angles_45,1),'r:'); hold off;
legend(' S0  ',' S1',' S2',' S3',' S4',' S5',' S6',' S7');
title('Pressure Readings'); xlim([1,size(contact_data,1)]); ylim([-100000,100000]); xlabel('Samples'); ylabel('Pressure (Pa)');

sgtitle('Raw Contact Data');





%% Plot distributions of pressure data, look for failing sensors
figure(2); clf; 
for i=1:8
    subplot(2,4,i);
    histogram(pressure_readings(:,i))
    title(['Sensor ',num2str(i-1)]);
end
sgtitle('Initial Pressure Distributions');

%% Plot the points from the top of the bowl to check for bad data
% figure(3); clf; 
% range_outer=[];
% for n=1:12
%    range_outer=[range_outer, 6*n*1960:6*n*1960+1960]; 
% end
% subplot(9,1,1);
% plot(sensor_angles(range_outer,1)'); hold on;
% plot(sensor_angles(range_outer,2)');
% legend('Theta','Phi');
% title('Angles');
% 
% for i=1:8
%     subplot(9,1,1+i);
%     plot(pressure_readings(range_outer,i));
%     title(['S',num2str(i)]);
% end
% sgtitle('Data from Patches near Bowl Edge');

%% Plot negative pressure values (if there are obvious sensor failures in figure 2 histogram)

% Prune out positive numbers
negative_pressures = pressure_readings;
negative_pressures(negative_pressures>0) = NaN;
% Plot distribution of negative numbers for each sensor
figure(4); clf; 
for i=1:8
    subplot(2,4,i);
    histogram(negative_pressures(:,i));
    title(['S',num2str(i-1)]);
end
sgtitle('Distributions of Negative Pressures');

%  Find negative outliers
neg_outliers = zeros(size(negative_pressures));
neg_outliers(negative_pressures<-50000) = 1;
any_neg_outliers = (sum(neg_outliers,2)>0); 

% Plot the the forces corresponding to these very negative numbers 
figure(5); clf; 
subplot(2,1,1); hold on;
plot((any_neg_outliers.*fingertip_forces(:,1))'); 
plot((any_neg_outliers.*fingertip_forces(:,2))'); 
plot((any_neg_outliers.*fingertip_forces(:,3))');
plot(zeros(1,size(fingertip_forces,1)),'k--'); 
hold off;
legend('Ftx','Fty','Fnz'); xlim([1,size(contact_data,1)]); ylim([-25,15]);
title('Fingertip Forces'); xlabel('Samples'); ylabel('Force (N)');
subplot(2,1,2);
plot(neg_outliers);
legend(' S0  ',' S1',' S2',' S3',' S4',' S5',' S6',' S7');
title('Pressure Reading < -50k');
xlim([1,size(contact_data,1)]); ylim([0,2]); xlabel('Samples'); ylabel('Yes / No');
sgtitle('Correlation of Negative Pressures');

%% Plot streaks of large negative values, indicating period of sensor failure (if there are large negative numbers in figure 2 histogram)
% TODO: fill this in if necessary based on lines 249-298 in process_raw_delta_data_1

%% Plot positive pressure values (if there are excessive fingertip saturations in figure 2 histogram)

% Prune out negative numbers
positive_pressures = pressure_readings;
positive_pressures(positive_pressures<0) = NaN;
% Plot distribution of positive numbers for each sensor
figure(6); clf; 
for i=1:8
    subplot(2,4,i);
    histogram(positive_pressures(:,i))
    title(['S',num2str(i-1)]);
end
sgtitle('Distributions of Positive Pressures');

%  Find positive outliers
pos_outliers = zeros(size(positive_pressures));
pos_outliers(positive_pressures>100000) = 1;
any_pos_outliers = (sum(pos_outliers,2)>0); 

% Plot the the forces corresponding to the saturated sensors 
figure(7); clf; 
subplot(2,1,1); hold on;
plot((any_pos_outliers.*fingertip_forces(:,1))'); 
plot((any_pos_outliers.*fingertip_forces(:,2))'); 
plot((any_pos_outliers.*fingertip_forces(:,3))');
plot(zeros(1,size(fingertip_forces,1)),'k--'); 
hold off;
legend('Ftx','Fty','Fnz'); xlim([1,size(contact_data,1)]); ylim([-25,15]);
title('Fingertip Forces'); xlabel('Samples'); ylabel('Force (N)');
subplot(2,1,2);
plot(pos_outliers);
legend(' S0  ',' S1',' S2',' S3',' S4',' S5',' S6',' S7');
title('Pressure Reading > 100k');
xlim([1,size(contact_data,1)]); ylim([0,2]); xlabel('Samples'); ylabel('Yes / No');
sgtitle('Correlation of Saturated Pressures');

%% Filter for excessively large z-forces (Fz > 0)

% Get indices of forces that are too large
Fz_max_thresh = -1.0; %2.5
Fz_too_large = (fingertip_forces(:,3)<Fz_max_thresh);
Fz_pressures = pressure_readings(Fz_too_large,:);

% Plot pressure reading distributions while z-force is too large
figure(8); clf; 
for i=1:8
    subplot(2,4,i);
    histogram(Fz_pressures(:,i))
    title(['S',num2str(i-1)]);
end
sgtitle('Distributions of Pressures for Fz > 2.5');

% Remove datapoints from the dataset
fingertip_forces(Fz_too_large,:) = [];
pressure_readings(Fz_too_large,:) = [];
sensor_angles(Fz_too_large,:) = [];
ATI_forces(Fz_too_large,:) = [];

%% Filter forces that are too large (Fx, Fy, and Fz)

Ft_mag_thresh = 20;
Fn_mag_thresh = 25;

Ft_too_large = (fingertip_forces(:,1)>Ft_mag_thresh)|(fingertip_forces(:,1)<-Ft_mag_thresh)|(fingertip_forces(:,2)>Ft_mag_thresh)|(fingertip_forces(:,2)<-Ft_mag_thresh);
Fn_too_large = (fingertip_forces(:,3)>Fn_mag_thresh);
forces_too_large = Ft_too_large|Fn_too_large;

% remove these points from the dataset
fingertip_forces(forces_too_large,:) = [];
pressure_readings(forces_too_large,:) = [];
sensor_angles(forces_too_large,:) = [];
ATI_forces(forces_too_large,:) = [];

%% Filter false positive contacts from the dataset (based on force magnitude)

% Get indices of points that may not be in contact
% Don't want to get too aggressive and prune out too many low-force points
non_contact=[];
cont_thresh = 0.5; % contact force threshold
for ii=1:length(fingertip_forces)
    if (sqrt(fingertip_forces(ii,:)*fingertip_forces(ii,:).') < cont_thresh)
        non_contact = [non_contact,ii];
    end
end
% Remove these indices
fingertip_forces(non_contact,:) = [];
pressure_readings(non_contact,:) = [];
sensor_angles(non_contact,:) = [];
ATI_forces(non_contact,:) = [];

%% Plot as a sanity check
figure(9); clf; 
subplot(4,1,1); hold on;
plot(sensor_angles(:,1)');
plot(sensor_angles(:,2)'); hold off;
legend('Pitch','Roll'); xlim([1,size(contact_data,1)]);
title('Sensor Angles'); xlabel('Samples'); ylabel('Angle (deg)');

subplot(4,1,2); hold on; 
plot(ATI_forces(:,1)'); 
plot(ATI_forces(:,2)'); 
plot(ATI_forces(:,3)');
plot(zeros(1,size(fingertip_forces,1)),'k--'); 
hold off;
legend('Fx','Fy','Fz'); xlim([1,size(contact_data,1)]); ylim([-35,25]);
title('ATI Forces'); xlabel('Samples'); ylabel('Force (N)');

subplot(4,1,3); hold on; 
plot(fingertip_forces(:,1)'); 
plot(fingertip_forces(:,2)'); 
plot(fingertip_forces(:,3)');
plot(zeros(1,size(fingertip_forces,2)),'k--'); 
hold off;
legend('Ftx','Fty','Fnz'); xlim([1,size(contact_data,1)]); ylim([-35,25]);
title('Fingertip Forces'); xlabel('Samples'); ylabel('Force (N)');

subplot(4,1,4);
plot(pressure_readings);
legend(' S0  ',' S1',' S2',' S3',' S4',' S5',' S6',' S7');
title('Pressure Readings'); xlim([1,size(contact_data,1)]); ylim([-100000,100000]); xlabel('Samples'); ylabel('Pressure (Pa)');

sgtitle('Final Contact Data');

%% Manual removal of bad data patches


% bad_idxs_D1r = [199774:214347, 230398:245739, 258326:274490, 281296:292337, 781653:793947, 810778:825679, 838237:854586];
% bad_idxs_D2r = [196658:211163, 227018:240461, 254072:265510, 273370:282103, 758004:770225, 789415:804734, 821782:836203];

% bad_idxs_D2r2 = [236828:246940, 272562:280271, 302410:307418];
% bad_idxs_D1r2 = [268081:278511, 295331:304775];

% bad_idxs_E3 = [266859:278197, 296128:305033];
% bad_idxs_E4 = [267901:276055, 294773:302318];

bad_idxs_E4 = [];
for ii=1:length(sensor_angles)
    if (sensor_angles(ii,2)>35) || (sensor_angles(ii,2)<-125)
        bad_idxs_E4 = [bad_idxs_E4,ii];
    end
end

% Remove these indices
fingertip_forces(bad_idxs_E4,:) = [];
pressure_readings(bad_idxs_E4,:) = [];
sensor_angles(bad_idxs_E4,:) = [];
ATI_forces(bad_idxs_E4,:) = [];

% Plot as a sanity check
figure(13); clf; 
subplot(4,1,1); hold on;
plot(sensor_angles(:,1)');
plot(sensor_angles(:,2)'); hold off;
legend('Theta','Phi'); xlim([1,size(contact_data,1)]);
title('Sensor Angles'); xlabel('Samples'); ylabel('Angle (deg)');

subplot(4,1,2); hold on; 
plot(ATI_forces(:,1)'); 
plot(ATI_forces(:,2)'); 
plot(ATI_forces(:,3)');
plot(zeros(1,size(fingertip_forces,1)),'k--'); 
hold off;
legend('Fx','Fy','Fz'); xlim([1,size(contact_data,1)]); ylim([-35,25]);
title('ATI Forces'); xlabel('Samples'); ylabel('Force (N)');

subplot(4,1,3); hold on; 
plot(fingertip_forces(:,1)'); 
plot(fingertip_forces(:,2)'); 
plot(fingertip_forces(:,3)');
plot(zeros(1,size(fingertip_forces,2)),'k--'); 
hold off;
legend('Ftx','Fty','Fnz'); xlim([1,size(contact_data,1)]); ylim([-35,25]);
title('Fingertip Forces'); xlabel('Samples'); ylabel('Force (N)');

subplot(4,1,4);
plot(pressure_readings);
legend(' S0  ',' S1',' S2',' S3',' S4',' S5',' S6',' S7');
title('Pressure Readings'); xlim([1,size(contact_data,1)]); ylim([-100000,0]); xlabel('Samples'); ylabel('Pressure (Pa)');

sgtitle('Final Final Contact Data');



%% Plot distributions of all data to set ranges for normalization

% Histograms
figure(10); clf; 
for i=1:8
    subplot(2,4,i);
    histogram(pressure_readings(:,i))
    title(['s',num2str(i-1)]);
end
sgtitle('Pressure Distributions before Normalization');
figure(11); clf; 
subplot(2,3,1);
histogram(fingertip_forces(:,1)); title('Fx');
subplot(2,3,2);
histogram(fingertip_forces(:,2)); title('Fy');
subplot(2,3,3);
histogram(fingertip_forces(:,3)); title('Fz');
subplot(2,3,4);
histogram(sensor_angles(:,1)); title('Theta');
subplot(2,3,5);
histogram(sensor_angles(:,2)); title('Phi');
sgtitle('Data Distributions before Normalization');

% Set ranges
pressure_min = round(min(pressure_readings),3,"significant");
pressure_max = round(max(pressure_readings),3,"significant");
force_min = round(min(fingertip_forces),2,"significant");
force_max= round(max(fingertip_forces),2,"significant");
angle_min=[ -135 -45];
angle_max=[ 45 45];

% Normalize data, making sure it goes from 0 to 1 to account for rounding errors
pressure_range_matrix=repmat(pressure_max-pressure_min,size(pressure_readings,1),1);
normalized_pressure_readings = (pressure_readings - pressure_min)./pressure_range_matrix;
normalized_pressure_readings(normalized_pressure_readings>1) = 1;
normalized_pressure_readings(normalized_pressure_readings<0) = 0;

force_range_matrix=repmat(force_max-force_min,size(fingertip_forces,1),1);
normalized_fingertip_forces = (fingertip_forces - force_min)./force_range_matrix;
normalized_fingertip_forces(normalized_fingertip_forces>1) = 1;
normalized_fingertip_forces(normalized_fingertip_forces<0) = 0;

% normalized_fingertip_forces(isnan(normalized_fingertip_forces)) = 0.5;


angle_range_matrix=repmat(angle_max-angle_min,size(sensor_angles,1),1);
normalized_sensor_angles = (sensor_angles - angle_min)./angle_range_matrix;
normalized_sensor_angles(normalized_sensor_angles>1) = 1;
normalized_sensor_angles(normalized_sensor_angles<0) = 0;

% Another set of histograms
figure(12); clf; 
for i=1:8
    subplot(2,4,i);
    histogram(normalized_pressure_readings(:,i))
    title(['s',num2str(i-1)]);
end
sgtitle('Pressure Distributions after Normalization');
figure(13); clf; 
subplot(2,3,1);
histogram(normalized_fingertip_forces(:,1)); title('Fx');
subplot(2,3,2);
histogram(normalized_fingertip_forces(:,2)); title('Fy');
subplot(2,3,3);
histogram(normalized_fingertip_forces(:,3)); title('Fz');
subplot(2,3,4);
histogram(normalized_sensor_angles(:,1)); title('Theta');
subplot(2,3,5);
histogram(normalized_sensor_angles(:,2)); title('Phi');
sgtitle('Data Distributions after Normalization');

%% Save normalization parameters, training dataset, testing dataset

% Save data as csv: forces, angles, sensor readings
NN_data = [normalized_fingertip_forces, normalized_sensor_angles, normalized_pressure_readings]; 
num_samples = size(fingertip_forces,1);
train = 1:num_samples;
test = randsample(num_samples, floor(num_samples/20));
train(test) = []; % remove testing samples from training set

save_data = 1;
if (save_data==1)
    if not(isfolder(['processed_data/',sensor_name]))
        mkdir(['processed_data/',sensor_name]);
    end
    save(['processed_data/',sensor_name,'/norm_params.mat'],'test','force_min','force_max','pressure_min','pressure_max','angle_min','angle_max');
    writematrix(NN_data,['processed_data/',sensor_name,'/full_dataset.csv']);
    writematrix(train,['processed_data/',sensor_name,'/train_full_dataset.csv']); 
    writematrix(test,['processed_data/',sensor_name,'/test_full_dataset.csv']); 
end

disp('Done!')
close all; clear all; clc;
addpath(genpath('matlab_helpers'))

%% set up data saving
save_data = 0;
savename = 'trajectories/ellipsoid_newgantry_trajtest.csv';
%% generate asterisk pattern at origin

% asterisk parameters
dx = 0.5; % distance in mm between points along each ray of the asterisk
N1 = 1; % number of pts along each ray of the asterisk (not including center point)
N2 = 2; % number of equally spaced rays of the asterisk

dz0 = 0; % initial compression distance for contact
dz = -0.5; % distance in mm between stacked asterisk patterns
N3 = 1; %3; % number of stacked asterisk patterns in the group

dw = 10; % starting and ending height from center of asterisk pattern - gantry will move up this distance between every asterisk group

% single asterisk group
rl = 2*N1; % ray length
ray = zeros(3,rl);
for ii=1:N1 % construct ray
    ray(:,ii) = [dx*ii;0;0]; % do we need to go out along ray and come back??
    ray(:,ii+N1) = [dx*(N1-ii);0;0];
end
pl = 1+rl*N2; % pattern length
pattern = zeros(3,pl);
for ii=1:N2 % rotate rays to form pattern
    rot_mat = [cosd(ii*(360/N2)), -sind(ii*(360/N2)), 0; sind(ii*(360/N2)), cosd(ii*(360/N2)), 0; 0,0,1]; % hardcode to avoid rotz() function issues
    pattern(:,(ii-1)*rl+2:ii*rl+1) = rot_mat*ray;
end
gl = 2+N3*pl; % group length
group = [zeros(3,2+N3*pl); ones(1,2+N3*pl)];
for ii=1:N3 % group consists of patterns at different heights
    group(1:3,(ii-1)*pl+2:ii*pl+1) = [pattern(1:2,:); pattern(3,:)+dz*(ii-1)+dz0];
end
group(1:4,[1,gl]) = [0,0;0,0;dw,dw;0,0]; % include start and end points away from contact, zero contact flags 
% group column is [x,y,z,contact_flag]

%% calculate contact points

% vertical offset
z_offset = -10  ; % z offset of delta : a lower value means the mount will be lower

% nominal ATI surface vector
ati_nominal = [0,0,12.6 + 15]';  %12.6mm is from the point of rotation to top of bare ati | 15mm is height of pedestal

%mount offset in x and y
y_mount_offset =  10;  %mm    offsets traj in y (+y is towards back when facing test setup)
x_mount_offset =  0; %mm    offsets traj in x (+x is towards right when facing test setup)


% surface parameters
a = 10.7; %original = 10.7
b = 9; %original = 9
c = 6.35; %original = 6.35
r = 1;

% % original contact point parameters
% azimuth_range = [-pi, pi]; % centered at zero
% polar_range = [0, -pi/4] + pi; % centered at pi
% ma = 20-1; % number of azimuthal angles
% mp = 10; % number of polar angle

% contact point parameters - testing
azimuth_range = [-pi, pi]; % centered at zero
polar_range = [0, -pi/4.5] + pi; % centered at pi
ma = 6-1; % number of azimuthal angles
mp = 4; % number of polar angles

data_threshold = 1E-10;

% just for sensor surface
n = 60; %number of points used to visualize sensor surface
t_range = linspace(pi, pi/2, n); 
t = t_range'*ones(1,n); %theta-polar
p = [linspace(0, pi, n/2) linspace(pi, 2*pi, n/2)];%phi-azimuthal
y = a*r*sin(t).*cos(p); y(y>6.539) = 6.539; y = y'; 
x = b*r*sin(t).*sin(p); x(x>6.052) = 6.052; x(x<-6.052) = -6.052; x = x';
z = c*r*cos(t); z = z';
sensor_surface = [];
for ii = 1:1:n
    z_layer = [x(:,ii) y(:,ii) z(:,ii) t(ii,:)' p']; %vertical layers of ellipsoid
    sensor_surface = cat(1,sensor_surface,z_layer);
end

% calculating contact points
tc_range = linspace(polar_range(1), polar_range(2), mp);
tc = tc_range'*ones(1,ma);
pc = linspace(azimuth_range(1), azimuth_range(2), ma);
yc = a*r*sin(tc).*cos(pc); yc = yc';
xc = b*r*sin(tc).*sin(pc); xc = xc';
zc = c*r*cos(tc); zc = zc';
contact_points = [];
for ii = 1:1:mp
    if ii==1
        z_layer = [xc(1,ii) yc(1,ii) zc(1,ii) tc(ii,1)' pc(1)']; %vertical layers of ellipsoid
    else
        z_layer = [xc(2:end,ii) yc(2:end,ii) zc(2:end,ii) tc(ii,2:end)' pc(2:end)'];
    end
%     z_layer = [xc(:,ii) yc(:,ii) zc(:,ii) tc(ii,:)' pc']; 
    contact_points = cat(1,contact_points,z_layer);
end


% calculate other data, like the normals and dynamixel angles
tc = contact_points(:,4); 
pc = contact_points(:,5);
ny = 2*a*sin(tc).*cos(pc)/a^2; 
nx = 2*b*sin(pc).*sin(tc)/b^2; 
nz = 2*c*cos(tc)/c^2; 

normals = [nx ny nz];
normals = normals./(vecnorm(normals'))'; %if deep learning toolbox is installed--> normals = normr(normals)
ati_normals = -normals;

%calculate the pitch and roll angle from normal
npitch_nroll = calc_pr(ati_normals);

fp_origin = contact_points(1,1:3); %Center contact point vector
% calculate the offsets due to the change in contact point on the sensor
p_offsets = fp_origin-contact_points(:,1:3); 
p_offsets = [p_offsets(:,1), p_offsets(:,2), p_offsets(:,3)]; % 



% compile contact point data
%[x, y, z, t, p, nx, ny, nz, npitch, nroll, x_offset, y_offset, z_offset]
contact_data = [contact_points normals npitch_nroll p_offsets];
contact_data(abs(contact_data)<data_threshold) = 0; %rounds small numbers to zero
%% 
%throw out bad contact points
%to avoid colliding non-urethane parts of the sensor, we don't want contact points within a
%certain angle

cdata = size(contact_data);
cdata = cdata(1);
mask = ones(cdata,1);


for jj=1:cdata
    t_cur = contact_data(jj,4);
    p_cur = contact_data(jj,5);

    %to avoid TOF sensors on the back
    if t_cur - pi < -pi/10
        if abs(p_cur) < 5*pi/20 && abs(p_cur) > 3*pi/20
            mask(jj) = 0;
        end
    end

    %to account for more open surface area on the front and back and throw
    %out points on the sides of the ellipsoid
    if t_cur - pi  < -pi/6
        if abs(p_cur) < 2*pi/3 && abs(p_cur) > pi/4
            mask(jj) = 0;
        end
    end

    %to account for more surface area on the front of the ellipsoid
    %compared to other parts on the shape
    if t_cur - pi  < -pi/4.5
        if abs(p_cur) < pi - 5*pi/6
            mask(jj) = 0;
        end
    end

%     %****************** just for testing purposes - delete
%     if t_cur - pi  > -pi/7 && t_cur - pi  < -pi/40
% 
%             mask(jj) = 0;
%     end
    
end

contact_data = contact_data(logical(mask),:);
contact_points = contact_points(logical(mask),:);
ati_normals = ati_normals(logical(mask),:);

%the ellipsoid extends further in the front than the back, throw out
%contact points in the back after a certain polar angle 




%% plot contact points
clf
figure(1);
plot3(sensor_surface(:,1),sensor_surface(:,2),sensor_surface(:,3),'.'); hold on;
plot3(contact_points(:,1),contact_points(:,2),contact_points(:,3),'o'); hold on;
quiver3(contact_points(:,1),contact_points(:,2),contact_points(:,3),ati_normals(:,1),ati_normals(:,2),ati_normals(:,3),"AutoScale","off");
grid on;
axis("equal");
xlabel("X"); ylabel("Y"); zlabel("Z");

%% transform asterisk patterns

% for each contact point
% built rotation and translation
% transform asterisk by rotations and translation
% append transformed asterisk and bonus data to final data array

num_contacts = size(contact_data,1);
% group for each contact, so initialize that array?
asterisk_data = [];
new_group = [];
phi_prev = -pi;

for ii=1:num_contacts
    % calculate rotation
    pitch = contact_data(ii,9);
    roll = contact_data(ii,10);
    R_i = roty(roll)*rotx(pitch); %rotation matrix for given contact point
    % calculate translation
    p_i = contact_data(ii,11:13)'; % use p_offsets
    
    ati_new = R_i*ati_nominal; %rotate ati mount surface
    ati_surface_offset = ati_new - ati_nominal; %offsets due to rotated ati mount surface
    p_i = p_i + ati_surface_offset + [x_mount_offset,y_mount_offset,z_offset]'; %combine offsets
    % modify contact data now that full offsets are calculated
    contact_data(ii,11:13) = p_i';

%     disp([contact_data(ii,11:13), ati_surface_offset', pitch, roll])

    new_group = R_i*group(1:3,:) + p_i; % transform asterisk points

    %to avoid collisions, create intermediate positions for problem parts
    %of the trajectory
    
    theta = contact_data(ii,4);
    phi = contact_data(ii,5);
    
    if ii > 1
        phi_prev = contact_data(ii-1,5);
    end

    %if points along phi are being skipped - this is super sketch, recheck
    %the extra z offset if the polar range is increased
    if phi - phi_prev > pc(2) - pc(1) || phi_prev - phi > pc(2) - pc(1)
        %bring the end effector up  to avoid the ATI mount
        center_group = [x_mount_offset,y_mount_offset,z_offset + 20,0,0,0,-10,theta,phi,0,0,-1,0,0,y_mount_offset,y_mount_offset,z_offset]; %go to origin at a high z value
        postcontact_prev = [asterisk_data(1:2,end); z_offset + 20;asterisk_data(4:17,end)]; %from the prev position, go up in z
        precontact = [new_group(1:2,1); z_offset + 20; 0; contact_data(ii,:)']; %reach the new position but at a higher z, before coming down
        asterisk_data = [asterisk_data,postcontact_prev,center_group',precontact];

    end

    % add other data
    new_group = [new_group; group(4,:); repmat(contact_data(ii,:)',1,gl)]; % contact flags are the same for each group, contact data are the same for each group
    asterisk_data = [asterisk_data, new_group];
end

%% plot all of the asterisk points
figure(2); clf;
hold on;

start = 1 + gl;
stop = start + gl-1; 

% disp(asterisk_data(13:17,start+1)')

scatter3(asterisk_data(1,start:stop),asterisk_data(2,start:stop),asterisk_data(3,start:stop),'.');
scatter3(asterisk_data(1,start+1), asterisk_data(2,start+1), asterisk_data(3,start+1),'og');
quiver3(0,0,0,50,0,0,'r');
quiver3(0,0,0,0,50,0,'g');
quiver3(0,0,0,0,0,50,'b');
axis equal
grid on
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Full Training Trajectory');

%% store data
asterisk_data = [zeros(17,1), asterisk_data, zeros(17,1)];
asterisk_data(3,1) = 10;
asterisk_data(3,end) = 10;
asterisk_data = asterisk_data';

% choose to save data or not
if save_data
    writematrix(asterisk_data,savename) 
end




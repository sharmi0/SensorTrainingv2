import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

# set up data saving
save_data = 1
check_data = 1 #flag to check whether trajectory will be within bounds
top = 1 # flag for spherical traj generation top orientation
savename = 'trajectories/pyspherical_newgantry_tester.csv'

#********************* function definitions for trajectory*****************************
'''takes in unit normals and returns pitch and roll euler angles'''    
def calc_pr(rowVec):
    # project to zy plane
    r,_ = rowVec.shape
    proj_zy = np.hstack((np.zeros((r,1)), rowVec[:,1:3]))
    x_sgn = np.sign(rowVec[:,0])
    
    # normalize the projected vector
    proj_zy = proj_zy / np.linalg.norm(proj_zy, axis=1)[:, np.newaxis]
    
    # calculate pitch
    pitch = -np.arctan2(proj_zy[:,1], proj_zy[:,2])  # per hardware configuration, rotation about x axis has to be flipped
    
    # calculate roll
    roll = x_sgn * np.arccos(np.diag(proj_zy @ rowVec.T).real)
    
    prMat = np.column_stack((pitch, roll))
    return prMat


def rotx(t, deg=None):
    if deg == 'deg':
        t = t * np.pi / 180

    ct = np.cos(t)
    st = np.sin(t)

    if not isinstance(t, complex):
        if abs(st) < 1e-15:
            st = 0
        if abs(ct) < 1e-15:
            ct = 0

    R = [
        [1, 0, 0],
        [0, ct, -st],
        [0, st, ct]
    ]
    return R

def roty(t, deg=None):
    if deg == 'deg':
        t = t * np.pi / 180

    ct = np.cos(t)
    st = np.sin(t)

    if not isinstance(t, complex):
        if abs(st) < 1e-15:
            st = 0
        if abs(ct) < 1e-15:
            ct = 0

    R = [
        [ct, 0, st],
        [0, 1, 0],
        [-st, 0, ct]
    ]
    return R

##********************* function definitions for gantry*****************************

# ANSI escape codes for text color
class Color:
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'

#bounds of dynamixels
x_lims = [0, 2048]
y1_lims = [500, 3900]
y2_lims = [145, 3600]
z_lims = [1100, 3640]
ati_pitch_lims = [1000, 2800]
ati_roll_lims = [1100, 2565]
pitch_d = 28.01 #mm

r2p = lambda rad: round(rad * 2048 / np.pi) + 2048   # radians to pulse counts
p2r = lambda pulse: (pulse - 2048) * np.pi / 2048    # pulse counts to radians

'''convert from position value to dxl pulse counts **X**'''    
def position_to_pulses_x(position):
    max_counts = 4095
    return round(position * (max_counts/(np.pi*pitch_d))) + 1997

'''convert from position value to dxl pulse counts **Y1**'''    
def position_to_pulses_y1(position):
    max_counts = 4095
    return 2098 - round(position * (max_counts/(np.pi*pitch_d)))

'''convert from position value to dxl pulse counts **Y2**'''    
def position_to_pulses_y2(position):
    max_counts = 4095
    return round(position * (max_counts/(np.pi*pitch_d))) + 1992

'''convert from position value to dxl pulse counts **Z**'''    
def position_to_pulses_z(position):
    max_counts = 4095
    return round(position * (max_counts/(np.pi*pitch_d))) + 2060 #set z offset to be such that 0 is where the sensor touches the pedestal

'''convert from pulse counts to position values **X** '''    
def pulses_to_position_x(counts):
    max_counts = 4095
    return (counts-1997) * ((np.pi)/max_counts) * pitch_d

'''convert from pulse counts to position values **Y1**'''    
def pulses_to_position_y1(counts):
    max_counts = 4095
    return (2098-counts) * ((np.pi)/max_counts) * pitch_d

'''convert from pulse counts to position values **Y2**'''    
def pulses_to_position_y2(counts):
    max_counts = 4095
    return (counts-1992) * ((np.pi)/max_counts) * pitch_d

'''convert from pulse counts to position values **Z**'''    
def pulses_to_position_z(counts):
    max_counts = 4095
    return (counts-2060) * ((np.pi)/max_counts) * pitch_d

# generate asterisk pattern at origin

# asterisk parameters
dx = 0.5  # distance in mm between points along each ray of the asterisk
N1 = 2  # number of pts along each ray of the asterisk (not including center point)
N2 = 2  # number of equally spaced rays of the asterisk
dz0 = 0  # initial compression distance for contact
dz = -1  # distance in mm between stacked asterisk patterns
N3 = 3  # number of stacked asterisk patterns in the group
dw = 10  # starting and ending height from center of asterisk pattern - gantry will move up this distance between every asterisk group


# single asterisk group
rl = 2 * N1  # ray length
ray = np.zeros((3, rl))
for ii in range(1, N1 + 1):
    ray[:, ii - 1] = np.array([dx * ii, 0, 0])
    ray[:, ii + N1 - 1] = np.array([dx * (N1 - ii), 0, 0])

pl = 1 + rl * N2  # pattern length = center point (1) + ray length * number of rays
pattern = np.zeros((3, pl))

for ii in range(1, N2 + 1):
    angle = ii * (360 / N2)
    rot_mat = np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle)), 0],
                        [np.sin(np.radians(angle)), np.cos(np.radians(angle)), 0],
                        [0, 0, 1]])
    rot_mat[np.abs(rot_mat) < 1e-10] = 0

    aa = np.dot(rot_mat, ray)
    pattern[:, (ii - 1) * rl + 1:ii * rl + 1] = np.matmul(rot_mat, ray)
gl = 2 + N3 * pl  # group length = starting and ending points (2) + layers * pattern length
group = np.vstack((np.zeros((3, 2 + N3 * pl)), np.ones((1, 2 + N3 * pl))))

for ii in range(1, N3 + 1):
    group[0:3, (ii - 1) * pl + 1:ii * pl + 1] = np.vstack((pattern[0:2, :], pattern[2, :] + dz * (ii - 1) + dz0))

group[0:4, [0, gl - 1]] = np.array([[0, 0], [0, 0], [dw, dw], [0, 0]])  # include start and end points away from contact, zero contact flags
# group column is [x, y, z, contact_flag]



# calculate contact points
# vertical offset
z_offset = -1  # z offset of delta : a lower value means the mount will be lower
# nominal ATI surface vector
ati_nominal = np.array([0, 0, 12.6 + 15]).reshape(-1, 1)  # 12.6 is from the point of rotation to top of bare ati / 15 is height of pedestal
# mount offset in x and y
y_offset = 0  # mm    offsets traj in y (+y is towards back when facing test setup)
x_offset = 0  # mm    offsets traj in x (+x is towards right when facing test setup
# surface parameters
a = 10
b = 10
c = 10
r = 1
# contact point parameters - testing
azimuth_range = [-np.pi, np.pi]  # centered at zero
polar_range = [0 + np.pi , -np.pi/4 + np.pi] # centered at pi
ma = 6 - 1  # number of azimuthal angles
mp = 2  # number of polar angles
data_threshold = 1E-10
# just for sensor surface
n = 60  # number of points used to visualize sensor surface
t_range = np.linspace(np.pi, np.pi/2, n)
t = np.tile(t_range.reshape(-1, 1), (1, n))  # theta-polar
p = np.concatenate((np.linspace(0, np.pi, n//2), np.linspace(np.pi, 2*np.pi, n//2)))  # phi-azimuthal
y = (a*r*np.sin(t)*np.cos(p)).T
x = (b*r*np.sin(t)*np.sin(p)).T
z = (c*r*np.cos(t)).T
sensor_surface = np.empty((0, 5))
for ii in range(n):
    z_layer = np.column_stack((x[:, ii], y[:, ii], z[:, ii], t[ii, :], p))  # Create a vertical layer of ellipsoid
    sensor_surface = np.concatenate((sensor_surface, z_layer), axis=0)  # Concatenate the layer to sensor_surface
# calculating contact points
tc_range = np.linspace(polar_range[0], polar_range[1], mp)
tc = np.tile(tc_range.reshape(-1, 1), (1, ma))
pc = np.linspace(azimuth_range[0], azimuth_range[1], ma)
yc = (a*r*np.sin(tc)*np.cos(pc)).T
xc = (b*r*np.sin(tc)*np.sin(pc)).T
zc = (c*r*np.cos(tc)).T
contact_points = np.empty((0, 5))
for ii in range(mp):
    if ii == 0:
        z_layer = np.array([xc[0, ii], yc[0, ii], zc[0, ii], tc[ii, 0], pc[0]])  # vertical layers of ellipsoid
    else:
        z_layer = np.column_stack((xc[1:, ii], yc[1:, ii], zc[1:, ii], tc[ii, 1:], pc[1:]))
    contact_points = np.vstack((contact_points, z_layer))
# calculate other data, like the normals and dynamixel angles
tc = contact_points[:, 3]
pc = contact_points[:, 4]
ny = 2*a*np.sin(tc)*np.cos(pc)/a**2
nx = 2*b*np.sin(pc)*np.sin(tc)/b**2
nz = 2*c*np.cos(tc)/c**2
normals = np.column_stack((nx, ny, nz))
normals = normals/(np.linalg.norm(normals, axis=1).reshape(-1, 1))  # if deep learning toolbox is installed--> normals = normr(normals)
ati_normals = -normals
# calculate the pitch and roll angle from normal
npitch_nroll = calc_pr(ati_normals)
fp_origin = contact_points[0, :3]  # Center contact point vector
# calculate the offsets due to the change in contact point on the sensor
p_offsets = fp_origin - contact_points[:, :3]
p_offsets = np.column_stack((p_offsets[:, 0], p_offsets[:, 1], p_offsets[:, 2]))
# compile contact point data
# [x, y, z, t, p, nx, ny, nz, npitch, nroll, x_offset, y_offset, z_offset]
contact_data = np.column_stack((contact_points, normals, npitch_nroll, p_offsets))
contact_data[np.abs(contact_data) < data_threshold] = 0  # rounds small numbers to zero
# throw out bad contact points
# to avoid colliding non-urethane parts of the sensor, we don't want contact points within a
# certain angle
if top == 1:
    cdata = contact_data.shape[0]
    mask = np.ones((cdata, 1))
    for jj in range(cdata):
        t_cur = contact_data[jj, 3]
        p_cur = contact_data[jj, 4]
        # to account for more surface area on the front of the ellipsoid
        # compared to other parts on the shape
        if t_cur - np.pi < -np.pi/7:
            if np.abs(p_cur) > 7*np.pi/8:
                mask[jj] = 0
    contact_data = contact_data[mask.flatten().astype(bool), :]
    contact_points = contact_points[mask.flatten().astype(bool), :]
    ati_normals = ati_normals[mask.flatten().astype(bool), :]
    # the ellipsoid extends further in the front than the back, throw out

# plot contact points
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

# Plot sensor surface
ax.plot(sensor_surface[:, 0], sensor_surface[:, 1], sensor_surface[:, 2], '.', label='Sensor Surface', markersize=5)

# Plot contact points
ax.plot(contact_points[:, 0], contact_points[:, 1], contact_points[:, 2], 'o', label='Contact Points')

# Plot normals
ax.quiver(contact_points[:, 0], contact_points[:, 1], contact_points[:, 2],
          ati_normals[:, 0], ati_normals[:, 1], ati_normals[:, 2], length=1)

ax.grid(True)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_aspect('equal')

plt.show()

# transform asterisk patterns
num_contacts = contact_data.shape[0]
asterisk_data = []
new_group = []
for ii in range(num_contacts):
    pitch = contact_data[ii,8]
    roll = contact_data[ii,9]
    R_i = np.dot(roty(roll), rotx(pitch))
    p_i = contact_data[ii,10:13].reshape(-1,1)
    ati_new = np.dot(R_i, ati_nominal)
    ati_surface_offset = ati_new - ati_nominal
    p_i = p_i + ati_surface_offset + np.array([x_offset, y_offset, z_offset]).reshape(-1,1)
    contact_data[ii,10:13] = p_i.T
    new_group = np.dot(R_i, group[0:3,:]) + p_i
    repeated_data = np.tile(contact_data[ii].reshape((13,1)), (1,gl))
    new_group = np.concatenate((new_group, group[3, :].reshape(1, -1), repeated_data), axis=0)
    asterisk_data.append(new_group)
asterisk_data = np.concatenate(asterisk_data, axis=1)

# plot all asterisk points
start = 1 + gl
stop = start + gl - 1

fig = plt.figure(2)
plt.clf()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(asterisk_data[0, start:stop], asterisk_data[1, start:stop], asterisk_data[2, start:stop], marker='.')
ax.scatter(asterisk_data[0, start+1], asterisk_data[1, start+1], asterisk_data[2, start+1], marker='o', color='g')
ax.quiver(0, 0, 0, 50, 0, 0, color='r')
ax.quiver(0, 0, 0, 0, 50, 0, color='g')
ax.quiver(0, 0, 0, 0, 0, 50, color='b')
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('Full Training Trajectory')
ax.axis('equal')
ax.grid(True)
plt.show()

# store data
asterisk_data = np.hstack((np.zeros((17,1)), asterisk_data, np.zeros((17,1))))
asterisk_data[2,0] = 10
asterisk_data[2,-1] = 10
asterisk_data = asterisk_data.T

if check_data:
    err_pts = []
    for i in asterisk_data:
        # calculate dynamixel positions
        #traj data = x, y, z, pitch, roll
        commands_x = position_to_pulses_x(i[0])
        commands_y1 = position_to_pulses_y1(i[1])
        commands_y2 = position_to_pulses_y2(i[1])
        commands_z = position_to_pulses_z(i[2])
        commands_pitch = r2p(float(i[12]))
        commands_roll = r2p(float(i[13]))

        # Construct the list of commands
        commands = [commands_x, commands_y1, commands_y2, commands_z, commands_pitch, commands_roll]

        # Check if any command is outside gantry limits
        out_of_bounds = [False] * 6  # Initialize list to track out-of-bounds values
        if commands_y1 < y1_lims[0] or commands_y1 > y1_lims[1]:
            out_of_bounds[1] = True
        if commands_y2 < y2_lims[0] or commands_y2 > y2_lims[1]:
            out_of_bounds[2] = True
        if commands_z < z_lims[0] or commands_z > z_lims[1]:
            out_of_bounds[3] = True
        if commands_pitch < ati_pitch_lims[0] or commands_pitch > ati_pitch_lims[1]:
            out_of_bounds[4] = True
        if commands_roll < ati_roll_lims[0] or commands_roll > ati_roll_lims[1]:
            out_of_bounds[5] = True

        if any(out_of_bounds):
            for j, cmd in enumerate(commands):
                if out_of_bounds[j]:
                    print(f"{Color.RED}{cmd}{Color.END}", end=' ')
                else:
                    print(f"{cmd}", end=' ')
            print()
            err_pts.append([commands_x, commands_y1, commands_y2, commands_z, commands_pitch, commands_roll])
    if len(err_pts) == 0:
        print("NO BAD POINTS")
    else:
        print("BAD POINTS")

header = [f'N1: {N1}', f'N2: {N2}', f'N3: {N3}', \
          f'dx: {dx}', f'dz0: {dz0}', f'dz: {dz}', f'dw: {dw}', \
          f'ma: {ma}', f'mp: {mp}', f'azimuth range: {azimuth_range[0]} to {azimuth_range[1]}', f'polar range: {polar_range[0]} to {polar_range[1]}', \
          f'x_offset: {x_offset}', f'y_offset: {y_offset}', f'z_offset: {z_offset}']


delimiter = ", "
header_str = delimiter.join(header)

# Calculate the difference in lengths between header and asterisk data columns
# header_len = len(header)
# asterisk_len = asterisk_data.shape[1]
# padding_width = asterisk_len - header_len

# # Pad the header with zeros
# padded_header = np.pad(header, (0, padding_width), mode='constant', constant_values='0')
# print(padded_header)

# asterisk_data_with_header = np.vstack([padded_header, asterisk_data])

if save_data:
    np.savetxt(savename, asterisk_data, delimiter=',',header=header_str)

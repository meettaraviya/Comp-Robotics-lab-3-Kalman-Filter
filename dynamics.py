import numpy as np

# map height
H = 500.0
# map width
W = 500.0

fps = 60.0
dt = 1/fps


# 2x2
Q = None

# 4x4
R = None

# Adrian
def f_transition(s, u, w_noise):
	w_noise_l, w_noise_r = w_noise
	omega_l, omega_r = u
	x, y, theta = s

	# effective inputs after wheel slippage/noise
	omega_l, omega_r = omega_l + w_noise_l, omega_r + w_noise_r
	print(omega_l, omega_r)
	s_ = x_, y_, theta_
	return s_

# Andrew
def h_observation(s, u, v_noise):
	'''
	Return a tuple of observations.
	Observations are noisy sensor measurements taken at state s.
	The added noise is determined based on the v_noise input.
	'''
	# extract parameters
	x, y, theta = s
	omega_l, omega_r = u
	front_noise, right_noise, theta_noise, omega_noise = v_noise
	
	# uses get_wall_distances
	distances = get_wall_distances(s)	# d_r, d_t, d_l, d_b
	i = get_front_wall(s)				# returns index of front wall

	front_o = distances[i]
	right_o = distances[(i-1)%4]		# right wall is at index (i-1) mod 4

	# state heading is observed heading (before adding noise)
	theta_o = theta

	# compute omega based on current action
	robot_width = 90.0 #mm
	wheel_radius = 25.0 #mm
	v_l = (omega_l/60.0) * (3.14*2*wheel_radius)
	v_r = (omega_r/60.0) * (3.14*2*wheel_radius)

	omega_o = (v_r - v_l)/robot_width
	
	# Derivation:
		# omega_r == omega_l 
		# v_r/rad_r == v_l/rad_l
		# rad_r - rad_l == robot_width
		# Choose counterclockwise rotations to be positive
		
		# v_l/rad_l == v_r/(robot_width + rad_l)
		# v_l*(robot_width + rad_l) == v_r * rad_l
		# v_l*robot_width == v_r*rad_l - v_l*rad_l
		# (v_l/rad_l) = (v_r-v_l)/robot_width

	# add noise
	front_o += front_noise
	right_o += right_noise
	theta_o += theta_noise
	omega_o += omega_noise

	o = front_o, right_o, theta_o, omega_o
	return o

# Andrew
def generate_v_noise(s, u):
	x, y, theta = s
	omega_l, omega_r = u
	laser_accuracy = 0.04
	gyro_accuracy = 0.03
	magn_accuracy = 0.625

	distances = get_wall_distances(s)	# d_r, d_t, d_l, d_b
	i = get_front_wall(s)				# returns index of front wall
	d_front = distances[i]
	d_right = distances[(i-1)%4]

	front_noise = np.random.normal(0, laser_accuracy*d_front, 1)[0]
	right_noise = np.random.normal(0, laser_accuracy*d_right, 1)[0]

	robot_width = 90.0 #mm
	wheel_radius = 25.0 #mm
	v_l = (-60/60.0) * (3.14*2*wheel_radius)
	v_r = (60/60.0) * (3.14*2*wheel_radius)
	max_omega = (v_r - v_l)/robot_width

	omega_noise = np.random.normal(0, gyro_accuracy*max_omega, 1)[0]
	theta_noise = np.random.normal(0, magn_accuracy*2*np.pi, 1)[0]

	v_noise = front_noise, right_noise, theta_noise, omega_noise
	return v_noise


def get_front_wall(s):
	x, y, theta = s
	d_r, d_t, d_l, d_b = get_wall_distances(s)
	positive_ds = [d for d in [d_r, d_t, d_l, d_b] if d>0]
	dist = min(positive_ds)
	# 0 = right wall
	# 1 = top wall
	# 2 = left wall
	# 3 = bottom wall
	return [d_r, d_t, d_l, d_b].index(dist)

# Meet
def get_W(s, u):
	# 3x2
	return W

# Meet
def get_F(s, u):
	# 3x3
	return F


def get_H_r(s):
	return H

def get_H_t(s):

	return H

def get_H_l(s):

	return H

def get_H_b(s):

	return H

# Yuanyuan
def get_H(s):
	# 4x3
	return H

# Yuanyuan
def get_V(s):
	# 4x4
	return V


# Yuanyuan
def get_wall_distances(s):
	x, y, theta = s

	d_r, d_t, d_l, d_b = 1,1,1,1	#DELETE THIS LINE ONCE IMPLEMENTED
	
	return d_r, d_t, d_l, d_b

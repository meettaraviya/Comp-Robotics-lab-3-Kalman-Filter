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

def f_transition(s, u, w_noise):
	w_noise_l, w_noise_r = w_noise
	omega_l, omega_r = u
	x, y, theta = s

	s_ = x_, y_, theta_
	return s_


def h_observation(s, v_noise):
	x, y, theta = s
	# uses get_wall_distances
	d_r, d_t, d_l, d_b = get_wall_distances(s)
	o = d_front, d_right, theta_o, omega_o
	return o


def generate_v_noise():

	return v_noise


def get_front_wall(s):
	x, y, theta = s
	d_r, d_t, d_l, d_b = get_wall_distances(s)
	# 0 = right wall
	# 1 = top wall
	# 2 = left wall
	# 3 = bottom wall
	return 0/1/2/3

def get_W(s, u):
	# 3x2
	return W

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


def get_H(s):
	# 4x3
	return H

def get_V(s):
	# 4x4
	return V



def get_wall_distances(s):
	x, y, theta = s

	return d_r, d_t, d_l, d_b



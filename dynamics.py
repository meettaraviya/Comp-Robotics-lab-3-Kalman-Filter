import numpy as np

# map height
map_height = 500.0
# map width
map_width = 500.0

robot_width = 90.0 #mm
wheel_radius = 25.0 #mm

fps = 60.0
playspeed = 1.0
dt = 1/fps

# C                B       A
# |----------------========|
# |                        |
# |                        |
# |                   o    | ---> Front when h = 0  
# |                        |
# |                        |
# |----------------========|
# F                E       D
# 
# o = Origin
# [A, B, C, D, E, F]
robot_points = [
	(+wheel_radius , -robot_width/2 ),
	(-wheel_radius , -robot_width/2 ),
	(-(robot_height - wheel_radius) , -robot_width/2 ),
	(+wheel_radius , +robot_width/2 ),
	(-wheel_radius , +robot_width/2 ),
	(-(robot_height - wheel_radius) , +robot_width/2 ),
]


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

	v_l = (omega_l/60.0) * (np.pi*2*wheel_radius)
	v_r = (omega_r/60.0) * (np.pi*2*wheel_radius)

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
	x, y, theta = s
	omega_l, omega_r = u
	W = np.matrix(np.zeros((3,2)))

	W[0] = dt * wheel_radius * np.cos(theta) / 2.0
	W[1] = dt * wheel_radius * np.sin(theta) / 2.0
	W[2, 0] = -dt * wheel_radius / robot_width
	W[2, 1] = dt * wheel_radius / robot_width

	return W

# Meet
def get_F(s, u):
	# 3x3
	x, y, theta = s
	omega_l, omega_r = u
	# (del s_)/(del x)
	F = np.matrix(np.zeros((3,3)))
	# (del s_)/(del y)
	F[:,0] = [[1.0], [0.0], [0.0]]
	# (del s_)/(del theta)
	F[:,1] = [[0.0], [1.0], [0.0]]
	
	# (del theta_)/(del theta)
	F[2,2] = 1.0
	# (del x_)/(del theta)
	F[0,2] = -np.sin(theta) * dt * wheel_radius * (omega_l+omega_r) / 2.0
	# (del y_)/(del theta)
	F[1,2] = np.cos(theta) * dt * wheel_radius * (omega_l+omega_r) / 2.0
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


def display_init():
	global , background
	pygame.init()
	screen = pygame.display.set_mode((round(map_width), round(map_height)))
	pygame.display.set_caption('Kalman Fiter demo')
	background = pygame.Surface(screen.get_size())
	background = background.convert()
	background.fill((255, 255, 255, 100))
	screen.blit(background, (0, 0))


def display_state(s):
	x, y, h = s
	t_points = [
		(x+p[0]*np.cos(h)-p[1]*np.sin(h), y+p[0]*np.sin(h)+p[1]*np.cos(h)) for p in robot_points
	]
	pygame.draw.line(screen, (255,0,0), t_points[0], t_points[2])
	pygame.draw.line(screen, (255,0,0), t_points[3], t_points[5])
	pygame.draw.line(screen, (255,0,0), t_points[0], t_points[3])
	pygame.draw.line(screen, (255,0,0), t_points[2], t_points[5])

	pygame.draw.line(screen, (0,0,255), t_points[0], t_points[1], 3)
	pygame.draw.line(screen, (0,0,255), t_points[3], t_points[4], 3)


def display_sample_state(s):
	x, y, h = s
	pygame.draw.circle(screen, (0,255,0), (round(x), round(y)), 10)
	pygame.draw.line(screen, (0,255,0), (round(x), round(y)), (round(x+2*point_size*np.cos(h)), round(y+2*point_size*np.sin(h))), 4)


def display_distribution(s_mean, Sigma):
	for state in np.random.multivariate_normal(s_mean, Sigma, 100):
		display_sample_state(state)


def display_update():
	pygame.display.flip()
	clock.tick(round(fps * playspeed))
	screen.blit(background, (0, 0))

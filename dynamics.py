import numpy as np
import pygame

# map height
map_height = 500.0
# map width
map_width = 500.0

robot_width = 90.0 #mm
robot_height = 100.0
wheel_radius = 25.0 #mm

fps = 10.0
playspeed = 0.1
dt = 1/fps

point_size = 4

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
	(+wheel_radius , +robot_width/2 ),
	(-wheel_radius , +robot_width/2 ),
	(-(robot_height - wheel_radius) , +robot_width/2 ),
	(+wheel_radius , -robot_width/2 ),
	(-wheel_radius , -robot_width/2 ),
	(-(robot_height - wheel_radius) , -robot_width/2 ),
]


# 2x2
Q = np.matrix( [[0.05*60, 0], [0, 0.05*60]] )
# Q = np.matrix( [[0, 0], [0, 0]] )

# 4x4
laser_accuracy = 0.04
gyro_accuracy = 0.03
magn_accuracy = 0.0625

max_dist = np.sqrt( map_height**2 + map_width**2 )

v_l = (-60/60.0) * (3.14*2*wheel_radius)
v_r = (60/60.0) * (3.14*2*wheel_radius)
max_omega = (v_r - v_l)/robot_width

R = np.matrix( [[laser_accuracy*max_dist, 0, 0, 0], 
				[0, laser_accuracy*max_dist, 0, 0], 
				[0, 0, magn_accuracy*2*np.pi, 0], 
				[0, 0, 0, gyro_accuracy*max_omega]] )

# Adrian
def f_transition(s, u, w_noise):
	w_noise_l, w_noise_r = w_noise
	omega_l, omega_r = u
	x, y, theta = s

	# effective inputs after wheel slippage/noise
	omega_l, omega_r = omega_l + w_noise_l, omega_r + w_noise_r
	
	v_l = (omega_l/60.0) * (np.pi*2*wheel_radius)
	v_r = (omega_r/60.0) * (np.pi*2*wheel_radius)
	omega = (v_r - v_l)/robot_width

	v = (v_l + v_r) / 2
	
	theta_ = (theta + (dt*omega)) % (2*np.pi)
	x_ = x + (dt * v * np.cos(theta))
	y_ = y + (dt * v * np.sin(theta))

	s_ = x_, y_, theta_
	return np.array(s_)

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
	
	s_rotated = (x, y, (theta - np.pi/2)%(2*np.pi))

	# uses get_wall_distances
	distances = get_wall_distances(s)	# d_r, d_t, d_l, d_b
	distances_rot = get_wall_distances(s_rotated)	# d_r, d_t, d_l, d_b

	front_o = distances[get_front_wall(s)]
	right_o = distances_rot[get_front_wall(s_rotated)]		# right wall is at index (i-1) mod 4

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

	o = front_o, right_o, theta_o % (2*np.pi), omega_o
	return np.array(o).reshape(4,1)

# Andrew
def generate_v_noise(s, u):
	x, y, theta = s
	omega_l, omega_r = u

	distances = get_wall_distances(s)	# d_r, d_t, d_l, d_b
	s_rotated = (x, y, theta - np.pi/2)
	distances_rot = get_wall_distances(s_rotated)	# d_r, d_t, d_l, d_b

	d_front = distances[get_front_wall(s)]
	d_right = distances_rot[get_front_wall(s_rotated)]

	# v_l = (omega_l/60.0) * (np.pi*2*wheel_radius)
	# v_r = (omega_r/60.0) * (np.pi*2*wheel_radius)
	# omega_o = abs((v_r - v_l)/robot_width)

	front_noise = np.random.normal(0, laser_accuracy*d_front, 1)[0]
	right_noise = np.random.normal(0, laser_accuracy*d_right, 1)[0]

	omega_noise = np.random.normal(0, gyro_accuracy*max_omega, 1)[0]
	theta_noise = np.random.normal(0, magn_accuracy*2*np.pi, 1)[0]

	v_noise = front_noise, right_noise, theta_noise, omega_noise
	return np.array(v_noise).reshape(4,1)


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


# Yuanyuan
def get_H_for_front(wall_sel, s):
	x, y, theta = s
	H = map_height
	W = map_width
	pi = np.pi
	angle_theta = ((theta/pi) % 2 ) *pi 
	if angle_theta < 0:
		angle_theta = angle_theta + 2*pi

	rows = [[-1/np.cos(angle_theta), 0, (W-x)*np.tan(angle_theta)/np.cos(angle_theta)],
			[0, -1/np.cos(angle_theta-0.5*pi), (H-y)*np.tan(angle_theta-0.5*pi)/np.cos(angle_theta-0.5*pi)],
			[1/np.cos(angle_theta-pi), 0, x*np.tan(angle_theta-pi)/np.cos(angle_theta-pi)],
			[0, 1/np.cos(angle_theta-1.5*pi), y*np.tan(angle_theta-1.5*pi)/np.cos(angle_theta-1.5*pi)]]
	
	return rows[wall_sel]

#Yuanyuan
def get_H_for_right(wall_sel, s):
	x, y, theta = s
	H = map_height
	W = map_width
	pi = np.pi
	angle_theta = ((theta/pi) % 2 ) *pi 
	if angle_theta < 0:
		angle_theta = angle_theta + 2*pi

	rows = [[-1/np.cos(angle_theta-0.5*pi), 0, (W-x)*np.tan(angle_theta-0.5*pi)/np.cos(angle_theta-0.5*pi)],
			[0, -1/np.cos(angle_theta-pi), (H-y)*np.tan(angle_theta-pi)/np.cos(angle_theta-pi)],
			[1/np.cos(angle_theta-1.5*pi), 0, x*np.tan(angle_theta-1.5*pi)/np.cos(angle_theta-1.5*pi)],
			[0, 1/np.cos(angle_theta), y*np.tan(angle_theta)/np.cos(angle_theta)]]
	
	return rows[wall_sel]

# Yuanyuan
def get_H(s):
	# 4x3
	x, y, theta = s
	front_wall_sel = get_front_wall(s)
	right_wall_sel = get_front_wall([x, y, theta-0.5*np.pi])
	# 0 = right wall
	# 1 = top wall
	# 2 = left wall
	# 3 = bottom wall
	d_front = get_H_for_front(front_wall_sel,s)
	d_right = get_H_for_right(right_wall_sel,s)

	n_theta = [0,0,1]
	omega = [0, 0, 1/dt] #since omega is independent of x,y,theta; the derivative should be 0

	H = [d_front, d_right, n_theta, omega]


	return np.matrix(H)


# Yuanyuan
def get_V(s):
	# 4x4
	V = [[1,0,0,0],
		 [0,1,0,0],
		 [0,0,1,0],
		 [0,0,0,1]]

	return np.matrix(V)


# Yuanyuan
def get_wall_distances(s):
	x, y, theta = s
	pi = np.pi

	H = map_height
	W = map_width
	
	angle_theta = ((theta/pi) % 2 ) *pi 
	if angle_theta < 0:
		angle_theta = angle_theta + 2*pi

	if angle_theta in [0, pi, 2*pi]:
		d_t = np.inf
		d_b = np.inf
		d_r = W-x
		d_l = -x
		if angle_theta == pi:
			d_r = -d_r
			d_l = -d_l
	elif angle_theta in [0.5*pi, 1.5*pi]:
		d_r = np.inf
		d_l = np.inf
		d_t = H-y
		d_b = -y
		if angle_theta == 1.5*pi:
			d_t = -d_t
			d_b = -d_b
	elif angle_theta >= 0 and angle_theta <= 0.5*pi:
		d_r = (W-x)  / np.cos(angle_theta)
		d_l = -x     / np.cos(angle_theta)	
		d_t = (H-y)  / np.cos(angle_theta-0.5*pi)
		d_b = -y     / np.cos(angle_theta-0.5*pi)
	elif angle_theta >= 0.5*pi and angle_theta <= pi:
		d_r = -(W-x) / np.cos(angle_theta-pi)
		d_l =  x     / np.cos(angle_theta-pi)
		d_t = (H-y)  / np.cos(angle_theta-0.5*pi)
		d_b = -y     / np.cos(angle_theta-0.5*pi)
	elif angle_theta >= pi and angle_theta <= 1.5*pi:
		d_r = -(W-x) / np.cos(angle_theta-pi)
		d_l =  x     / np.cos(angle_theta-pi)
		d_t = -(H-y) / np.cos(angle_theta-1.5*pi)
		d_b =  y     / np.cos(angle_theta-1.5*pi)
	elif angle_theta >= 1.5*pi and angle_theta <= 2*pi:
		d_r = (W-x)  / np.cos(angle_theta)
		d_l = -x     / np.cos(angle_theta)
		d_t = -(H-y) / np.cos(angle_theta-1.5*pi)
		d_b =  y     / np.cos(angle_theta-1.5*pi)

	return d_r, d_t, d_l, d_b


def display_init():
	global screen, background, clock
	pygame.init()
	screen = pygame.display.set_mode((round(map_width), round(map_height)))
	pygame.display.set_caption('Kalman Fiter demo')
	background = pygame.Surface(screen.get_size())
	background = background.convert()
	background.fill((255, 255, 255, 100))
	screen.blit(background, (0, 0))
	clock = pygame.time.Clock()


def display_state(s):
	global screen
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
	global screen
	x, y, h = s
	pygame.draw.circle(screen, (0,255,0), (round(float(x)), round(float(y))), point_size)
	pygame.draw.line(screen, (0,255,0), (round(float(x)), round(float(y))), (round(x+2*point_size*np.cos(h)), round(y+2*point_size*np.sin(h))), point_size//2)


def display_distribution(s_mean, Sigma):
	for state in np.random.multivariate_normal(s_mean, Sigma, 10):
		display_sample_state(state)


def display_update():
	global clock, screen
	pygame.display.flip()
	clock.tick(round(fps * playspeed))
	screen.blit(background, (0, 0))

	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			return False
		if event.type == pygame.KEYDOWN:
			if event.key == pygame.K_ESCAPE or event.unicode == 'q':
				return False

	return True

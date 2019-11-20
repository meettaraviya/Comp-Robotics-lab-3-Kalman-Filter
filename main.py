import numpy as np
from dynamics import *
from trajectories import *

def get_action(s, t):
	# # Case 1
	# # omega_l = 60
	# # omega_r = 40

	# # Case 2
	# # omega_l = 60
	# # omega_r = 30

	# # Case 3
	# # omega_l = 60
	# # omega_r = 60

	# # Case 4
	# # omega_l = 60
	# # omega_r = -60

	# # Case 5
	# omega_l = 25 * np.cos(2*np.pi*t/180)
	# omega_r = 25 * np.sin(np.pi*t/180)

	# # print(omega_l, omega_r)

	x, y, theta = s

	if x < 0 or x > map_width or y < 0 or y > map_height:
		return None
	
	# return omega_l, omega_r

	# return straight_line(t)
	# return circle(t)
	# return no_movement(t)
	# return big_circle(t)
	return square(t)
	

# Andrew
def get_next_state(s, u):
	'''
	Return the true next state, after wheel slippage.
	'''
	# effective wheel speed is normal with standard deviation of 5% max motor speed
	w_noise = np.random.multivariate_normal(np.zeros((2)), Q)
	s_ = f_transition(s, u, w_noise)
	return np.array(s_)


def KalmanFilter():

	# very interesting phenomenon
	# np.random.seed(0)

	# very interesting phenomenon
	# np.random.seed(10)

	s_initial = np.array([100,250, np.pi/2])
	
	# case 1: initial state known exactly
	s_mean = s_initial
	Sigma = np.zeros((3,3))

	# case 2: uncertain initial state knowledge
	s_mean = s_initial
	Sigma = np.eye(3)
	Sigma[0,0] = 20
	Sigma[1,1] = 20
	# s_mean = s_initial
	# Sigma = 10*np.ones((3,3))

	display_init()
	display_distribution(s_mean, Sigma)
	display_state(s_initial)
	if not display_update():
		return

	s = s_initial
	print(s)

	T = 10000
	for t in range(T):

		u = get_action(s, t)

		if not u:
			break
	
		# just used for getting the observation
		s_ = get_next_state(s, u)

		# time update
		s_mean_ = f_transition(s_mean, u, (0,0))
		
		F = get_F(s_mean, u)
		W = get_W(s_mean, u)

		Sigma_ = F * Sigma * F.T + W * Q * W.T

		display_distribution(s_mean_, Sigma_)
		display_state(s_)
		if not display_update():
			break

		# observation update
		o = h_observation(s_, u, generate_v_noise(s_, u))

		H = get_H(s_mean_)

		update = Sigma_ * H.T * (H * Sigma_ * H.T + R).I * (o - h_observation(s_mean_, u, (0,0,0,0)))
		s_mean__ = s_mean_ + update.reshape(1,3)
		Sigma__ = Sigma_ - Sigma_ * H.T * (H * Sigma_ * H.T + R).I * H * Sigma_

		s_mean = np.asarray(s_mean__).reshape(-1)
		Sigma = Sigma__
		s = s_
		
		display_distribution(s_mean, Sigma)
		display_state(s_)
		if not display_update():
			break

		print(t+1, s, s_mean)


if __name__ == "__main__":
	KalmanFilter()
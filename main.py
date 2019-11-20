import numpy as np
from dynamics import *

np.random.seed(0)

# Adrian
def get_action(t):
	omega_l = 30
	omega_r = 10
	
	return omega_l, omega_r

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
	s_initial = np.array([250,250, np.pi/4])
	s_mean = s_initial

	# 3x3
	Sigma = np.zeros((3,3))

	display_init()
	display_state(s_initial)
	display_distribution(s_mean, Sigma)

	T = 1000
	for t in range(T):

		u = get_action(t)
	
		# just used for getting the observation
		s_ = get_next_state(s_mean, u)

		# time update
		s_mean_ = f_transition(s_mean, u, (0,0))
		
		F = get_F(s_mean, u)
		W = get_W(s_mean, u)

		Sigma_ = F * Sigma * F.T + W * Q * W.T

		display_state(s_)
		display_distribution(s_mean_, Sigma_)
		display_update()

		# observation update
		o = h_observation(s_, u, generate_v_noise(s_, u))

		H = get_H(s_mean_)

		update = Sigma_ * H.T * (H * Sigma_ * H.T + R).I * (o - h_observation(s_mean_, u, (0,0,0,0)))
		s_mean__ = s_mean_ + update.reshape(1,3)
		Sigma__ = Sigma_ - Sigma_ * H.T * (H * Sigma_ * H.T + R).I * H * Sigma_

		s_mean = np.asarray(s_mean__).reshape(-1)
		Sigma = Sigma__
		s = s_
		
		display_state(s_)
		display_distribution(s_mean, Sigma)
		display_update()

		print(t)


tester = "meet"

if __name__ == "__main__" and tester == "andrew":
	# Andrew's testing space
	print("Starting h_observation tests.")
	s = [(0,0,0)]
	u = [(0,0), (60,60), (-60,-60), (60,-60), (-60,60), (40,20), (-30,30), (0,1)]
	sign = [0, 0, 0, -1, +1, -1, +1, +1]
	for state in s:
		for action in u:
			h = h_observation(state,action,(0,0,0,0))
			assert(np.sign(h[3]) == sign[u.index(action)])
			print(state, action, h)
	print("Ending h_observation tests.")
	print("-------------------------------------------")
	print("Starting generate_v_noise tests.")
	for state in s:
		for action in u:
			v = generate_v_noise(state, action)
			print(v)
	print("Ending generate_v_noise tests.")

	display_init()
	display_sample_state((0,0,0))

elif __name__ == "__main__" and tester == "meet":
	KalmanFilter()
	pass

elif __name__ == "__main__" and tester == "yy":
	pass

elif __name__ == "__main__" and tester == "adrian":
	pass
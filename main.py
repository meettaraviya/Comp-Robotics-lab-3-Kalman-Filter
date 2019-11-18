import numpy as np
from dynamics import *

np.random.seed(0)

# Adrian
def get_action(t):

	return omega_l, omega_r

# Andrew
def get_next_state(s, u):
	'''
	Return the true next state, after wheel slippage.
	'''
	# effective wheel speed is normal with standard deviation of 5% max motor speed
	w_noise = np.random.normal(0, 0.05*60, 2)

	s_ = f_transition(s, u, w_noise)
	return s_


def KalmanFilter():
	s_initial = (250,250, np.pi/4)
	s_mean = s_initial

	# 3x3
	Sigma = None

	display_state(s)
	display_distribution(s_mean, Sigma)

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
		o = h_observation(s_, u, generate_v_noise(s_))

		H = get_H(s_mean_)

		s_mean__ = s_mean_ + Sigma_ * H.T * (H * Sigma_ * H.T + R).I * (o - h_observation(s_mean_, u, (0,0,0,0)))
		Sigma__ = Sigma_ - Sigma_ * H.T * (H * Sigma_ * H.T + R).I * H * Sigma_

		s_mean = s_mean__
		Sigma = Sigma__
		s = s_

		display_state(s_)
		display_distribution(s_mean__, Sigma__)
		display_update()



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

elif __name__ == "__main__" and tester == "meet":
	pass

elif __name__ == "__main__" and tester == "yy":
	pass

elif __name__ == "__main__" and tester == "adrian":
	pass
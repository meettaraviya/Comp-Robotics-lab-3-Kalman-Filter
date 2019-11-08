import numpy as np
from dynamics import *


def get_action(t):

	return omega_l, omega_r


def get_next_state(s, u):
	# noisy function
	w_noise = random(Q)
	s_ = f_transition(s, u, w_noise)
	return s_

def KalmanFilter():

	s_initial = (250,250, np.pi/4)
	s_mean = s_initial


	# 3x3
	Sigma = None

	for t in range(T):

		u = get_action(t)
	
		# just used for getting the observation
		s_ = get_next_state(s, u)

		# time update
		s_mean_ = f_transition(s_mean, u, (0,0))
		
		F = get_F(s_mean, u)
		W = get_W(s_mean, u)

		Sigma_ = F * Sigma * F.T + W * Q * W.T

		# observation update
		o = h_observation(s_, random())

		H = get_H(s_mean_)

		s_mean__ = s_mean_ - Sigma_ * H.T *




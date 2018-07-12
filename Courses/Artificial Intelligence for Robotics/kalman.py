# Artificial Intelligence for Robotics, Udacity
# Jonathon Rice

import math
import numpy as np

# Calculates gaussian values
def gaussian(mu, sigma, x):
	n = pow(np.e, ((x-mu)**2)/(-2*sigma*sigma))
	return (n/(np.sqrt(2*np.pi)*sigma))

# print(gaussian(10, 2, 8))

# take previous belief and update with new measurement
def update(mean1, var1, mean2, var2):
	new_mean = (mean2*var1 + mean1*var2)/(var1+var2)
	new_var = 1/(1/var1+1/var2)
	return [new_mean, new_var]

# print(update(10.,8.,13.,2.))

def predict(mean1, var1, mean2, var2):
	new_mean = mean1 + mean2
	new_var = var1 + var2
	return [new_mean, new_var]

# print(predict(10.,4.,12.,4.))

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0.
sig = 10000.

for i in range(len(motion)):
	mu, sig = update(mu, sig, measurements[i], measurement_sig)
	mu, sig = predict(mu, sig, motion[i], motion_sig)

print [mu, sig]
# Artificial Intelligence for Robotics, Udacity
# Jonathon Rice

import math
import numpy as np

# Calculates gaussian values
def gaussian(mu, sigma, x):
	n = pow(np.e, ((x-mu)**2)/(-2*sigma*sigma))
	return (n/(np.sqrt(2*np.pi)*sigma))

# print(gaussian(10, 2, 8))

def update(mean1, var1, mean2, var2):
	new_mean = (mean2*var1 + mean1*var2)/(var1+var2)
	new_var = 1/(1/var1+1/var2)
	return [new_mean, new_var]

print(update(10.,8.,13.,2.))
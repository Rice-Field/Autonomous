# Artificial Intelligence for Robotics, Udacity
# Jonathon Rice

import math
import numpy as np

# Calculates gaussian values
def gaussian(mu, sigma, x):
	n = pow(np.e, ((x-mu)**2)/(-2*sigma*sigma))
	return (n/(np.sqrt(2*np.pi)*sigma))

print(gaussian(10, 2, 8))
# Artificial Intelligence for Robotics, Udacity
# Jonathon Rice
# Python 3.6

import numpy as np

world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2

pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

#########################
# Uniform Probability
#########################
n = 5
Loc = np.zeros(n, dtype=float)
distance = len(Loc)

uni = 1.0/distance
for i in range(distance):
	Loc[i] = uni

# print(Loc)

#########################
# Probability After Sense
#########################

# Assume key point is in index 1,2
def senseProto():
    for i in range(distance):
        if i == 1 or i == 2:
            Loc[i] *= pHit

        else:
            Loc[i] *= pMiss


# print(Loc)

#########################
# Sum
#########################
def sum(p):
    total = 0
    for i in range(len(p)):
        total += p[i]
    return total

# print(total)

#########################
# Sense
#########################

def sense(p, Z):
    q = []
    
    for i in range(len(p)):
        if Z == world[i]:
            q.append(p[i]*pHit)
        else:
            q.append(p[i]*pMiss)

    total = sum(q)
    for i in range(len(q)):
        q[i] /= total
    
    return q

print(sense(Loc, 'red'))

#########################
# Multiple measurements
#########################

def multmeas(p, measurements):
    for i in range(len(measurements)):
        p = sense(p, measurements[i])
    return p

#########################
# Motion - Cyclic world
#########################
def move(p, U):
    q = []
    for i in range(len(p)):
        q.append(p[(i-U) % len(p)])
    return q

print(move(sense(Loc, 'red'), 1))

#########################
# Inaccurate Motion
#########################

def iMove(p, U):
    U = U % len(p)
    q = p[-U:] + p[:-U]

    max = 0
    index = 0
    for i in range(len(q)):
        if q[i] > max:
            max = q[i]
            index = i
    q[index] *= pExact
    if index == 0:
        q[-1] = max*pUndershoot
    else:
        q[index-1] = max*pUndershoot
        
    if index == len(q)-1:
        q[0] = max*pOvershoot
    else:
        q[index+1] = max*pOvershoot
        
    return q

# or 
def iMove2(p, U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U) % len(p)]
        s = s + pOvershoot * p[(i-U-1) % len(p)]
        s = s + pUndershoot * p[(i-U+1) % len(p)]
    return q
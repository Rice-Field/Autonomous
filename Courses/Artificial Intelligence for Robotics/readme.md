# Artificial Intelligence for Robotics, Udacity

Notes on the Udacity course. [Course Site](https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373)

## Localization

### Initial info

- GPS isn't very accurate
	* 10m error, much higher with buildings
- Need 2-10 cm of error for autonomous driving

### Total Probability

- Give equal probability for entire space
	* Initialized location belief
- Match with known key points
	* Reduces probability to particular locations
	* Posterior belief
- Use motion to shift location beliefs
	* Convolution
- Using motion and key points together
	* Narrows location belief until localized
	* Probabities are never zeroed out to account for errors

```
1D Location Probability
	Denisty = Probability
   ╔═══════════════╗
1. ║▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒║ uniform
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   ╔═══════════════╗
2. ║░▒░░▒░░░░░▒░░░░║ key point, 3 locations
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   ╔═══════════════╗
3. ║░░░░▒░░▒░░░░░▒░║ Shift with motion
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   ╔═══════════════╗
4. ║ ░  ▓  ░  ░  ░ ║  Near same key point
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   Localized, 1 best option
```

### Example
- 5 Grid cells, start uniformed
	* Each cell p = 0.2
- 2 Cells contain key point, index 1 & 2
	* Multiply by .6 for key point, otherwise .2
	* Non-key point p = .04, key point p = .12
	* Normalize so sum = 1
		- By dividing each cell by sum

```
   ╔════════════════════════╗
1. ║ .2 ║ .2 ║ .2 ║ .2 ║ .2 ║ uniform
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   ╔═════════════════════════════╗
2. ║ .04 ║ .12 ║ .12 ║ .04 ║ .04 ║ sum = .36
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   ╔═════════════════════════════╗
3. ║ .11 ║ .33 ║ .33 ║ .11 ║ .11 ║ normalize
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
```

### Motion
- Inaccurate motion accrues, makes localization hard
	* Given enough movement localization will return to
	maximum entropy
	* Cycle of gaining and losing information from sensing and moving

```
Exact = 0.8, probability
Overshoot = 0.1
Undershoot = 0.1
   ╔═══════════════╗
1. ║    ▓          ║ Move 2 to the right
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   ╔═══════════════╗
2. ║     ░▓░       ║ Errors in measurement and movement
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
   ╔═══════════════╗
3. ║▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒║ Max entropy, can be reached in 1000 movements
   ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
```

### Bayes' Rule
* p(Xi|Z) = P(Z|Xi)P(Xi)/P(Z)
	- P(Z|Xi) = measurement probability
	- P(Xi) = prior
	- P(Z) = sum P(Z|Xi)P(Xi)
* Total probability

## Kalman Filters

### Initial info

```
      ┌ ─ ─ ─ ─ ─ ─ ┐
      |             √
   update      prediction
      ^             |
      └ ─ ─ ─ ─ ─ ─ ┘
```

- Used to estimate the state of a system
   * Will be used for tracking
   * Using observables we are able to derive information on the hidden states
- Similar to the Monte Carlo method (Carlo was used for localization earlier)
   * Kalman is continuous, Carlo is discrete

### Gaussian Representation
- Maintain mean and variance to estimate localization of object
   * Continuous space compared the histogram space previously used

### Motion
- Old mean plus motion equals new belief mean
- Old sigma plus motion variance equals new belief variance
   * Without new measurements, certainty decreases

```
Matrix Multiplication
x = position, m = motion
┌    ┐     ┌     ┐ ┌   ┐
| x` | <-- | 1 1 | | x | State transition function, F
| m` |     | 0 1 | | m |
└    ┘     └     ┘ └   ┘
┌   ┐      ┌     ┐ ┌   ┐
| z | <--  | 1 0 | | x | Measurement function, H
└   ┘      └     ┘ | m |
                   └   ┘

Prediction                    x = estimate
x` = Fx + u                   P = uncertainty covariance
P` = F * P * F^T              F = state transition matrix
                              u = motion vector
Measurement                   z = measurement
y = z - H * x                 H = measurement function
S = H * P * H^T + R           R = measurement noise
K = P * H^T * S^-1            I = identity matrix
x` = x + (K * y)
P` = (I - K * H) * P

kalman.py implements these equations
```

### MultiVariate Representation
- 1 mean for each dimension, vector
- Variance is a DxD matrix, called co-variance
- 2D looks like contour lines
- Kalman filter has a quadratic runtime
   * Previous histogram filter has exponential runtime

## Particle Filters

### Initial info
- State space: continuous
- Belief: multimodal
- Runtime: good for tracking, exponential with high dimensions
- Distribute particle sets evenly, (x, y, direction)
   * Particles survive based of consistency with measurements
- Easy to program

### Resampling
- An importance weight is assigned to each particle
   * Based off the consistency with measurements
- Normalized weights are used as probability for survival
   * With replacement, so a particle can have multiple copies

## Search

### Initial info
- Method used for motion planning
   * Using a discrete representation
- Given map, starting location, goal location and cost
   * Find minimum cost path

### A* Method
- More efficient than checking every location
   * a heuristic function
- By attempting to predict addition cost from
cell locations reduces search space to optimal locations
   * under the right conditions

```
   Heuristic Cost Function
      Cell value = additional cost to goal from cell

   ┌────┬────┬────┬────┬────┐
   │ 08 │ 07 │ 06 │ 05 │ 04 │
   ├────┼────┼────┼────┼────┤
   │ 07 │ 06 │ 05 │ 04 │ 03 │
   ├────┼────┼────┼────┼────┤
   │ 06 │ 05 │ 04 │ 03 │ 02 │
   ├────┼────┼────┼────┼────┤
   │ 05 │ 04 │ 03 │ 02 │ 01 │
   ├────┼────┼────┼────┼────┤
   │ 04 │ 03 │ 02 │ 01 │ 00 │ <── Goal
   └────┴────┴────┴────┴────┘

   Assumes no obstacles

```
### Dynamic Programming
- Given map and goal
   * gives best path from anywhere
   * By labeling each cell with an optimal policy
- Recursively fill in cost from goal to cells
   * Costs start at 0 from goal
   * assign policies that lower cost

```
   Dynamic Programming

   ┌────┬────┬────┬────┬────┐
   │ V  │ V  │ <  │ <  │ <  │
   ├────┼────┼────┼────┼────┤
   │ V  │ V  │▓▓▓▓│▓▓▓▓│▓▓▓▓│
   ├────┼────┼────┼────┼────┤
   │ V  │ V  │▓▓▓▓│▓▓▓▓│ V  │
   ├────┼────┼────┼────┼────┤
   │ V  │ V  │ V  │▓▓▓▓│ V  │
   ├────┼────┼────┼────┼────┤
   │  > │  > │  > │  > │ G* │
   └────┴────┴────┴────┴────┘

```

## PID Control

### Initial info
- Discrete motion is not ideal for robot motion
   * Generate smooth paths from discrete ones
   * Diagonal straight lines might be infeasible

### Smoothing Algorithm
   ```
   y_i = x_i

   Optimize:
      (x_i - y_i)^2 -> min
      (y_i - y_i+1)^2 -> min

   apply a weight to both terms
   ```

### Control
- Steering in proportion to Cross Track Error
   * CTE: Distance from desired trajectory
   * P of PID, will overshoot with just proportion
- To avoid overshooting CTE, using temporal differential
   * Smoother approach to desired trajectory
- Include proportion to sum of CTE

## SLAM


### Initial info

```
▓▒░ ╔═╚╗╝╦╩║¯ ┌ ┐┘─└ ┼ ├ ┬ ┴ ┤ │ ASCII builders
```
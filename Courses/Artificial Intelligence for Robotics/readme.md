# Artificial Intelligence for Robotics, Udacity

Notes on the Udacity course. [Course Site](https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373)

## Localization

### Initial info

- GPS isn't very accurate
	* 10m error, much higher with buildings
- need 2-10 cm of error for autonomous driving

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
	* probabities are never zeroed out to account for errors

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
- 5 grid cells, start uniformed
	* Each cell p = 0.2
- 2 cells contain key point, index 1 & 2
	* multiply by .6 for key point, otherwise .2
	* non-key point p = .04, key point p = .12
	* normalize so sum = 1
		- by dividing each cell by sum

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
	* given enough movement localization will return to
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
- Similar to the Monte Carlo method (Carlo was used for localization earlier)
   * Kalman is continuous, Carlo is discrete

### Gaussian Representation
- Maintain mean and variance to estimate localization of object
   * Continuous space compared the histogram space previously used

### Motion
- Old mean plus motion equals new belief mean
- Old sigma plus motion variance equals new belief variance
   * Without new measurements, certainty decreases


## Particle Filters

## Search

## PID Control

## SLAM

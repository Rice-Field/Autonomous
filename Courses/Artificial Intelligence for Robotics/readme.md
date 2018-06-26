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
	* Inaccurate motion accrues, makes localization hard 
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

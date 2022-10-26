For gloc_node2.cpp

Basic Parameters 1: 
robot trajectory: a_clock45 starting from upper left corner(0, 0) and goes clockwise
map landmarks: c_map
range of observation:	double r_obs = 3; 
Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, 0.0, 0.0)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 20cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range

example1.pdf: (x.txt lm.txt cov_x.txt cov_lm.txt box1.txt)
-- only measurement factors, no map factors

example2.pdf: (x1.txt lm1.txt cov_x1.txt cov_lm1.txt box2.txt)
-- only measurement factors with systematic error(range + 1sigma), no map factors

example3.pdf:(x1.txt lm1.txt cov_x1.txt cov_lm1.txt x2.txt lm2.txt cov_x2.txt cov_lm2.txt box2.txt)
-- comparison of example 2 and 3



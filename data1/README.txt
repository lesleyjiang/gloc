For gloc_node1.cpp

Basic Parameters 1: 
map landmarks: a_map
range of oservation:	double r_obs = 2; 
Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, 0.0, 0.0)  priorNoise = Sigmas(0.2, 0.2, 0.01) // 20cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range

example1.pdf: (x.txt lm.txt cov_x.txt cov_lm.txt box.txt)
-- only measurement factors, no map factors

example2.pdf: (x1.txt lm1.txt cov_x1.txt cov_lm1.txt box.txt)
-- only measurement factors, no map factors, smaller prior of posex0
pose(x0)  prior(0.0, 0.0, 0.0)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing

example3.pdf: (x2.txt lm2.txt cov_x2.txt cov_lm2.txt box2.txt)
-- only measurement factors with systematic error(range + 1sigma), no map factors, smaller prior of posex0

example4.pdf:(x1.txt lm1.txt cov_x1.txt cov_lm1.txt x2.txt lm2.txt cov_x2.txt cov_lm2.txt box.txt)
-- comparison of example 2 and 3

(in example123 forgot to comment the uniform distrubution noise of measurement range, so results are not strictly correct, but very close, since uniform distrubution noise dose not have big effect)

Basic Parameters 2: 
map landmarks: b_map
range of oservation:	double r_obs = 2; 
Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, 0.0, 0.0)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range

example5.pdf: (x3.txt lm3.txt cov_x3.txt cov_lm3.txt box3.txt)
-- only measurement factors, no map factors

example6.pdf: (x4.txt lm4.txt cov_x4.txt cov_lm4.txt box4.txt)
-- only measurement factors with systematic error(range + 1sigma), no map factors

example7.pdf:(x3.txt lm3.txt cov_x3.txt cov_lm3.txt x4.txt lm4.txt cov_x4.txt cov_lm4.txt box4.txt)
-- comparison of example 5 and 6

Basic Parameters 3: 
map landmarks: b_map
range of oservation:	double r_obs = 15; 
Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, 0.0, 0.0)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range

example8.pdf: (x5.txt lm5.txt cov_x5.txt cov_lm5.txt box5.txt)
-- only measurement factors, no map factors

example9.pdf: (x6.txt lm6.txt cov_x6.txt cov_lm6.txt box6.txt)
-- only measurement factors with systematic error(range + 1sigma), no map factors

example10.pdf:(x5.txt lm5.txt cov_x5.txt cov_lm5.txt x6.txt lm6.txt cov_x6.txt cov_lm6.txt box6.txt)
-- comparison of example 8 and 9

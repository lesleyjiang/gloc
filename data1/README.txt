For gloc_node1.cpp

Basic Parameters 1: 
robot trajectory: a_clock (starting from upper left corner(0, 0) and goes clockwise)
map landmarks: a_map
range of observation:	double r_obs = 2; 
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
robot trajectory: a_clock (starting from upper left corner(0, 0) and goes clockwise)
map landmarks: b_map
range of observation:	double r_obs = 2; 
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
robot trajectory: a_clock (starting from upper left corner(0, 0) and goes clockwise)
map landmarks: b_map
range of observation:	double r_obs = 15; 
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


Basic Parameters 4: 
robot trajectory: scenario1 a_counterclock (starting from upper left corner(0, 0) and goes counter-clockwise)
map landmarks: b_map
range of observation:	double r_obs = 2; 
Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, 0.0, -M_PI_2)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range
	
example11.pdf: (x7.txt lm7.txt cov_x7.txt cov_lm7.txt box7.txt)
-- only measurement factors, no map factors

example12.pdf: (x8.txt lm8.txt cov_x8.txt cov_lm8.txt box8.txt)
-- only measurement factors with systematic error(range + 1sigma), no map factors

example13.pdf:(x7.txt lm7.txt cov_x7.txt cov_lm7.txt x8.txt lm8.txt cov_x8.txt cov_lm8.txt box8.txt)
-- comparison of example 11 and 12


Basic Parameters 5: 
robot trajectory: scenario2 b_clock (starting from upper right corner(10, 0) and goes clockwise)
map landmarks: b_map
range of observation:	double r_obs = 2; 
Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(10.0, 0.0, -M_PI_2)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range
	
example14.pdf: (x9.txt lm9.txt cov_x9.txt cov_lm9.txt box9.txt)
-- only measurement factors, no map factors

example15.pdf: (x10.txt lm10.txt cov_x10.txt cov_lm10.txt box10.txt)
-- only measurement factors with systematic error(range + 1sigma), no map factors

example16.pdf:(x9.txt lm9.txt cov_x9.txt cov_lm9.txt x10.txt lm10.txt cov_x10.txt cov_lm10.txt box10.txt)
-- comparison of example 14 and 15


Basic Parameters 6: 
robot trajectory: scenario3 c_clock (starting from bottom right corner(10, -10) and goes clockwise)
map landmarks: b_map
range of observation:	double r_obs = 2; 
Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(10.0, -10.0, -M_PI)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range
	
example17.pdf: (x11.txt lm11.txt cov_x11.txt cov_lm11.txt box11.txt)
-- only measurement factors, no map factors

example18.pdf: (x12.txt lm12.txt cov_x12.txt cov_lm12.txt box12.txt)
-- only measurement factors with systematic error(range + 1sigma), no map factors

example19.pdf:(x11.txt lm11.txt cov_x11.txt cov_lm11.txt x12.txt lm12.txt cov_x12.txt cov_lm12.txt box12.txt)
-- comparison of example 17 and 18


Basic Parameters 7: 
robot trajectory: scenario4 d_clock (starting from bottom left corner(0, -10) and goes clockwise)
map landmarks: b_map
range of observation:	double r_obs = 2; 
Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, -10.0, M_PI_2)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range
	
example20.pdf: (x13.txt lm13.txt cov_x13.txt cov_lm13.txt box13.txt)
-- only measurement factors, no map factors

example21.pdf: (x14.txt lm14.txt cov_x14.txt cov_lm14.txt box14.txt)
-- only measurement factors with systematic error(range + 1sigma), no map factors

example22.pdf:(x13.txt lm13.txt cov_x13.txt cov_lm13.txt x14.txt lm14.txt cov_x14.txt cov_lm14.txt box14.txt)
-- comparison of example 21 and 22


Basic Parameters 8: 
robot trajectory: scenario4 d_clock (starting from bottom left corner(0, -10) and goes clockwise)
map landmarks: b_map
range of observation:	double r_obs = 2; 
Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, -10.0, M_PI_2)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range


example23.pdf:(x15.txt lm15.txt cov_x15.txt cov_lm15.txt box_13.txt)
-- measurement factors,  map factors
landmark: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(0.01, 0.01) // 1cm x, y

example24.pdf:(x16.txt lm16.txt cov_x16.txt cov_lm16.txt box_14.txt)
-- measurement factors,  map factors, systematic error(range + 1sigma)
landmark: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(0.01, 0.01) // 1cm x, y

example25.pdf:(x17.txt lm17.txt cov_x17.txt cov_lm17.txt box_15.txt)
-- measurement factors,  map factors, systematic error(range + 2sigma)
landmark: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(0.01, 0.01) // 1cm x, y

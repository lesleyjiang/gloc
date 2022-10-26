For gloc_node1.cpp

Basic Parameters: 
robot trajectory: a_clock (starting from upper left corner(0, 0) and goes clockwise)
map landmarks: b_map
range of observation:	double r_obs = 2; 
Interval:
	landmark radius = 0.03m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, 0.0, 0.0)  priorNoise = Sigmas(0.2, 0.2, 0.01) // 20cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range
	map prior: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(0.01, 0.01) // 1cm x, y



example27.pdf:(x18.txt lm18.txt cov_x18.txt cov_lm18.txt box16.txt)
-- measurement factors,  map factors

example28.pdf:(x19.txt lm19.txt cov_x19.txt cov_lm19.txt box17.txt)
-- measurement factors,  map factors, systematic error(range + 1sigma)


example29.pdf:(x20.txt lm20.txt cov_x20.txt cov_lm20.txt box18.txt)
-- measurement factors,  map factors, systematic error(range + 2sigma)


Basic Parameters 10: 
robot trajectory: a_clock (starting from upper left corner(0, 0) and goes clockwise)
map landmarks: b_map
range of observation:	double r_obs = 5; 
Actual measurements are random points in a [-0.03m, 0.03m]x[-0.03m, 0.03m] box centered at the real map point position
Interval:
	landmark radius = 0.03m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, -10.0, M_PI_2)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range
	landmark: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(0.01, 0.01) // 1cm x, y


example32.pdf:(x23.txt lm23.txt cov_x23.txt cov_lm23.txt box21.txt)
-- measurement factors,  map factors

example33.pdf:(x24.txt lm24.txt cov_x24.txt cov_lm24.txt box22.txt)
-- measurement factors,  map factors, systematic error(range + 2sigma)


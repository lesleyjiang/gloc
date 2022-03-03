Basic Parameters:

Interval:
	landmark radius = 0.3m (3*sigma)
	no prior on poses
	measurement  range radius= 0.3m (3*sigma) , bearing radius= 0.03 rad (3*sigma)

Factor graph:
	no prior on landmarks
	pose(x0)  prior(0.0, 0.0, 0.0)  priorNoise = Sigmas(0.02, 0.02, 0.01) // 2cm x, y, 0.01 rad bearing
	measurement  measurementNoise = Sigmas(0.01, 0.1)  // 0.01 rad std on bearing, 10cm on range


example2.pdf:Basic Parameters (cov_xy_m.txt cov_lm_m.txt) --good prior, no landmark
example3.pdf:Basic Parameters (cov_xy_m.txt box_m.txt)

example4.pdf: --bad prior no landmark
Pose at origin: prior(0.0, 0.0, 0.0);
priorNoise: Sigmas(0.2, 0.2, 0.01); // 20cm std on x,y, 0.01 rad on theta

example5.pdf: (cov_xy_mm.txt cov_lm_mm.txt, x_result.txt, lm_result.txt) --bad prior, good landmark
Pose at origin: prior(0.0, 0.0, 0.0);
priorNoise: Sigmas(0.2, 0.2, 0.01); // 20cm std on x,y, 0.01 rad on theta
landmark: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(0.1, 0.1) // 10cm x, y

example6.pdf: (cov_xy_mm1.txt cov_lm_mm1.txt box_m.txt) --bad prior, bad landmark
Pose at origin: prior(0.0, 0.0, 0.0);
priorNoise: Sigmas(0.2, 0.2, 0.01); // 20cm std on x,y, 0.01 rad on theta
landmark: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(1, 1) // 1m x, y

example7.pdf: (x_result_s1.txt lm_result_s1.txt cov_xy_m_s1.txt cov_lm_m_s1.txt) --good prior, no landmark, measurement sytematic error(1sigma)
range += 0.1m (range value by sensor)

example8.pdf: same as example7

example9.pdf: (x_result_s2.txt lm_result_s2.txt cov_xy_mm_s2.txt cov_lm_mm_s2.txt box_s2.txt) --bad prior, good landmark, measurement sytematic error(1sigma) [for both interval and factor graph]
Factor:
Pose at origin: prior(0.0, 0.0, 0.0);
priorNoise: Sigmas(0.2, 0.2, 0.01); // 20cm std on x,y, 0.01 rad on theta
landmark: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(0.1, 0.1) // 10cm x, y
range += 0.1m (range value by sensor)
Interval:
codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3) + 0.1; (range value by sensor)

example10.pdf: (x_result_s3.txt lm_result_s3.txt cov_xy_mm_s3.txt cov_lm_mm_s3.txt box_s3.txt) --bad prior, good landmark, measurement sytematic error(2sigma)
Factor:
Pose at origin: prior(0.0, 0.0, 0.0);
priorNoise: Sigmas(0.2, 0.2, 0.01); // 20cm std on x,y, 0.01 rad on theta
landmark: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(0.1, 0.1) // 10cm x, y
range += 0.2m (range value by sensor)
Interval:
codac::Interval l1_r = L1_D + codac::Interval(-0.3, 0.3) + 0.2; (range value by sensor)

example11.pdf: (x_result_u.txt lm_result_u.txt cov_xy_mm_u.txt cov_lm_mm_u.txt box_m.txt) --bad prior, good landmark, measurement uniform distribution(probabilistic)
Pose at origin: prior(0.0, 0.0, 0.0);
priorNoise: Sigmas(0.2, 0.2, 0.01); // 20cm std on x,y, 0.01 rad on theta
landmark: m_prior(a_map[k][0], a_map[k][1])  mapNoise = Sigmas(0.1, 0.1) // 10cm x, y

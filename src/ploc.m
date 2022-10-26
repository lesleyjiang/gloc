import gtsam.*

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have bearing and range information for measurements
%  - We have full odometry for measurements
%  - The robot and landmarks are on a grid, moving 2 meters each step
%  - Landmarks are 2 meters away from the robot trajectory

%% Create keys for variables
i1 = symbol('x',1); 
j1 = symbol('l',1); j2 = symbol('l',2); j3 = symbol('l',3); j4 = symbol('l',4);

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Add prior
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.02; 0.01; 0.01]);
graph.add(PriorFactorPose2(i1, priorMean, priorNoise));

%% Add bearing/range measurement factors
degrees = pi/180;
brNoise = noiseModel.Diagonal.Sigmas([0.1; 0.2]);
graph.add(BearingRangeFactor2D(i1, j1, Rot2(45*degrees), sqrt(8), brNoise));
graph.add(BearingRangeFactor2D(i1, j2, Rot2(0*degrees), 2, brNoise));
graph.add(BearingRangeFactor2D(i1, j3, Rot2(-45*degrees), sqrt(8), brNoise));
graph.add(BearingRangeFactor2D(i1, j4, Rot2(180*degrees), 2, brNoise));

% print
graph.print(sprintf('\nFull graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(i1, Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(j1, Point2(1.8, 2.1));
initialEstimate.insert(j2, Point2(2.3, 0.1));
initialEstimate.insert(j3, Point2(1.8, -2.1));
initialEstimate.insert(j4, Point2(-2.1, 0.2));

initialEstimate.print(sprintf('\nInitial estimate:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n'));

%% Plot Covariance Ellipses
cla;hold on

marginals = Marginals(graph, result);
plot2DTrajectory(result, [], marginals);
plot2DPoints(result, 'b', marginals);

%%plot([result.at(i1).x; result.at(j1).x],[result.at(i1).y; result.at(j1).y], 'c-');
%%plot([result.at(i2).x; result.at(j1).x],[result.at(i2).y; result.at(j1).y], 'c-');
%%plot([result.at(i3).x; result.at(j2).x],[result.at(i3).y; result.at(j2).y], 'c-');
axis([-2.6 4.8 -1 1])
axis equal
view(2)
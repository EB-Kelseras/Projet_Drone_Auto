import gtsam
import numpy as np

#####       C++ version     #####
# // Create an empty nonlinear factor graph
# NonlinearFactorGraph graph;

# // Add a Gaussian prior on pose x_1
# Pose2 priorMean(0.0, 0.0, 0.0);
# noiseModel::Diagonal::shared_ptr priorNoise =
#   noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
# graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

# // Add two odometry factors
# Pose2 odometry(2.0, 0.0, 0.0);
# noiseModel::Diagonal::shared_ptr odometryNoise =
#   noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
# graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
# graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));

# Create an empty NonlinearFactorGraph
graph = gtsam.NonlinearFactorGraph()

# Add a Gaussian prior on pose x_1
prior_mean = gtsam.Pose2(0.0, 0.0, 0.0)
prior_noise = gtsam.noiseModel.Diagonal.Sigmas([0.3, 0.3, 0.1])
graph.add(gtsam.PriorFactorPose2(1, prior_mean, prior_noise))

# Add two odometry factors
odometry = gtsam.Pose2(2.0, 0.0, 0.0)
odometry_noise = gtsam.noiseModel.Diagonal.Sigmas([0.2, 0.2, 0.1])
graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, odometry_noise))
graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, odometry_noise))

# print graph
graph.print("Factor Graph:\n")

#####       C++ code        #####
# // create (deliberately inaccurate) initial estimate
# Values initial;
# initial.insert(1, Pose2(0.5, 0.0, 0.2));
# initial.insert(2, Pose2(2.3, 0.1, -0.2));
# initial.insert(3, Pose2(4.1, 0.1, 0.1));

# // optimize using Levenberg-Marquardt optimization
# Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

# Create (deliberately inaccurate) initial estimate
initial = gtsam.Values()
initial.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
initial.insert(3, gtsam.Pose2(4.1, 0.1, 0.1))

# Optimize using Levenberg-Marquardt optimization
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial)
result = optimizer.optimize()

# Print the optimized result
initial.print("Initial Values:\n")
result.print("Optimized Result:\n")

#####       C++     #####
# // Query the marginals
# cout.precision(2);
# Marginals marginals(graph, result);
# cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
# cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
# cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

marginals = gtsam.Marginals(graph, result)
np.set_printoptions(precision=2)

cov_x1 = marginals.marginalCovariance(1)
cov_x2 = marginals.marginalCovariance(2)
cov_x3 = marginals.marginalCovariance(3)

print("x1 covariance:\n", cov_x1)
print("x2 covariance:\n", cov_x2)
print("x3 covariance:\n", cov_x3)
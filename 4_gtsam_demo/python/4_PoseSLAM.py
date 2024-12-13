import gtsam
import math

#####       C++ Code        #####
#
# NonlinearFactorGraph graph;
# noiseModel::Diagonal::shared_ptr priorNoise =
#   noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
# graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

# // Add odometry factors
# noiseModel::Diagonal::shared_ptr model =
#   noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
# graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0     ), model));
# graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));
# graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model));
# graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model));

# // Add the loop closure constraint
# graph.add(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model));

graph = gtsam.NonlinearFactorGraph()

prior_noise = gtsam.noiseModel.Diagonal.Sigmas([0.3, 0.3, 0.1])
graph.add(gtsam.PriorFactorPose2(1, gtsam.Pose2(0.0, 0.0, 0.0), prior_noise))

# Add odometry factors
model = gtsam.noiseModel.Diagonal.Sigmas([0.2, 0.2, 0.1])
graph.add(gtsam.BetweenFactorPose2(1, 2, gtsam.Pose2(2,0,0), model))
graph.add(gtsam.BetweenFactorPose2(2, 3, gtsam.Pose2(2,0,math.pi/2), model))
graph.add(gtsam.BetweenFactorPose2(3, 4, gtsam.Pose2(2,0,math.pi/2), model))
graph.add(gtsam.BetweenFactorPose2(4, 5, gtsam.Pose2(2,0,math.pi/2), model))

# Add the loop closure constraints
graph.add(gtsam.BetweenFactorPose2(2, 2, gtsam.Pose2(2,0,math.pi/2), model))

# print graph
graph.print("Factor Graph:\n")

# Create (deliberately inaccurate) initial estimate
initial = gtsam.Values()
initial.insert(1, gtsam.Pose2(0.5, -0.2, 0.2))
initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
initial.insert(3, gtsam.Pose2(4.1, 0.2, 0.1))
initial.insert(4, gtsam.Pose2(3.8, 2.1, -0.2))
initial.insert(5, gtsam.Pose2(1.9, 1.8, 0.1))

# Optimize using Levenberg-Marquardt optimization
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial)
result = optimizer.optimize()

# Print the optimized result
initial.print("Initial Values:\n")
result.print("Optimized Result:\n")

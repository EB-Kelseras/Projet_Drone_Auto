import gtsam
from gtsam.gtsam import BearingRangeFactor2D
from gtsam.gtsam import BetweenFactorPose2
import numpy as np

# Create graph container and add factors to it
graph = gtsam.NonlinearFactorGraph()

# Create keys for variables
i1 = gtsam.symbol('x',1)
i2 = gtsam.symbol('x',2)
i3 = gtsam.symbol('x',3)
j1 = gtsam.symbol('l',1)
j2 = gtsam.symbol('l',2)

# Add prior
priorMean = gtsam.Pose2(0.0, 0.0, 0.0) # prior at origin
priorNoise = gtsam.noiseModel.Diagonal.Sigmas([0.3, 0.3, 0.1])
# Add directly to graph
graph.add(gtsam.PriorFactorPose2(i1, priorMean, priorNoise))

# Add odometry
odometry = gtsam.Pose2(2.0, 0.0, 0.0)
odometryNoise = gtsam.noiseModel.Diagonal.Sigmas([0.2, 0.2, 0.1])
graph.add(BetweenFactorPose2(i1, i2, odometry, odometryNoise))
graph.add(BetweenFactorPose2(i2, i3, odometry, odometryNoise))

# Add bearing/range measurement factors
degrees = np.pi/180
brNoise = gtsam.noiseModel.Diagonal.Sigmas([0.1, 0.2])
graph.add(BearingRangeFactor2D(i1, j1, gtsam.Rot2(45*degrees), np.sqrt(8), brNoise))
graph.add(BearingRangeFactor2D(i2, j1, gtsam.Rot2(90*degrees), 2, brNoise))
graph.add(BearingRangeFactor2D(i3, j2, gtsam.Rot2(90*degrees), 2, brNoise))

# Print graph
graph.print("Factor Graph: \n")

# Create initial estimate
initial = gtsam.Values()
initial.insert(i1, gtsam.Pose2(0.5, -0.2, 0.2))
initial.insert(i2, gtsam.Pose2(2.3, 0.1, -0.2))
initial.insert(i3, gtsam.Pose2(4.1, 0.2, 0.1))
initial.insert(j1, gtsam.Point2(1.9, 1.8))
initial.insert(j2, gtsam.Point2(3.8, 2.1))

# Optimize using Levenberg-Marquardt optimization
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial)
result = optimizer.optimize()

# Print the optimized result
initial.print("Initial Values:\n")
result.print("Optimized Result:\n")
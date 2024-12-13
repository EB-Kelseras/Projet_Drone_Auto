import rclpy
import gtsam
import numpy as np

def main(args=None):
    rclpy.init(args=args)

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

    marginals = gtsam.Marginals(graph, result)
    np.set_printoptions(precision=2)

    cov_x1 = marginals.marginalCovariance(1)
    cov_x2 = marginals.marginalCovariance(2)
    cov_x3 = marginals.marginalCovariance(3)

    print("x1 covariance:\n", cov_x1)
    print("x2 covariance:\n", cov_x2)
    print("x3 covariance:\n", cov_x3)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
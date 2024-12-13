import rclpy
import gtsam
import numpy as np
from typing import List, Optional
from functools import partial

def error_unary_factor(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[np.ndarray[np.float64[2, 3]]]) -> np.ndarray:
    
    key = this.keys()[0]
    estimate = values.atPose2(key)
    # Get the rotation part of Pose2
    R = estimate.rotation()

    if jacobians is not None:
        # Compute the Jacobian matrix
        h_matrix = np.array([
            [R.c(), -R.s(), 0.0],
            [R.s(), R.c(), 0.0]
        ])
        jacobians[:, :] = h_matrix

    error = np.array([estimate.x() - measurement[0], estimate.y() - measurement[1]])
    return error # with input types this is a 1D np.ndarray

def main(args=None):
    rclpy.init(args=args)

    # Create a diagonal noise model with 10 cm standard deviation for x and y
    unary_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))

    # Create an empty factor graph
    graph = gtsam.NonlinearFactorGraph()

    # Add unary measurement factors (e.g., GPS) on all three poses
    factor = gtsam.CustomFactor(unary_noise, [1], 
                                partial(error_unary_factor, np.array([0.0, 0.0])))
    graph.add(factor)  # Pose 1 at (0.0, 0.0)

    factor = gtsam.CustomFactor(unary_noise, [2], 
                                partial(error_unary_factor, np.array([2.0, 0.0])))
    graph.add(factor)  # Pose 2 at (2.0, 0.0)

    factor = gtsam.CustomFactor(unary_noise, [3], 
                                partial(error_unary_factor, np.array([4.0, 0.0])))
    graph.add(factor)  # Pose 3 at (4.0, 0.0)

    # Display graph
    graph.print("Factor graph :\n")

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

    

    rclpy.shutdown()


if __name__ == '__main__':
    main()
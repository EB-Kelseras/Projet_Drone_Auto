import gtsam
import numpy as np
from typing import List, Optional
from functools import partial

### NOT WORKING YET ###

#####       C++ code        #####
# class UnaryFactor: public NoiseModelFactor1<Pose2> {
#   double mx_, my_; ///< X and Y measurements
# public:
#   UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
#     NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}
#   Vector evaluateError(const Pose2& q,
#                        boost::optional<Matrix&> H = boost::none) const
#   {
#     const Rot2& R = q.rotation();
#     if (H) (*H) = (gtsam::Matrix(2, 3) <<
#             R.c(), -R.s(), 0.0,
#             R.s(), R.c(), 0.0).finished();
#     return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
#   }
# };

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

# # Define the custom unary factor
# class UnaryFactor(gtsam.CustomFactor):
#     def __init__(self, key, x, y, model):
#         # Initialize the CustomFactor with 1 key (unary factor)
#         super().__init__(model, [key])
#         self.mx_ = x
#         self.my_ = y

#     # Define the error function
#     def evaluateError(self, q, H=None):
#         # Get the rotation part of Pose2
#         R = q.rotation()

#         if H is not None: 
#             # Compute the Jacobian matrix
#             H_matrix = np.array([
#                 [R.c(), -R.s(), 0.0],
#                 [R.s(), R.c(), 0.0]
#             ])
#             H[:, :] = H_matrix

#         # Compute the error vector
#         error = np.array([q.x() - self.mx_, q.y() - self.my_])
#         return error


#####       C++ code        #####
# // add unary measurement factors, like GPS, on all three poses
# noiseModel::Diagonal::shared_ptr unaryNoise =
#  noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
# graph.add(boost::make_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise));
# graph.add(boost::make_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise));
# graph.add(boost::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));

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


# # Example usage
# if __name__ == "__main__":
#     # Create a noise model
#     model = noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))  # Small uncertainty for x, y, theta

#     # Create a prior Pose2
#     prior_pose = Pose2(2.0, 3.0, np.pi / 4)

#     # Add the custom unary factor to the factor graph
#     graph = gtsam.NonlinearFactorGraph()
#     initial_pose_key = 1
#     graph.add(UnaryFactor(initial_pose_key, prior_pose, model))

#     # Provide an initial estimate for the pose
#     initial_estimate = gtsam.Values()
#     initial_estimate.insert(initial_pose_key, Pose2(1.5, 2.5, np.pi / 6))

#     # Optimize the factor graph
#     optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
#     result = optimizer.optimize()

#     # Print the optimized pose
#     optimized_pose = result.atPose2(initial_pose_key)
#     print("Optimized Pose2:", optimized_pose)

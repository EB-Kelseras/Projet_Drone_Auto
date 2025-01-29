cmake_minimum_required(VERSION 3.10)

project(GTSAMTest)

# Find GTSAM (make sure GTSAM is installed in the container)
find_package(GTSAM REQUIRED)

# Add the executable
add_executable(GTSAMTest GTSAMTest.cpp)

# Link GTSAM to your executable
target_link_libraries(GTSAMTest gtsam)
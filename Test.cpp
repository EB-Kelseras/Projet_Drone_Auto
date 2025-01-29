#include <gtsam/linear/LinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <iostream>

using namespace gtsam;
using namespace std;

int main() {
    // Create a factor graph
    LinearFactorGraph graph;

    // Add a prior factor: x1 = 0 (noise model: variance 0.01)
    graph.add(0, I_1x1, Vector1(0), noiseModel::Isotropic::Sigma(1, 0.1));

    // Add a constraint: x2 = x1 + 1 (noise model: variance 0.01)
    graph.add(0, -I_1x1, 1, I_1x1, Vector1(1), noiseModel::Isotropic::Sigma(1, 0.1));

    // Solve the factor graph
    GaussianFactorGraph::shared_ptr gfg = graph.linearize();
    VectorValues solution = gfg->optimize();

    // Print the solution
    cout << "Solution:" << endl;
    solution.print();

    return 0;
}
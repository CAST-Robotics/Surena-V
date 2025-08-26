#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <iostream>

/*
    Minimize: 0.5 * x^T * H * x + g^T * x
    Subject to: l <= A * x <= u
    Where:
        H is the hessian matrix (2x2 sparse matrix with values [1.0, 0; 0, 1.0])
        g is the gradient vector (-1.0, -1.0)
        A is the linear constraints matrix (2x2 sparse matrix with values [1.0, 0; 0, 1.0])
        l is the lower bound vector (0.0, 0.0)
        u is the upper bound vector (10.0, 10.0)
*/

int main() {
    // Define a simple QP problem
    Eigen::SparseMatrix<double> hessian(2, 2);
    hessian.insert(0, 0) = 1.0;
    hessian.insert(1, 1) = 1.0;

    Eigen::VectorXd gradient(2);
    gradient << -1.0, -1.0;

    Eigen::SparseMatrix<double> linearMatrix(2, 2);
    linearMatrix.insert(0, 0) = 1.0;
    linearMatrix.insert(1, 1) = 1.0;

    Eigen::VectorXd lowerBound(2);
    lowerBound << 0.0, 0.0;

    Eigen::VectorXd upperBound(2);
    upperBound << 10.0, 10.0;

    // Instantiate the solver
    OsqpEigen::Solver solver;
    
    // Set up the QP data
    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(2);

    if (!solver.data()->setHessianMatrix(hessian)) return 1;
    if (!solver.data()->setGradient(gradient)) return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
    if (!solver.data()->setLowerBound(lowerBound)) return 1;
    if (!solver.data()->setUpperBound(upperBound)) return 1;

    // Initialize the solver
    if (!solver.initSolver()) return 1;

    // Solve the QP problem
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

    // Get the solution
    Eigen::VectorXd solution = solver.getSolution();

    // Output or use the solution as needed
    std::cout << "Solution:\n" << solution << std::endl;
    return 0;
}
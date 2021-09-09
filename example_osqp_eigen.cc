// osqp-eigen
#include "ThirdParty/osqp_eigen/OsqpEigen/OsqpEigen.h"

using namespace Eigen;
using namespace OsqpEigen;

int main()
{
    /*
     * @min_x 0.5 * x'Px + q'x
     * @s.t.  l <= Ax <= u
     * @objective_matrix: p
     * @objective_vector: q
     * @constraint_matrix: A
     * @lower_bounds: l
     * @upper_bounds: u
     */

    // Define problem data
    // P
    Eigen::SparseMatrix<double> P(2, 2);
    const Eigen::Triplet<double> p[] = {{0, 0, 4},
                                        {0, 1, 1},
                                        {1, 0, 1},
                                        {1, 1, 2}};
    P.setFromTriplets(std::begin(p),
                      std::end(p));

    // q
    Eigen::VectorXd q(2);
    q << 1, 1;

    // A
    Eigen::SparseMatrix<double> A(3, 2);
    const Eigen::Triplet<double> a[] = {{0, 0, 1},
                                        {0, 1, 1},
                                        {1, 0, 1},
                                        {1, 1, 0},
                                        {2, 0, 0},
                                        {2, 1, 1}};
    A.setFromTriplets(std::begin(a),
                      std::end(a));

    // l and u
    Eigen::VectorXd l(3);
    Eigen::VectorXd u(3);
    l << 1, 0, 0;
    u << 1, 0.7, 0.7;

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    // solver.settings()->setVerbosity(false);
    // solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-04);
    solver.settings()->setMaxIteration(1e4);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(2);   // col of A
    solver.data()->setNumberOfConstraints(3); // row of A
    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    // instantiate the solver
    solver.initSolver();

    // solve the QP problem
    solver.solve();

    // get solution
    Eigen::VectorXd QPSolution;
    QPSolution = solver.getSolution();
    std::cout << "QPSolution: " << std::endl
              << QPSolution << std::endl;
    return 0;
}
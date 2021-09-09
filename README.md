# **osqp-eigen-bazel**

![LICENSE](https://img.shields.io/badge/license-GPL%203.0-brightgreen)
![Build Status](https://img.shields.io/badge/build-passing-blue)

The `osqp-cpp` is built by `bazel`, which is easy to use `osqp` for rookies.

![](http://latex.codecogs.com/svg.latex?\begin{aligned}minimize\quad&\frac{1}{2}x^T\begin{bmatrix}4&1\\\\1&2\end{bmatrix}x+\begin{bmatrix}1\\\\1\end{bmatrix}^Tx\\\\s.t.\quad&\begin{bmatrix}1\\\\0\\\\0\end{bmatrix}\le\begin{bmatrix}1&1\\\\1&0\\\\0&1\end{bmatrix}x\le\begin{bmatrix}1\\\\0.7\\\\0.7\end{bmatrix}\end{aligned})

We show below how to solve the problem in C++.

```c++
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
```

## Tutorial

1. Set `bazel` sync.(Need to install `bazel` before)

```bash
bazel sync
```

2. Build the code.

```bash
bazel build //:example_osqp_eigen
```

3. Run the code.

```bash
bazel run //:example_osqp_eigen
```

Terminal print like this

```bash
-----------------------------------------------------------------
           OSQP v0.0.0  -  Operator Splitting QP Solver
              (c) Bartolomeo Stellato,  Goran Banjac
        University of Oxford  -  Stanford University 2021
-----------------------------------------------------------------
problem:  variables n = 2, constraints m = 3
          nnz(P) + nnz(A) = 9
settings: linear system solver = qdldl,
          eps_abs = 1.0e-04, eps_rel = 1.0e-03,
          eps_prim_inf = 1.0e-04, eps_dual_inf = 1.0e-04,
          rho = 1.00e-01 (adaptive),
          sigma = 1.00e-06, alpha = 1.60, max_iter = 10000
          check_termination: on (interval 25),
          scaling: on, scaled_termination: off
          warm start: on, polish: off, time_limit: off

iter   objective    pri res    dua res    rho        time
   1  -7.8808e-03   1.01e+00   2.00e+02   1.00e-01   1.68e-05s
  50   1.8800e+00   1.90e-04   3.80e-06   1.00e-01   2.93e-05s

status:               solved
number of iterations: 50
optimal objective:    1.8800
run time:             3.25e-05s
optimal rho estimate: 1.38e+00

QPSolution: 
0.30019
0.69981
```

## Link

- [osqp](https://github.com/osqp/osqp)
- [osqp-eigen](https://github.com/robotology/osqp-eigen)
- [eigen](https://gitlab.com/libeigen/eigen)

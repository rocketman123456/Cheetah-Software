
#include "OsqpTest.h"
#include "OsqpEigen/Data.hpp"
#include "OsqpEigen/Settings.hpp"
#include "OsqpEigen/Solver.hpp"
#include "Timer.h"
#include <eigen3/Eigen/SparseCore>

Vector<double> testOSQP(QpProblem<double>& problem)
{
    OsqpEigen::Solver solver;

    solver.data()->setNumberOfVariables(problem.n);
    solver.data()->setNumberOfConstraints(problem.m);

    solver.data()->setLowerBound(problem.l);
    solver.data()->setUpperBound(problem.u);
    solver.settings()->setScaling(0);
    solver.settings()->setAdaptiveRho(false);
    solver.settings()->setRho(7);
    solver.settings()->setWarmStart(false);

    Timer                       t;
    Eigen::SparseMatrix<double> pSparse = problem.P.sparseView();
    Eigen::SparseMatrix<double> aSparse = problem.A.sparseView();
    solver.data()->setHessianMatrix(pSparse);
    solver.data()->setGradient(problem.q);
    solver.data()->setLinearConstraintsMatrix(aSparse);

    solver.initSolver();
    solver.solve();
    fprintf(stderr, "%.3f\n", t.getMs());

    return solver.getSolution();
}

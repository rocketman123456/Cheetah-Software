#ifndef QPSOLVER_QPOASES_H
#define QPSOLVER_QPOASES_H

#include "QpProblem.h"
#include "types.h"
#include <qpOASES.hpp>

Vector<double> testQPOASES(QpProblem<double>& problem);

#endif // QPSOLVER_QPOASES_H

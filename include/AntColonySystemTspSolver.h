#ifndef ANTCOLONYSYSTEMTSPSOLVER_H
#define ANTCOLONYSYSTEMTSPSOLVER_H

#include "TspSolver.h"

class AntColonySystemTspSolver : public TspSolver {

public:

    /**
     * {@inheritDoc}
     */
    virtual std::vector<Position2D> solve(
        const std::vector<Position2D>& points
    );

};

#endif
#ifndef NEARESTNEIGHBOURTSPSOLVER_H
#define NEARESTNEIGHBOURTSPSOLVER_H

#include "TspSolver.h"

class NearestNeighbourTspSolver : public TspSolver {

public:

    /**
     * {@inheritDoc}
     */
    virtual std::vector<Position2D> solve(
        const std::vector<Position2D>& points
    );

};

#endif
#ifndef ANTCOLONYSYSTEM_H
#define ANTCOLONYSYSTEM_H

#include "TspSolver.h"

class AntColonySystem : public TspSolver {

public:

    /**
     * {@inheritDoc}
     */
    virtual std::vector<Position2D> solve(
        const std::vector<Position2D>& points
    );

};

#endif
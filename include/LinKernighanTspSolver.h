#ifndef LINKERNIGHANTSPSOLVER_H
#define LINKERNIGHANTSPSOLVER_H

#include "TspSolver.h"

/**
 * A TSP solver which uses the Lin-Kernighan heuristic [1].
 * [1]: Lin, S. and Kernighan, B.W., 1973. An effective heuristic algorithm for
 * the traveling-salesman problem. Operations research, 21(2), pp.498-516.
 */
class LinKernighanTspSolver : public TspSolver {

public:

    /**
     * {@inheritDoc}
     */
    virtual std::vector<Position2D> solve(
        const std::vector<Position2D>& points
    );

};

#endif
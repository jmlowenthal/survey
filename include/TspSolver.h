#ifndef TSPSOLVER_H
#define TSPSOLVER_H

#include <vector>
#include "Position2D.h"

/**
 * Producer of solutions for the TSP.
 */
class TspSolver {
public:
    /**
     * Produces a solution to the Traveling Salesperson Problem. The
     * implementation is permitted to produce an optimal or approximate
     * solution, and it may use the provided ordering of points as an initial
     * tour upon which a solution (possibly only a local optimum) is built.
     * @param points    The set of points to operate over (may be used as an
     *                  initial solution).
     * @return          An optimal or approximate solution to the TSP.
     */
    virtual std::vector<Position2D> solve(
        const std::vector<Position2D>& points
    ) = 0;
};

#endif
#ifndef RANDOMTSPSOLVER_H
#define RANDOMTSPSOLVER_H

#include "TspSolver.h"
#include <random>

class RandomTspSolver : public TspSolver {

private:

    std::default_random_engine _engine;

public:

    RandomTspSolver(int seed);

    RandomTspSolver();

    virtual std::vector<Position2D> solve(
        const std::vector<Position2D>& points
    );

};

#endif
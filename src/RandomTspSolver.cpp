#include "RandomTspSolver.h"

RandomTspSolver::RandomTspSolver(int seed) {
    _engine = std::default_random_engine(seed);
}

RandomTspSolver::RandomTspSolver() {
    std::random_device dev;
    _engine = std::default_random_engine(dev());
}

std::vector<Position2D> RandomTspSolver::solve(
    const std::vector<Position2D>& points
) {
    std::vector<Position2D> p = points;
    std::vector<Position2D> res(p.size());
    for (int i = 0; i < points.size(); ++i) {
        int j = p.size() - 1;
        std::uniform_int_distribution<int> d(0, j);
        int k = d(_engine);
        Position2D tmp = p[k];
        p[k] = p[j];
        p[j] = tmp;
        res.push_back(p[j]);
        p.pop_back();
    }
    return res;
}
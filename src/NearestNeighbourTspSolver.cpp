#include "NearestNeighbourTspSolver.h"
#include <stack>

std::vector<Position2D> NearestNeighbourTspSolver::solve(
    const std::vector<Position2D>& points
) {
    if (points.size() < 1) {
        return points;
    }
    
    std::vector<Position2D> to_visit(points);
    std::vector<Position2D> tour;
    tour.reserve(points.size());
    
    Position2D current = to_visit[to_visit.size() - 1];
    tour.push_back(current);
    to_visit.pop_back();

    while (!to_visit.empty()) {
        std::vector<Position2D>::iterator best;
        float best_d = __FLT_MAX__;
        for (auto itr = to_visit.begin(); itr != to_visit.end(); ++itr) {
            float d = Position2D::distance(current, *itr);
            if (d < best_d) {
                best = itr;
                best_d = d;
            }
        }
        tour.push_back(*best);
        current = *best;
        to_visit.erase(best);
    }

    return tour;
}
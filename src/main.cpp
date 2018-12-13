#include "AntColonySystem.h"
#include <stdio.h>

int main() {
    std::vector<Position2D> points = {
        Position2D(3, 0),
        Position2D(0, 0),
        Position2D(2, 0),
        Position2D(1, 0)
    };
    std::vector<Position2D> desired = {
        Position2D(0, 0),
        Position2D(1, 0),
        Position2D(2, 0),
        Position2D(3, 0),
    };
    AntColonySystem solver;
    std::vector<Position2D> solution = solver.solve(points);
    return 0;
}
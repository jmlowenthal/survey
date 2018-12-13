#include "catch.hpp"
#include "AntColonySystemTspSolver.h"
#include "TourMatcher.h"

TEST_CASE("[ACS] Simple linear 4-node graph", "[full]") {
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
    AntColonySystemTspSolver solver;
    std::vector<Position2D> solution = solver.solve(points);
    REQUIRE_THAT(solution, TourEqual(desired));
}

TEST_CASE("[ACS] Circular graph", "[full]") {
    const int N = 8;
    std::vector<Position2D> points;
    std::vector<Position2D> desired;
    for (int i = 0; i < N; ++i) {
        float theta = ((float)i) / ((float)N);
        desired.push_back(Position2D(cosf(theta), sinf(theta)));
    }

    SECTION("Rotated") {
        int order[]{ 3, 4, 5, 6, 7, 0, 1, 2 };
        for (int i = 0; i < N; ++i) {
            float theta = ((float)order[i]) / ((float) N);
            points.push_back(Position2D(cosf(theta), sinf(theta)));
        }
        AntColonySystemTspSolver solver;
        std::vector<Position2D> solution = solver.solve(points);
        REQUIRE_THAT(solution, TourEqual(desired));
    }

    SECTION("Reordered") {
        int order[]{ 4, 7, 5, 1, 3, 0, 2, 6 };
        for (int i = 0; i < N; ++i) {
            float theta = ((float)order[i]) / ((float) N);
            points.push_back(Position2D(cosf(theta), sinf(theta)));
        }
        AntColonySystemTspSolver solver;
        std::vector<Position2D> solution = solver.solve(points);
        REQUIRE_THAT(solution, TourEqual(desired));
    }
}
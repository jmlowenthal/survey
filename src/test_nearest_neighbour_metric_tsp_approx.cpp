#include "catch.hpp"
#include "nearest_neighbour_metric_tsp_approx.h"
#include "simple_point.h"
#include "TourMatcher.h"
#include <bits/stl_iterator.h>

using namespace boost;
typedef simple_point<float> pos;
typedef adjacency_list<vecS, vecS, directedS, pos> G;
typedef graph_traits<G>::vertex_descriptor V;

TEST_CASE("[NN] Simple linear 4-node graph", "[full]") {
    G g;
    V a = add_vertex({3, 0}, g);
    V b = add_vertex({0, 0}, g);
    V c = add_vertex({2, 0}, g);
    V d = add_vertex({1, 0}, g);
    std::vector<V> desired = { b, d, c, a };
    std::vector<V> solution;
    nearest_neighbour_metric_tsp_approx(g, std::back_inserter(solution));
    REQUIRE_THAT(solution, TourEqual(desired));
}

TEST_CASE("[NN] Circular graph", "[full]") {
    const int N = 8;
    G g;
    std::vector<V> desired(N);

    SECTION("Rotated") {
        int order[]{ 3, 4, 5, 6, 7, 0, 1, 2 };
        for (int i = 0; i < N; ++i) {
            float theta = 2 * M_PI * ((float)order[i]) / ((float) N);
            V v = add_vertex({cosf(theta), sinf(theta)}, g);
            desired[order[i]] = v;
        }
        std::vector<V> solution;
        nearest_neighbour_metric_tsp_approx(g, std::back_inserter(solution));
        REQUIRE_THAT(solution, TourEqual(desired));
    }

    SECTION("Reordered") {
        int order[]{ 4, 7, 5, 1, 3, 0, 2, 6 };
        for (int i = 0; i < N; ++i) {
            float theta = 2 * M_PI * ((float)order[i]) / ((float) N);
            V v = add_vertex({cosf(theta), sinf(theta)}, g);
            desired[order[i]] = v;
        }
        std::vector<V> solution;
        nearest_neighbour_metric_tsp_approx(g, std::back_inserter(solution));
        REQUIRE_THAT(solution, TourEqual(desired));
    }
}
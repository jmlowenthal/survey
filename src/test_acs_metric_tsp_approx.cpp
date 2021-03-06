#include "catch.hpp"
#include "acs_metric_tsp_approx.h"
#include "simple_point.h"
#include "TourMatcher.h"
#include <bits/stl_iterator.h>
#include <boost/container_hash/hash.hpp>

using namespace boost;
using namespace acs;
typedef simple_point<float> pos;
typedef adjacency_list<vecS, vecS, bidirectionalS, pos> G;
typedef graph_traits<G>::vertex_descriptor V;

TEST_CASE("[ACS] Simple linear 4-node graph", "[full]") {
    G g;
    V a = add_vertex({3, 0}, g);
    V b = add_vertex({0, 0}, g);
    V c = add_vertex({2, 0}, g);
    V d = add_vertex({1, 0}, g);
    std::vector<V> desired = { b, d, c, a };
    std::vector<V> solution;
    auto itr = std::back_inserter(solution);
    auto vis = boost::make_tsp_tour_visitor(itr);
    acs_metric_tsp_approx(g, vis);
    REQUIRE_THAT(solution, TourEqual(desired));
}

TEST_CASE("[ACS] Circular graph", "[full]") {
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
        auto itr = std::back_inserter(solution);
        auto vis = boost::make_tsp_tour_visitor(itr);
        acs_metric_tsp_approx(g, vis);        
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
        auto itr = std::back_inserter(solution);
        auto vis = boost::make_tsp_tour_visitor(itr);
        acs_metric_tsp_approx(g, vis);
        REQUIRE_THAT(solution, TourEqual(desired));
    }
}

TEST_CASE("[ACS] Restricted simple linear graph", "[full]") {
    G g;
    V a = add_vertex({4, 0}, g);
    V b = add_vertex({0, 0}, g);
    V c = add_vertex({2, 0}, g);
    V d = add_vertex({1, 0}, g);
    add_edge(c, d, g);
    std::vector<V> desired = { b, c, d, a };
    std::vector<V> solution;
    auto itr = std::back_inserter(solution);
    auto vis = boost::make_tsp_tour_visitor(itr);
    acs_metric_tsp_approx(g, vis, _sop=true);
    REQUIRE_THAT(solution, TourEqual(desired));
}
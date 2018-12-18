#include "catch.hpp"
#include "acs_metric_sop_approx.h"
#include "simple_point.h"
#include "TourMatcher.h"
#include <bits/stl_iterator.h>

using namespace boost;
typedef simple_point<float> pos;
typedef adjacency_list<vecS, vecS, directedS, pos> G;
typedef graph_traits<G>::vertex_descriptor V;

TEST_CASE("[ACS-SOP] Unrestricted simple linear graph", "[full]") {
    G g;
    V a = add_vertex({4, 0}, g);
    V b = add_vertex({0, 0}, g);
    V c = add_vertex({2, 0}, g);
    V d = add_vertex({1, 0}, g);
    std::vector<V> desired = { b, d, c, a };
    std::vector<V> solution;
    auto itr = std::back_inserter(solution);
    auto vis = boost::make_tsp_tour_visitor(itr);
    acs_metric_sop_approx(g, vis);
    REQUIRE_THAT(solution, TourEqual(desired));
}

TEST_CASE("[ACS-SOP] Restricted simple linear graph", "[full]") {
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
    acs_metric_sop_approx(g, vis);
    REQUIRE_THAT(solution, TourEqual(desired));
}
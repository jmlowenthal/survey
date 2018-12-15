#include <stdio.h>
#include <vector>
#include "nearest_neighbour_metric_tsp_approx.h"
#include "EuclideanDistanceFunctor.h"
#include <boost/graph/metric_tsp_approx.hpp>
#include <boost/graph/simple_point.hpp>
#include <boost/property_map/function_property_map.hpp>

using namespace boost;
typedef adjacency_list<listS, vecS, directedS, simple_point<float>> G;
typedef G::vertex_descriptor V;

int main() {
    G g;
    V a = add_vertex({0.0f, 1.0f}, g);
    add_vertex({0.0f, 4.0f}, g);
    add_vertex({0.0f, 2.0f}, g);
    add_vertex({0.0f, 3.0f}, g);
    std::vector<V> tour;
    nearest_neighbour_metric_tsp_approx_from_vertex(
        g, a, std::back_inserter(tour)
    );
    for (V& p : tour) {
        printf("(%f, %f)\n", g[p].x, g[p].y);
    }
    return 0;
}
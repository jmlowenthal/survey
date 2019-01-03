#include "catch.hpp"
#include "ada_star_search.h"
#include "simple_point.h"
#include "EuclideanDistanceFunctor.h"
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <algorithm>

using namespace boost;
typedef simple_point<float> pos;
typedef adjacency_list<vecS, vecS, bidirectionalS, pos> G;
typedef graph_traits<G>::vertex_descriptor V;

TEST_CASE("[ADA] Optimality", "[full]") {
    random::mt11213b generator(0);
    G g;
    generate_random_graph(g, 100, 500, generator);
    V source = random_vertex(g, generator);
    V destination = random_vertex(g, generator);

    EuclideanDistanceFunctor<G, float> weight_map(g);
    std::map<V, V> pred_map_map;
    associative_property_map<std::map<V, V>> pred_map(pred_map_map);
    dijkstra_shortest_paths(
        g,
        source,
        predecessor_map(pred_map).
        weight_map(weight_map)
    );
    
    std::vector<V> desired;
    V current = destination;
    while (current != source) {
        desired.push_back(current);
        current = pred_map[current];
    }
    desired.push_back(current);
    std::reverse(desired.begin(), desired.end());

    REQUIRE(desired.size() > 0);

    associative_property_map<std::map<V, float>> g_map = make_g(
        source,
        destination
    );
    
    ada_star_search(
        g,
        source,
        destination,
        _weight_map=weight_map,
        _g=g_map
    );

    std::vector<V> solution;
    current = source;
    while (current != destination) {
        solution.push_back(current);
        current = ada_star_next_step(
            g,
            current,
            weight_map,
            g_map
        );
    }
    solution.push_back(current);

    REQUIRE(solution.size() > 0);
    REQUIRE(solution == desired);

}
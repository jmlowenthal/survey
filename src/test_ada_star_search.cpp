#include "catch.hpp"
#include "ada_star_search.h"
#include "simple_point.h"
#include "EuclideanDistanceFunctor.h"
#include <boost/graph/random.hpp>
#include <boost/graph/circle_layout.hpp>
#include <boost/random.hpp>
#include <boost/graph/astar_search.hpp>
#include <algorithm>
#include <cmath>

using namespace boost;
using namespace ada_star;
typedef simple_point<float> pos;
typedef adjacency_list<vecS, vecS, bidirectionalS, pos> G;
typedef graph_traits<G>::vertex_descriptor V;

TEST_CASE("[ADA] Path tests", "[full]") {

    // Generate graph
    random::mt11213b generator(0);
    G g;
    const int N = 10000;
    const float radius = 10.0f;
    V prev = add_vertex({ 0.0f, radius }, g);
    for (int i = 1; i < N; ++i) {
        float theta = (float)i * 2 * M_PI / N;
        pos p = { sin(theta) * radius, cos(theta) * radius };
        V next = add_vertex(p, g);
        add_edge(prev, next, g);
        prev = next;
    }
    for (int i = 0; i < 1000; ++i) {
        V u = random_vertex(g, generator);
        V v = random_vertex(g, generator);
        add_edge(u, v, g);
    }
    V source = random_vertex(g, generator);
    V destination = random_vertex(g, generator);

    // Initialise helpers
    EuclideanDistanceFunctor<G, float> weight_map(g);
    
    // Build optimal solution
    std::map<V, V> pred_map_map;
    associative_property_map<std::map<V, V>> pred_map(pred_map_map);

    std::vector<V> desired;
    BENCHMARK("Baseline A*") {
        astar_search(
            g,
            source,
            [&weight_map, &destination](V v){
                return weight_map[{ v, destination }];
            },
            predecessor_map(pred_map).
            weight_map(weight_map)
        );
        
        V current = destination;
        while (current != source) {
            desired.push_back(current);
            current = pred_map[current];
        }
        desired.push_back(current);
        std::reverse(desired.begin(), desired.end());
    }

    float desiredLength = 0.0f;
    for (int i = 0; i + 1 < desired.size(); ++i) {
        desiredLength += get(weight_map, { desired[i], desired[i + 1] });
    }

    SECTION("Optimal") {
        map_property_map<V, float> g_map = make_g(source, destination);
        
        std::vector<V> solution;
        BENCHMARK("Optimal path") {
            ada_star_search(
                g,
                source,
                destination,
                _weight_map=weight_map,
                _g=g_map
            );

            V current = source;
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
        }

        REQUIRE(solution.size() > 0);

        float solutionLength = 0.0f;
        for (int i = 0; i + 1 < solution.size(); ++i) {
            solutionLength += get(weight_map, { solution[i], solution[i + 1] });
        }

        REQUIRE(solutionLength == desiredLength);
    }

    SECTION("Suboptimal (ε = 1.5)") {
        map_property_map<V, float> g_map = make_g(source, destination);

        std::vector<V> solution;
        BENCHMARK("Suboptimal path") {
            ada_star_search(
                g,
                source,
                destination,
                _weight_map=weight_map,
                _g=g_map,
                _suboptimality=1.5f
            );

            V current = source;
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
        }

        REQUIRE(solution.size() > 0);

        float solutionLength = 0.0f;
        for (int i = 0; i + 1 < solution.size(); ++i) {
            solutionLength += get(weight_map, { solution[i], solution[i + 1] });
        }

        REQUIRE(desiredLength <= solutionLength);
        REQUIRE(solutionLength <= 1.5f * desiredLength);
    }

}
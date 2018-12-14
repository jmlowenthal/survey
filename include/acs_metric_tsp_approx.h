#ifndef ACS_METRIC_TSP_APPROX_H
#define ACS_METRIC_TSP_APPROX_H

#ifndef ACS_RESOLUTION
#define ACS_RESOLUTION float
#endif

#include <boost/graph/metric_tsp_approx.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <algorithm>
#include <utility>
#include <cmath>

#include "EuclideanDistanceFunctor.h"
#include "tour_distance.h"
#include "min_element_by.h"

/*
    float len = 0.0f;
    for (int i = 0; i < points.size() - 1; ++i) {
        len += Position2D::distance(points[i], points[i + 1]);
    }
    _tau_zero = 1.0f / (len * points.size
*/

template<
    typename VertexListGraph,
    typename WeightMap,
    typename PheromoneMap,
    typename TSPVertexVisitor>
void acs_metric_tsp_approx_from_vertex(
    const VertexListGraph& g,
    typename boost::graph_traits<VertexListGraph>::vertex_descriptor start,
    WeightMap weight_map,
    PheromoneMap pheromone_map,
    TSPVertexVisitor vis,
    const int num_ants = 100,
    const int iterations = 100,
    const float beta = 2.0f,
    const float q0 = 0.9f,
    const float p = 0.1f,
    const float a = 0.1f,
    const float tau_zero = 0.0f,
    const bool closed_tour = false
) {
    using namespace boost;

    BOOST_CONCEPT_ASSERT((VertexListGraphConcept<VertexListGraph>));
    BOOST_CONCEPT_ASSERT((TSPVertexVisitorConcept<TSPVertexVisitor, VertexListGraph>));

    typedef typename graph_traits<VertexListGraph>::vertex_descriptor V;
    typedef typename graph_traits<VertexListGraph>::vertex_iterator VItr;

    const int num_points = num_vertices(g);

    const random::mt11213b generator;

    for (int iteration = 0; iteration < iterations; ++iteration) {

        std::vector<std::vector<V>> ants(num_ants);
        for (int i = 0; i < num_ants; ++i) {
            ants[i].reserve(num_points);
            ants[i].push_back(random_vertex(g, generator));
        }
        for (int step = 0; step < num_points - 1; ++step) {
            for (int i = 0; i < num_ants; ++i) {

                // Initialisation and auxilary computation
                std::vector<V>& ant = ants[i];
                std::map<V, float> probs;
                float normalising = 0.0f;
                V current = ant[ant.size() - 1];
                V best = current;
                for (V j : vertices(g)) {
                    if (std::find(ant.begin(), ant.end(), j) != ant.end()) {
                        continue;
                    }
                    std::pair<V, V> edge(current, j);
                    float nu = 1.0f / weight_map[edge];
                    float prob = pheromone_map[edge] * powf(nu, beta);
                    probs.set(j, prob);
                    normalising += prob;
                    if (best == current || prob > probs.get(best)) {
                        best = j;
                    }
                }

                // State transition rule
                float q = ((float)rand()) / __INT_MAX__;
                V j;
                if (q <= q0) {
                    BOOST_ASSERT(best != current);
                    j = best;
                }
                else {
                    float choice = ((float)rand()) / __INT_MAX__ * normalising;
                    for (j : vertices(g)) {
                        if (choice <= probs.get(j)) {
                            break;
                        }
                        choice -= probs.get(j);
                    }
                }
                ant.push_back(j);

                // Local updating rule
                std::pair<V, V> edge(current, j);
                pheromone_map[edge] = (1 - p) * pheromone_map[edge] + p * tau_zero;

            }

            // Find the best tour
            auto tour_distance = closed_tour ? closed_tour_distance : open_tour_distance;
            std::vector<V>& best_tour = min_element_by(
                ants.begin(),
                ants.end(),
                tour_distance
            );

            // Global updating rule
            for (V i : vertices(g)) {
                for (V j : vertices(g)) {
                    std::pair<V, V> edge(i, j);
                    pheromone_map[edge] *= (1 - a);
                }
            }
            float bonus = a / tour_distance(best_tour);
            for (int i = 0; i < best_tour.size() - 1; ++i) {
                std::pair<V, V> edge(best_tour[i], best_tour[i + 1]);
                pheromone_map[edge] += bonus;
            }

        }

    }

}

#endif
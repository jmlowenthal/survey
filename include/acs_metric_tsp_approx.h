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

template<typename VertexListGraph, typename OutputIterator>
void acs_metric_tsp_approx(
    const VertexListGraph& graph,
    OutputIterator output
) {
    BOOST_CONCEPT_ASSERT((boost::VertexListGraphConcept<VertexListGraph>));
    typedef typename boost::graph_traits<VertexListGraph>::vertex_descriptor V;

    EuclideanDistanceFunctor<VertexListGraph, ACS_RESOLUTION> weight_map(graph);
    std::map<std::pair<V, V>, float> pheromone_map;
    acs_metric_tsp_approx_from_vertex(
        graph,
        *vertices(graph).first,
        weight_map,
        boost::make_assoc_property_map(pheromone_map),
        boost::tsp_tour_visitor<OutputIterator>(output)
    );
}

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
    const bool closed_tour = true
) {
    using namespace boost;

    BOOST_CONCEPT_ASSERT((VertexListGraphConcept<VertexListGraph>));
    BOOST_CONCEPT_ASSERT((TSPVertexVisitorConcept<TSPVertexVisitor, VertexListGraph>));

    typedef typename graph_traits<VertexListGraph>::vertex_descriptor V;
    typedef typename graph_traits<VertexListGraph>::vertex_iterator VItr;

    const int num_points = num_vertices(g);

    random::mt11213b generator;

    auto tour_distance =
        closed_tour ? closed_tour_distance<WeightMap, ACS_RESOLUTION, V>
            : open_tour_distance<WeightMap, ACS_RESOLUTION, V>;

    std::vector<V> best_tour;

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
                VItr j, end;
                for (tie(j, end) = vertices(g); j != end; ++j) {
                    if (std::find(ant.begin(), ant.end(), *j) != ant.end()) {
                        continue;
                    }
                    std::pair<V, V> edge(current, *j);
                    float nu = 1.0f / weight_map[edge];
                    float prob = pheromone_map[edge] * powf(nu, beta);
                    probs[*j] = prob;
                    normalising += prob;
                    if (best == current || prob > probs[best]) {
                        best = *j;
                    }
                }

                // State transition rule
                float q = ((float)rand()) / __INT_MAX__;
                V next;
                if (q <= q0) {
                    BOOST_ASSERT(best != current);
                    next = best;
                }
                else {
                    float choice = ((float)rand()) / __INT_MAX__ * normalising;
                    VItr k;
                    for (tie(k, end) = vertices(g); k != end; ++k) {
                        if (choice <= probs[*k]) {
                            next = *k;
                            break;
                        }
                        choice -= probs[*k];
                    }
                }
                ant.push_back(next);

                // Local updating rule
                std::pair<V, V> edge(current, next);
                pheromone_map[edge] = (1 - p) * pheromone_map[edge] + p * tau_zero;

            }

            // Find the best tour
            best_tour = *min_element_by(
                ants.begin(),
                ants.end(),
                [&weight_map, &tour_distance](std::vector<V>& tour) {
                    return tour_distance(tour, weight_map);
                }
            );

            // Global updating rule
            VItr i, j, end_i, end_j;
            for (tie(i, end_i) = vertices(g); i != end_i; ++i) {
                for (tie(j, end_j) = vertices(g); j != end_j; ++j) {
                    std::pair<V, V> edge(*i, *j);
                    pheromone_map[edge] *= (1 - a);
                }
            }
            float bonus = a / tour_distance(best_tour, weight_map);
            for (int i = 0; i < best_tour.size() - 1; ++i) {
                std::pair<V, V> edge(best_tour[i], best_tour[i + 1]);
                pheromone_map[edge] += bonus;
            }
        }

    }

    for (V v : best_tour) {
        vis.visit_vertex(v, g);
    }

}

#endif
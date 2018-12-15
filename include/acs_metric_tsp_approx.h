#ifndef ACS_METRIC_TSP_APPROX_H
#define ACS_METRIC_TSP_APPROX_H

#ifndef ACS_RESOLUTION
#define ACS_RESOLUTION float
#endif

#define BOOST_PARAMETER_MAX_ARITY 20

#include <boost/graph/metric_tsp_approx.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/parameter.hpp>
#include <boost/type_traits.hpp>
#include <boost/mpl/front_inserter.hpp>
#include <algorithm>
#include <utility>
#include <cmath>
#include <map>

#include "EuclideanDistanceFunctor.h"
#include "tour_distance.h"
#include "min_element_by.h"
#include "nearest_neighbour_metric_tsp_approx.h"

/*
    float len = 0.0f;
    for (int i = 0; i < points.size() - 1; ++i) {
        len += Position2D::distance(points[i], points[i + 1]);
    }
    _tau_zero = 1.0f / (len * points.size
*/

BOOST_PARAMETER_NAME(graph)
BOOST_PARAMETER_NAME(start)
BOOST_PARAMETER_NAME(weight_map)
BOOST_PARAMETER_NAME(pheromone_map)
BOOST_PARAMETER_NAME(visitor)
BOOST_PARAMETER_NAME(output)
BOOST_PARAMETER_NAME(num_ants)
BOOST_PARAMETER_NAME(iterations)
BOOST_PARAMETER_NAME(beta)
BOOST_PARAMETER_NAME(q0)
BOOST_PARAMETER_NAME(p)
BOOST_PARAMETER_NAME(a)
BOOST_PARAMETER_NAME(tau_zero)
BOOST_PARAMETER_NAME(closed_tour)


BOOST_PARAMETER_FUNCTION(
    (void),
    acs_metric_tsp_approx,
    tag,
    (required
        (graph, *)
        (in_out(visitor), *)
    )
    (optional
        (start, *, *boost::vertices(graph).first)
        (weight_map, *, (EuclideanDistanceFunctor<graph_type, ACS_RESOLUTION>(graph)))
        (num_ants, (const int), 100)
        (iterations, (const int), 100)
        (beta, (const float), 2.0f)
        (q0, (const float), 0.9f)
        (p, (const float), 0.1f)
        (a, (const float), 0.1f)
        (tau_zero, (const float), 0.0f)
        (closed_tour, (const bool), true)
    )
) {
    std::map<std::pair<start_type, start_type>, float> pmap;
    acs_metric_tsp_approx_iterate(graph, pmap, visitor, start, weight_map, num_ants,
        iterations, beta, q0, p, a, tau_zero, closed_tour);
}

BOOST_PARAMETER_FUNCTION(
    (void),
    acs_metric_tsp_approx_iterate,
    tag,
    (required
        (graph, *)
        (in_out(pheromone_map), *)
        (in_out(visitor), *)
    )
    (optional
        (start, *, *boost::vertices(graph).first)
        (weight_map, *, (EuclideanDistanceFunctor<graph_type, ACS_RESOLUTION>(graph)))
        (num_ants, (const int), 100)
        (iterations, (const int), 100)
        (beta, (const float), 2.0f)
        (q0, (const float), 0.9f)
        (p, (const float), 0.1f)
        (a, (const float), 0.1f)
        (tau_zero, (const float), 0.0f)
        (closed_tour, (const bool), true)
    )
) {
    using namespace boost;

    BOOST_CONCEPT_ASSERT((VertexListGraphConcept<graph_type>));
    // BOOST_CONCEPT_ASSERT((TSPVertexVisitorConcept<visitor_type, graph_type>));

    typedef typename graph_traits<graph_type>::vertex_descriptor V;
    typedef typename graph_traits<graph_type>::vertex_iterator VItr;

    const int num_points = num_vertices(graph);

    random::mt11213b generator;

    auto tour_distance =
        closed_tour ? closed_tour_distance<weight_map_type, ACS_RESOLUTION, V>
            : open_tour_distance<weight_map_type, ACS_RESOLUTION, V>;

    std::vector<V> best_tour;

    for (int iteration = 0; iteration < iterations; ++iteration) {

        std::vector<std::vector<V>> ants(num_ants);
        for (int i = 0; i < num_ants; ++i) {
            ants[i].reserve(num_points);
            ants[i].push_back(random_vertex(graph, generator));
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
                for (tie(j, end) = vertices(graph); j != end; ++j) {
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
                    for (tie(k, end) = vertices(graph); k != end; ++k) {
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
            for (tie(i, end_i) = vertices(graph); i != end_i; ++i) {
                for (tie(j, end_j) = vertices(graph); j != end_j; ++j) {
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
        visitor.visit_vertex(v, graph);
    }

}

#endif
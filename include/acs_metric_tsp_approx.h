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

/**
 * Computes the Nearest-Neighbour heuristic for tau-zero, as described by Marco
 * Dorigo et al, 1997 [1].
 * [1]: http://people.idsia.ch/~luca/acs-ec97.pdf
 */
BOOST_PARAMETER_FUNCTION(
    (float),
    acs_nn_heuristic,
    tag,
    (required
        (graph, *)
    )
    (optional
        (start, *, *boost::vertices(graph).first)
        (weight_map, *, (EuclideanDistanceFunctor<graph_type, ACS_RESOLUTION>(graph)))
        (closed_tour, (const bool), true)
    )
) {
    typedef typename boost::graph_traits<graph_type>::vertex_descriptor V;
    std::vector<V> tour;
    nearest_neighbour_metric_tsp_approx(
        graph, weight_map, std::back_inserter(tour)
    );
    auto tour_distance =
        closed_tour ? closed_tour_distance<weight_map_type, ACS_RESOLUTION, V>
            : open_tour_distance<weight_map_type, ACS_RESOLUTION, V>;
    ACS_RESOLUTION len = tour_distance(tour, weight_map);
    return 1.0f / (tour.size() * len);
}

/**
 * Computes an approximate solution to the Traveling Salesperson Problem using
 * the Ant Colony System as described by Marco Dorigo et al, 1997 [1].
 * [1]: http://people.idsia.ch/~luca/acs-ec97.pdf
 */
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
        (closed_tour, (const bool), true)
        (tau_zero, (const float), acs_nn_heuristic(graph, start, weight_map, closed_tour))
    )
) {
    std::map<std::pair<start_type, start_type>, float> pmap;
    acs_metric_tsp_approx_iterate(graph, pmap, visitor, start, weight_map,
        num_ants, iterations, beta, q0, p, a, closed_tour, tau_zero);
}

/**
 * Finds the most probable next node to visit for the given ant, as described by
 * Marco Dorigo et al, 1997 [1]. This function will also update `probs` to be a
 * map from vertices to their probabilities.
 * [1]: http://people.idsia.ch/~luca/acs-ec97.pdf
 */
template<typename Graph, typename PMap, typename WMap>
inline typename boost::graph_traits<Graph>::vertex_descriptor acs_find_best(
    const Graph& graph,
    const WMap& weight_map,
    const std::vector<typename boost::graph_traits<Graph>::vertex_descriptor>& ant,
    typename boost::graph_traits<Graph>::vertex_descriptor current,
    PMap& pheromone_map,
    std::map<typename boost::graph_traits<Graph>::vertex_descriptor, float>& probs,
    float& normalising,
    const float beta
) {
    using namespace boost;

    typedef typename graph_traits<Graph>::vertex_descriptor V;
    typedef typename graph_traits<Graph>::vertex_iterator VItr;

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
    return best;
}

/**
 * Produces the next vertex for the given ant based on the current state, as
 * described by Marco Dorigo et al, 1997 [1].
 * [1]: http://people.idsia.ch/~luca/acs-ec97.pdf 
 */
template<typename Graph>
inline typename boost::graph_traits<Graph>::vertex_descriptor acs_state_transition(
    const Graph& graph,
    const typename boost::graph_traits<Graph>::vertex_descriptor current,
    const typename boost::graph_traits<Graph>::vertex_descriptor best,
    const float normalising,
    const float q0,
    std::map<typename boost::graph_traits<Graph>::vertex_descriptor, float>& probs
) {
    using namespace boost;

    typedef typename graph_traits<Graph>::vertex_iterator VItr;

    float q = ((float)rand()) / __INT_MAX__;
    if (q > q0) {
        float choice = ((float)rand()) / __INT_MAX__ * normalising;
        VItr k, end;
        for (tie(k, end) = vertices(graph); k != end; ++k) {
            if (choice <= probs[*k] && probs[*k] > 0) {
                return *k;
            }
            choice -= probs[*k];
        }
    }

    BOOST_ASSERT(best != current);
    return best;
}

/**
 * Updates the pheromone map based on the transition chosen by the given ant, as
 * described by Marco Dorigo et al, 1997 [1].
 * [1]: http://people.idsia.ch/~luca/acs-ec97.pdf
 */
template<typename V, typename PMap>
inline void acs_local_update(
    PMap& pheromone_map,
    const V& current,
    const V& next,
    const float p,
    const float tau_zero
) {
    std::pair<V, V> edge(current, next);
    pheromone_map[edge] = (1 - p) * pheromone_map[edge] + p * tau_zero;
}

/**
 * Updates the pheromone map based on the best produces tour in this iteration,
 * as described by Marco Dorigo et al, 1997 [1].
 * [1]: http://people.idsia.ch/~luca/acs-ec97.pdf
 */
template<typename Graph, typename PMap, typename WMap, typename Func>
inline void acs_global_update(
    const Graph& graph,
    PMap& pheromone_map,
    const std::vector<typename boost::graph_traits<Graph>::vertex_descriptor>& best_tour,
    const WMap& weight_map,
    const float a,
    Func tour_distance
) {
    using namespace boost;

    typedef typename graph_traits<Graph>::vertex_descriptor V;
    typedef typename graph_traits<Graph>::vertex_iterator VItr;

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

/**
 * Computes an approximate solution to the Traveling Salesperson Problem using
 * the Ant Colony System as described by Marco Dorigo et al, 1997 [1].
 * [1]: http://people.idsia.ch/~luca/acs-ec97.pdf
 */
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
        (closed_tour, (const bool), true)
        (tau_zero, (const float), acs_nn_heuristic(graph, start, weight_map, closed_tour))
        
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
            if (iteration + 1 == iterations) {
                ants[i].push_back(start);
            }
            else {
                ants[i].push_back(random_vertex(graph, generator));
            }
        }
        for (int step = 0; step < num_points - 1; ++step) {
            for (int i = 0; i < num_ants; ++i) {

                // Initialisation and auxilary computation
                std::vector<V>& ant = ants[i];
                std::map<V, float> probs;
                float normalising = 0.0f;
                V current = ant[ant.size() - 1];
                V best = acs_find_best(
                    graph,
                    weight_map,
                    ant,
                    current,
                    pheromone_map,
                    probs,
                    normalising,
                    beta
                );

                // State transition rule
                V next = acs_state_transition(
                    graph,
                    current,
                    best,
                    normalising,
                    q0,
                    probs
                );
                ant.push_back(next);

                // Local updating rule
                acs_local_update(
                    pheromone_map,
                    current,
                    next,
                    p,
                    tau_zero
                );

            }
        }

        // Find the best tour
        std::vector<V> tour = *min_element_by(
            ants.begin(),
            ants.end(),
            [&weight_map, &tour_distance](std::vector<V>& tour) {
                return tour_distance(tour, weight_map);
            }
        );

        if (best_tour.size() == 0 || tour_distance(tour, weight_map) < tour_distance(best_tour, weight_map)) {
            best_tour = tour;
        }

        // Global updating rule
        acs_global_update(
            graph,
            pheromone_map,
            best_tour,
            weight_map,
            a,
            tour_distance
        );

    }

    for (V v : best_tour) {
        visitor.visit_vertex(v, graph);
    }

}

#endif
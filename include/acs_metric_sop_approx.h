#ifndef ACS_METRIC_SOP_APPROX_H
#define ACS_METRIC_SOP_APPROX_H

#include "acs_metric_tsp_approx.h"

BOOST_PARAMETER_FUNCTION(
    (void),
    acs_metric_sop_approx_iterate,
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

    typedef graph_type G;
    typedef typename graph_traits<G>::vertex_descriptor Vertex;
    typedef typename graph_traits<G>::edge_iterator EdgeItr;

    const int num_points = num_vertices(graph);

    G visible;
    for (tie(itr, end) = vertices(_source); itr != end; ++itr) {
        if (in_degree(*itr, graph) == 0) {
            add_vertex(graph[*itr], visible);
        }
    }

    std::vector<Vertex> best_tour;
    std::map<std::pair<Vertex, Vertex>, float> pheromone_map;

    for (int iteration = 0; iteration < iterations; ++iteration) {
        
        std::vector<std::vector<Vertex>> ants(num_ants);
        for (int i = 0; i < num_ants; ++i) {
            ants[i].reserve(num_points);
            ants[i].push_back(random_vertex(graph, generator));
        }
        for (int step = 0; step < num_points; ++step) {
            for (std::vector<Vertex>& ant : ants) {

                // Initialisation and auxilary computation
                std::map<Vertex, float> probs;
                float normalising = 0.0f;
                Vertex current = ant[ant.size() - 1];
                Vertex best = acs_find_best(
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
        best_tour = *min_element_by(
            ants.begin(),
            ants.end(),
            [&weight_map, &tour_distance](std::vector<V>& tour) {
                return tour_distance(tour, weight_map);
            }
        );

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
}

#endif
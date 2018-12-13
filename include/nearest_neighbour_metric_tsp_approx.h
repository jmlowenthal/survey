#ifndef NEARESTNEIGHBOURTSPSOLVER_H
#define NEARESTNEIGHBOURTSPSOLVER_H

#ifndef NN_RESOLUTION
#define NN_RESOLUTION float
#endif

#include <boost/graph/metric_tsp_approx.hpp>
#include <boost/property_map/property_map.hpp>

#include "EuclideanDistanceFunctor.h"

template<typename VertexListGraph, typename OutputIterator>
void nearest_neighbour_metric_tsp_approx(
    const VertexListGraph& graph,
    OutputIterator output
) {
    nearest_neighbour_metric_tsp_approx_from_vertex(
        graph,
        *vertices(graph).first,
        EuclideanDistanceFunctor<VertexListGraph, NN_RESOLUTION>(graph),
        boost::tsp_tour_visitor<OutputIterator>(output)
    );
}

template<typename VertexListGraph, typename WeightMap, typename OutputIterator>
void nearest_neighbour_metric_tsp_approx(
    const VertexListGraph& graph,
    WeightMap weights,
    OutputIterator output
) {
    nearest_neighbour_metric_tsp_approx_from_vertex(
        graph,
        *vertices(graph).first,
        weights,
        boost::tsp_tour_visitor<OutputIterator>(output)
    );
}

template<typename VertexListGraph, typename OutputIterator>
void nearest_neighbour_metric_tsp_approx_from_vertex(
    const VertexListGraph& graph,
    typename boost::graph_traits<VertexListGraph>::vertex_descriptor start,
    OutputIterator output
) {
    nearest_neighbour_metric_tsp_approx_from_vertex(
        graph,
        start,
        EuclideanDistanceFunctor<VertexListGraph, NN_RESOLUTION>(graph),
        boost::tsp_tour_visitor<OutputIterator>(output)
    );
}

template<
    typename VertexListGraph,
    typename WeightMap,
    typename TSPVertexVisitor>
void nearest_neighbour_metric_tsp_approx_from_vertex(
    const VertexListGraph& g,
    typename boost::graph_traits<VertexListGraph>::vertex_descriptor start,
    WeightMap weight_map,
    TSPVertexVisitor vis
) {
    using namespace boost;

    BOOST_CONCEPT_ASSERT((VertexListGraphConcept<VertexListGraph>));
    BOOST_CONCEPT_ASSERT((TSPVertexVisitorConcept<TSPVertexVisitor, VertexListGraph>));

    typedef typename graph_traits<VertexListGraph>::vertex_descriptor Vertex;
    typedef typename graph_traits<VertexListGraph>::vertex_iterator VertexItr;

    // BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<WeightMap, std::pair<Vertex, Vertex>));

    VertexItr begin, end;
    tie(begin, end) = vertices(g);
    std::vector<Vertex> to_visit(begin, end);

    Vertex current = start;
    to_visit.erase(std::find(to_visit.begin(), to_visit.end(), current));
    vis.visit_vertex(current, g);

    while (!to_visit.empty()) {
        auto best = std::min_element(
            to_visit.begin(),
            to_visit.end(),
            [&current, &weight_map](Vertex a, Vertex b) {
                typedef typename boost::property_traits<WeightMap>::value_type Distance;
                Distance a_d = weight_map[{ current, a }];
                Distance b_d = weight_map[{ current, b }];
                return a_d < b_d;
            }
        );
        vis.visit_vertex(*best, g);
        current = *best;
        to_visit.erase(best);
    }

}

#endif
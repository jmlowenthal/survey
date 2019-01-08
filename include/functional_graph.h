#ifndef FUNCTIONAL_GRAPH_H
#define FUNCTIONAL_GRAPH_H

#include <utility>
#include <boost/graph/graph_traits.hpp>
#include <boost/shared_container_iterator.hpp>
#include <boost/concept_check.hpp>
#include <functional>

template<typename V, typename Container = std::vector<std::pair<V, V>>,
    typename ExpanderFunc = std::function<Container(V)>>
struct functional_graph {

    typedef typename V vertex_descriptor;
    typedef std::pair<typename V, typename V> edge_descriptor;
    typedef boost::directed_tag directed_category;
    typedef boost::disallow_parallel_edge_tag edge_parallel_category;
    typedef boost::bidirectional_graph_tag traversal_category;
    typedef boost::shared_container_iterator<Container> out_edge_iterator;
    typedef unsigned int degree_size_type;
    typedef out_edge_iterator in_edge_iterator;

    const ExpanderFunc pred;
    const ExpanderFunc succ;

    BOOST_CONCEPT_ASSERT((boost::UnaryFunctionConcept<ExpanderFunc, Container, V>));
    BOOST_CONCEPT_ASSERT((boost::ForwardContainerConcept<Container>));

    functional_graph(ExpanderFunc p, ExpanderFunc s) : pred(p), succ(s) {};

};

namespace boost {

template<typename V, typename EF, typename C>
std::pair<typename functional_graph<V, EF, C>::out_edge_iterator, 
    typename functional_graph<V, EF, C>::out_edge_iterator>
out_edges(
    typename functional_graph<V, EF, C>::vertex_descriptor v,
    functional_graph<V, EF, C> g
) {
    shared_ptr<C> p(new C(g.succ(v)));
    return make_shared_container_range(p);
}

template<typename V, typename EF, typename C>
typename functional_graph<V, EF, C>::vertex_descriptor source(
    typename functional_graph<V, EF, C>::edge_descriptor e,
    functional_graph<V, EF, C> g
) {
    return e.first;
}

template<typename V, typename EF, typename C>
typename functional_graph<V, EF, C>::vertex_descriptor target(
    typename functional_graph<V, EF, C>::edge_descriptor e,
    functional_graph<V, EF, C> g
) {
    return e.second;
}

template<typename V, typename EF, typename C>
typename functional_graph<V, EF, C>::vertex_descriptor out_degree(
    typename functional_graph<V, EF, C>::vertex_descriptor v,
    functional_graph<V, EF, C> g
) {
    typename functional_graph<V, EF, C>::out_edge_iterator itr, end;
    int count = 0;
    for (tie(itr, end) = out_edges(v, g); itr != end; ++itr) {
        ++count;
    }
    return count;
}

template<typename V, typename EF, typename C>
std::pair<typename functional_graph<V, EF, C>::out_edge_iterator, 
    typename functional_graph<V, EF, C>::out_edge_iterator>
in_edges(
    typename functional_graph<V, EF, C>::vertex_descriptor v,
    functional_graph<V, EF, C> g
) {
    shared_ptr<C> p(new C(g.pred(v)));
    return make_shared_container_range(p);
}

template<typename V, typename EF, typename C>
typename functional_graph<V, EF, C>::vertex_descriptor in_degree(
    typename functional_graph<V, EF, C>::vertex_descriptor v,
    functional_graph<V, EF, C> g
) {
    typename functional_graph<V, EF, C>::in_edge_iterator itr, end;
    int count = 0;
    for (tie(itr, end) = in_edges(v, g); itr != end; ++itr) {
        ++count;
    }
    return count;
}

template<typename V, typename EF, typename C>
typename functional_graph<V, EF, C>::vertex_descriptor degree(
    typename functional_graph<V, EF, C>::vertex_descriptor v,
    functional_graph<V, EF, C> g
) {
    return in_degree(v, g) + out_degree(v, g);
}

} // namespace boost

#endif
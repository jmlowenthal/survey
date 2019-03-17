#ifndef FUNCTIONAL_GRAPH_H
#define FUNCTIONAL_GRAPH_H

#include <utility>
#include <functional>

#include <boost/graph/graph_traits.hpp>
#include <boost/shared_container_iterator.hpp>
#include <boost/concept_check.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pending/property.hpp>
#include <boost/assert.hpp>

template<typename V>
struct edge {
    V source, target;
    inline bool operator==(const edge& other) const {
        return source == other.source && target == other.target;
    }
    inline bool operator!=(const edge& other) const {
        return !(*this == other);
    }
};

template<typename V, typename Container = std::vector<edge<V>>,
    typename ExpanderFunc = std::function<Container(V)>>
struct functional_graph {

    typedef V vertex_descriptor;
    typedef edge<V> edge_descriptor;
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

    functional_graph(ExpanderFunc p, ExpanderFunc s) : pred(p), succ(s) {
        #if !defined(BOOST_IS_VOID) && !defined(UNSAFE_FUNCTION_GRAPH)
            out_edge_iterator itr, end;
            V origin;
            for (tie(itr, end) = out_edges(origin, *this); itr != end; ++itr) {
                in_edge_iterator in_begin, in_end;
                V dst = target(*itr, *this);
                tie(in_begin, in_end) = in_edges(dst, *this);
                edge_descriptor edge = { origin, dst };
                BOOST_ASSERT_MSG(
                    std::find(in_begin, in_end, edge) != in_end,
                    "Predecessor function is not inverse of successor"
                );
            }
        #endif
    };

};

template<typename V, typename C, typename EF>
std::pair<typename functional_graph<V, C, EF>::out_edge_iterator, 
    typename functional_graph<V, C, EF>::out_edge_iterator>
out_edges(
    const typename functional_graph<V, C, EF>::vertex_descriptor& v,
    const functional_graph<V, C, EF>& g
) {
    boost::shared_ptr<C> p(new C(g.succ(v)));
    return make_shared_container_range(p);
}

template<typename V, typename C, typename EF>
typename functional_graph<V, C, EF>::vertex_descriptor source(
    const typename functional_graph<V, C, EF>::edge_descriptor& e,
    const functional_graph<V, C, EF>& g
) {
    return e.source;
}

template<typename V, typename C, typename EF>
typename functional_graph<V, C, EF>::vertex_descriptor target(
    const typename functional_graph<V, C, EF>::edge_descriptor& e,
    const functional_graph<V, C, EF>& g
) {
    return e.target;
}

template<typename V, typename C, typename EF>
typename functional_graph<V, C, EF>::degree_size_type out_degree(
    const typename functional_graph<V, C, EF>::vertex_descriptor& v,
    const functional_graph<V, C, EF>& g
) {
    typename functional_graph<V, C, EF>::out_edge_iterator itr, end;
    int count = 0;
    for (tie(itr, end) = out_edges(v, g); itr != end; ++itr) {
        ++count;
    }
    return count;
}

template<typename V, typename C, typename EF>
std::pair<typename functional_graph<V, C, EF>::in_edge_iterator, 
    typename functional_graph<V, C, EF>::in_edge_iterator>
in_edges(
    const typename functional_graph<V, C, EF>::vertex_descriptor& v,
    const functional_graph<V, C, EF>& g
) {
    auto l = g.pred(v);
    boost::shared_ptr<C> p(new C(l));
    return make_shared_container_range(p);
}

template<typename V, typename C, typename EF>
typename functional_graph<V, C, EF>::degree_size_type in_degree(
    const typename functional_graph<V, C, EF>::vertex_descriptor& v,
    const functional_graph<V, C, EF>& g
) {
    typename functional_graph<V, C, EF>::in_edge_iterator itr, end;
    int count = 0;
    for (tie(itr, end) = in_edges(v, g); itr != end; ++itr) {
        ++count;
    }
    return count;
}

template<typename V, typename C, typename EF>
typename functional_graph<V, C, EF>::degree_size_type degree(
    const typename functional_graph<V, C, EF>::vertex_descriptor& v,
    const functional_graph<V, C, EF>& g
) {
    return in_degree(v, g) + out_degree(v, g);
}

#endif
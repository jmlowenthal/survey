#ifndef ADA_STAR_SEARCH_H
#define ADA_STAR_SEARCH_H

#define BOOST_PARAMETER_MAX_ARITY 20

#include <boost/parameter.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/mpl/assert.hpp>

#include <map>
#include <set>
#include <queue>
#include <utility>
#include <algorithm>
#include <vector>

#include "EuclideanDistanceFunctor.h"
#include "min_element_by.h"

BOOST_PARAMETER_NAME(graph)
BOOST_PARAMETER_NAME(start)
BOOST_PARAMETER_NAME(goal)
BOOST_PARAMETER_NAME(updates)
BOOST_PARAMETER_NAME(g)
BOOST_PARAMETER_NAME(rhs)
BOOST_PARAMETER_NAME(open_set)
BOOST_PARAMETER_NAME(closed_set)
BOOST_PARAMETER_NAME(incons_set)
BOOST_PARAMETER_NAME(suboptimality)
BOOST_PARAMETER_NAME(weight_map)
BOOST_PARAMETER_NAME(heuristic)
BOOST_PARAMETER_NAME(visited)


#define V_TYPE(G) typename boost::graph_traits<G>::vertex_descriptor
#define E_TYPE(G) typename boost::graph_traits<G>::edge_descriptor

template <class G>
struct vertex_descriptor
{
    typedef V_TYPE(G) type;
};
#define V_PTYPE(G) vertex_descriptor<tag::G::_>

template <class G>
struct edge_descriptor
{
    typedef E_TYPE(G) type;
};
#define E_PTYPE(G) edge_descriptor<tag::G::_>

template<typename V>
inline boost::associative_property_map<std::map<V, float>> make_g(V start, V goal) {
    std::map<V, float> map;
    boost::associative_property_map<std::map<V, float>> s(map);
    s[start] = __FLT_MAX__;
    s[goal] = __FLT_MAX__;
    return s;
}

template<typename V>
inline boost::associative_property_map<std::map<V, float>> make_rhs(V start, V goal) {
    std::map<V, float> map;
    boost::associative_property_map<std::map<V, float>> s(map);
    s[start] = __FLT_MAX__;
    s[goal] = 0;
    return s;
}

template<typename V, typename GMap, typename RhsMap, typename Heuristic>
inline std::vector<std::pair<std::pair<float, float>, V>> make_open_set(
    GMap& g,
    RhsMap& rhs,
    Heuristic heuristic,
    const V goal,
    const V start,
    const float suboptimality
) {
    std::pair<std::pair<float, float>, V> pair = std::make_pair(
        ada_key(g, rhs, heuristic, goal, start, suboptimality),
        goal
    );
    std::vector<std::pair<std::pair<float, float>, V>> vec = { pair };
    return vec;
}

template<typename T>
inline boost::associative_property_map<std::map<T, bool>> make_property_map_set() {
    std::map<T, bool> m;
    return boost::associative_property_map<std::map<T, bool>>(m);
}

template<typename V, typename GMap, typename RhsMap, typename Heuristic>
inline std::pair<float, float> ada_key(
    GMap& g,
    RhsMap& rhs,
    Heuristic heuristic,
    const V s,
    const V start,
    const float suboptimality
) {
    std::pair<V, V> edge(start, s);
    if (g[s] < rhs[s]) {
        return { rhs[s] + suboptimality * heuristic[edge], rhs[s] };
    }
    else {
        return { g[s] + heuristic[edge], g[s] };
    }
}

template<typename G, typename GMap, typename RhsMap, typename WeightMap,
    typename OpenSet, typename Heuristic, typename VisitedMap>
inline void ada_update_state(
    const V_TYPE(G) s,
    const G& graph,
    const GMap& g,
    const RhsMap& rhs,
    WeightMap& weight_map,
    OpenSet& open_set,
    std::set<V_TYPE(G)>& closed_set,
    const V_TYPE(G) start,
    const V_TYPE(G) goal,
    Heuristic heuristic,
    const float suboptimality,
    VisitedMap& visited
) {
    using namespace boost;
    typedef typename graph_traits<G>::vertex_descriptor V;

    // If s was not visited before
    if (visited[s]) {
        visited[s] = true;
        g[s] = __FLT_MAX__;
    }

    // If s != goal, update rhs
    if (s != goal) {
        float bd = __FLT_MAX__;
        typedef typename graph_traits<G>::out_edge_iterator EItr;
        EItr i, end;
        for (tie(i, end) = out_edges(s, graph); i != end; ++i) {
            V s_prime = target(*i, graph);
            std::pair<V, V> edge(s, s_prime);
            float d = weight_map[edge] + g[s_prime];
            if (d < bd) {
                bd = d;
            }
        }
        rhs[s] = bd;
    }

    // Remove s from open_set
    for (typename std::vector<std::pair<std::pair<float, float>, V>>::iterator itr = open_set.begin(); itr != open_set.end(); ++itr) {
        if (itr->second == s) {
            open_set.erase(itr);
            std::make_heap(open_set.begin(), open_set.end());
            break;
        }
    }

    if (g[s] != rhs[s]) {
        if (closed_set.count(s) <= 0) {
            open_set.push_back({ ada_key(g, rhs, heuristic, s, start, suboptimality), s });
            std::push_heap(open_set.begin(), open_set.end());
        }
        else {
            closed_set.insert(s);
        }
    }
}

template<typename G, typename GMap, typename RhsMap, typename Heuristic,
    typename WeightMap, typename VisitedMap>
inline void ada_compute_or_improve_path(
    const G& graph,
    GMap& g,
    RhsMap& rhs,
    std::vector<std::pair<std::pair<float, float>, V_TYPE(G)>>& open_set,
    std::set<V_TYPE(G)>& closed_set,
    const V_TYPE(G) start,
    const V_TYPE(G) goal,
    Heuristic heuristic,
    const float suboptimality,
    const WeightMap& weight_map,
    VisitedMap& visited
) {
    using namespace boost;
    typedef V_TYPE(G) V;
    typedef typename graph_traits<G>::vertex_iterator VItr;

    while (
        open_set[0].first < ada_key(g, rhs, heuristic, start, start, suboptimality)
        || rhs[start] != g[start]
    ) {
        //TODO minheap
        // Pop s from the min-heap
        std::pair<std::pair<float, float>, V> s = open_set[0];
        std::pop_heap(open_set.begin(), open_set.end());
        open_set.pop_back();

        if (g[s.second] > rhs[s.second]) {
            g[s.second] = rhs[s.second];
            closed_set.insert(s.second);
        }
        else {
            g[s.second] = __FLT_MAX__;
            ada_update_state(
                s.second,
                graph,
                g,
                rhs,
                weight_map,
                open_set,
                closed_set,
                start,
                goal,
                heuristic,
                suboptimality,
                visited
            );
        }
        
        typedef typename graph_traits<G>::in_edge_iterator EItr;
        EItr i, end;
        for (tie(i, end) = in_edges(s.second, graph); i != end; ++i) {
            ada_update_state(
                source(*i, graph),
                graph,
                g,
                rhs,
                weight_map,
                open_set,
                closed_set,
                start,
                goal,
                heuristic,
                suboptimality,
                visited
            );
        }
    }
}

BOOST_PARAMETER_FUNCTION(
    (void),
    ada_star_search,
    tag,
    (required
        (graph, *)
        (start, (V_PTYPE(graph)))
        (goal,  (V_PTYPE(graph)))
    )
    (optional
        (heuristic, *, (EuclideanDistanceFunctor<graph_type, float>(graph)))
        (weight_map, *, (EuclideanDistanceFunctor<graph_type, float>(graph)))
        (updates, (std::vector<E_PTYPE(graph)>), std::vector<E_TYPE(graph_type)>())
        (suboptimality, (const float), 1.0f)
        (in_out(g), *, make_g<V_TYPE(graph_type)>(start, goal))
        (in_out(rhs), *, make_rhs<V_TYPE(graph_type)>(start, goal))
        (in_out(open_set), (std::vector<std::pair<std::pair<float, float>, V_PTYPE(graph)>>), make_open_set(g, rhs, heuristic, goal, start, suboptimality))
        (in_out(closed_set), (std::set<V_PTYPE(graph)>), std::set<V_TYPE(graph_type)>())
        (in_out(incons_set), (std::set<V_PTYPE(graph)>), std::set<V_TYPE(graph_type)>())
        (in_out(visited), *, make_property_map_set<V_TYPE(graph_type)>())
    )
) {

    using namespace boost;

    typedef V_TYPE(graph_type) V;
    typedef E_TYPE(graph_type) E;
    typedef typename graph_traits<graph_type>::vertex_iterator VItr;

    BOOST_CONCEPT_ASSERT((Sequence<updates_type>));
    // BOOST_CONCEPT_ASSERT((ReadPropertyMapConcept<heuristic_type, std::pair<V, V>>));
    // BOOST_CONCEPT_ASSERT((ReadPropertyMapConcept<weight_map_type, std::pair<V, V>>));
    BOOST_CONCEPT_ASSERT((ReadWritePropertyMapConcept<g_type, V>));
    BOOST_CONCEPT_ASSERT((ReadWritePropertyMapConcept<rhs_type, V>));

    // BOOST_MPL_ASSERT((boost::is_same<typename boost::remove_const<start_type>::type, V_TYPE(graph_type)>));
    // BOOST_MPL_ASSERT((boost::is_same<typename boost::remove_const<goal_type>::type, V_TYPE(graph_type)>));

    std::make_heap(open_set.begin(), open_set.end());

    for (E e : updates) {
        ada_update_state(
            source(e, graph),
            graph,
            g,
            rhs,
            weight_map,
            open_set,
            closed_set,
            start,
            goal,
            heuristic,
            suboptimality,
            visited
        );
    }

    // Move states from INCONS into OPEN
    for (V v : incons_set) {
        open_set.push_back({ ada_key(g, rhs, heuristic, v, start, suboptimality), v });
        std::push_heap(open_set.begin(), open_set.end());
    }
    
    closed_set.clear();

    ada_compute_or_improve_path(
        graph,
        g,
        rhs,
        open_set,
        closed_set,
        start,
        goal,
        heuristic,
        suboptimality,
        weight_map,
        visited
    );

}

template<typename G, typename WeightMap, typename GMap>
inline V_TYPE(G) ada_star_next_step(
    const G& graph,
    const V_TYPE(G) current,
    const WeightMap& weight_map,
    const GMap& g
) {
    using namespace boost;
    typedef typename graph_traits<G>::vertex_descriptor V;
    typedef typename graph_traits<G>::vertex_iterator VItr;

    // BOOST_CONCEPT_ASSERT();

    VItr begin, end;
    tie(begin, end) = vertices(graph);
    return *min_element_by(
        begin,
        end,
        [&g, &weight_map, &current](V v) {
            return weight_map[{ current, v }] + g[v];
        }
    );
}

template<typename G, typename GMap>
inline V_TYPE(G) ada_star_next_step(
    const G& graph,
    const V_TYPE(G) current,
    const GMap& g
) {
    ada_star_next_step(
        graph,
        current,
        EuclideanDistanceFunctor<G, float>(graph),
        g
    );
}

#endif
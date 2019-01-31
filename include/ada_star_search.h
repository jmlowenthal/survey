#ifndef ADA_STAR_SEARCH_H
#define ADA_STAR_SEARCH_H

#define BOOST_PARAMETER_MAX_ARITY 12

#include <boost/assert.hpp>
#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/heap_concepts.hpp>
#include <boost/heap/policies.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/parameter.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

#include "EuclideanDistanceFunctor.h"
#include "map_property_map.h"
#include "min_element_by.h"

namespace ada_star {

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

template<typename Key, typename Val>
struct key_value_pair {
    Key key;
    Val val;
};
template<typename Key, typename Val>
bool operator<(const key_value_pair<Key, Val>& a, const key_value_pair<Key, Val>& b) {
    return a.key < b.key;
}
template<typename Key, typename Val>
bool operator>(const key_value_pair<Key, Val>& a, const key_value_pair<Key, Val>& b) {
    return a.key > b.key;
}
#define K_TYPE(V) key_value_pair<std::pair<float, float>, V>

template<typename V>
inline map_property_map<V, float> make_g(V start, V goal) {
    map_property_map<V, float> s(INFINITY);
    put(s, start, INFINITY);
    put(s, goal, INFINITY);
    return s;
}

template<typename V>
inline map_property_map<V, float> make_rhs(V start, V goal) {
    map_property_map<V, float> s(INFINITY);
    put(s, start, INFINITY);
    put(s, goal, 0);
    return s;
}

template<typename V>
inline map_property_map<V, bool> make_visited(V start, V goal) {
    map_property_map<V, bool> m(false);
    put(m, start, true);
    put(m, goal, true);
    return m;
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
    if (get(g, s) > get(rhs, s)) {
        return {
            get(rhs, s) + suboptimality * get(heuristic, edge),
            get(rhs, s)
        };
    }
    else {
        return { get(g, s) + get(heuristic, edge), get(g, s) };
    }
}

template<typename V, typename GMap, typename RhsMap, typename Heuristic>
inline boost::shared_ptr<boost::heap::fibonacci_heap<K_TYPE(V), boost::heap::compare<std::greater<K_TYPE(V)>>>> make_open_set(
    GMap& g,
    RhsMap& rhs,
    Heuristic heuristic,
    const V goal,
    const V start,
    const float suboptimality
) {
    using namespace boost;
    using namespace boost::heap;
    typedef fibonacci_heap<K_TYPE(V), compare<std::greater<K_TYPE(V)>>> heap;
    shared_ptr<heap> q(new heap());
    q->push({
        ada_key(g, rhs, heuristic, goal, start, suboptimality),
        goal
    });
    return q;
}

template<typename G, typename GMap, typename RhsMap, typename WeightMap,
    typename OpenSet, typename Heuristic, typename VisitedMap>
inline void ada_update_state(
    const V_TYPE(G) s,
    const G& graph,
    const GMap& g,
    const RhsMap& rhs,
    WeightMap& weight_map,
    OpenSet open_set,
    std::unordered_set<V_TYPE(G)>& closed_set,
    std::unordered_set<V_TYPE(G)>& incons_set,
    const V_TYPE(G) start,
    const V_TYPE(G) goal,
    Heuristic heuristic,
    const float suboptimality,
    VisitedMap& visited
) {
    using namespace boost;
    typedef typename graph_traits<G>::vertex_descriptor V;

    // If s was not visited before
    if (!get(visited, s)) {
        put(visited, s, true);
        put(g, s, INFINITY);
    }

    // If s != goal, update rhs
    if (s != goal) {
        float bd = INFINITY;
        typedef typename graph_traits<G>::out_edge_iterator EItr;
        EItr i, end;
        for (tie(i, end) = out_edges(s, graph); i != end; ++i) {
            V s_prime = target(*i, graph);
            std::pair<V, V> edge(s, s_prime);
            float d = get(weight_map, edge) + get(g, s_prime);
            if (d < bd) {
                bd = d;
            }
        }
        put(rhs, s, bd);
    }

    // Remove s from open_set
    for (typename OpenSet::element_type::iterator itr = open_set->begin(); itr != open_set->end(); ++itr) {
        if (itr->val == s) {
            open_set->erase(OpenSet::element_type::s_handle_from_iterator(itr));
            break;
        }
    }

    if (get(g, s) != get(rhs, s)) {
        if (closed_set.count(s) <= 0) {
            open_set->push({
                ada_key(g, rhs, heuristic, s, start, suboptimality),
                s
            });
        }
        else {
            incons_set.insert(s);
        }
    }
}

template<typename G, typename GMap, typename RhsMap, typename OpenSet,
    typename Heuristic, typename WeightMap, typename VisitedMap>
inline void ada_compute_or_improve_path(
    const G& graph,
    GMap& g,
    RhsMap& rhs,
    OpenSet open_set,
    std::unordered_set<V_TYPE(G)>& closed_set,
    std::unordered_set<V_TYPE(G)>& incons_set,
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

    std::pair<float, float> key_start =
        ada_key(g, rhs, heuristic, start, start, suboptimality);

    while (
        !open_set->empty() && (
            open_set->top().key < key_start
            || get(rhs, start) != get(g, start)
        )
    ) {
        // Pop s from the min-heap
        V s = open_set->top().val;
        open_set->pop();

        bool update_s = false;
        if (get(g, s) > get(rhs, s)) {
            put(g, s, get(rhs, s));
            closed_set.insert(s);
        }
        else {
            put(g, s, INFINITY);
            update_s = true;
        }
        
        typedef typename graph_traits<G>::in_edge_iterator EItr;
        EItr i, end;
        for (tie(i, end) = in_edges(s, graph); i != end; ++i) {
            ada_update_state(
                source(*i, graph),
                graph,
                g,
                rhs,
                weight_map,
                open_set,
                closed_set,
                incons_set,
                start,
                goal,
                heuristic,
                suboptimality,
                visited
            );
        }
        if (update_s) {
            ada_update_state(
                s,
                graph,
                g,
                rhs,
                weight_map,
                open_set,
                closed_set,
                incons_set,
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
        (in_out(open_set), *, make_open_set(g, rhs, heuristic, goal, start, suboptimality))
        (in_out(visited), *, make_visited<V_TYPE(graph_type)>(start, goal))
    )
) {

    using namespace boost;
    using namespace boost::heap;

    BOOST_CONCEPT_ASSERT((BidirectionalGraphConcept<graph_type>));
    BOOST_CONCEPT_ASSERT((Sequence<updates_type>));
    BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<heuristic_type, std::pair<V_TYPE(graph_type), V_TYPE(graph_type)>>));
    BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<weight_map_type, std::pair<V_TYPE(graph_type), V_TYPE(graph_type)>>));
    BOOST_CONCEPT_ASSERT((Mutable_LvaluePropertyMapConcept<g_type, V_TYPE(graph_type)>));
    BOOST_CONCEPT_ASSERT((Mutable_LvaluePropertyMapConcept<rhs_type, V_TYPE(graph_type)>));
    BOOST_CONCEPT_ASSERT((Mutable_LvaluePropertyMapConcept<visited_type, V_TYPE(graph_type)>));
    BOOST_CONCEPT_ASSERT((MutablePriorityQueue<typename open_set_type::element_type>));

    typedef V_TYPE(graph_type) V;
    typedef E_TYPE(graph_type) E;
    typedef typename graph_traits<graph_type>::vertex_iterator VItr;

    std::unordered_set<V> incons_set;
    std::unordered_set<V> closed_set;
    
    // Update priorities
    std::vector<typename open_set_type::element_type::iterator> itrs;
    for (auto itr = open_set->begin(); itr != open_set->end(); ++itr) {
        itrs.push_back(itr);
    }
    for (typename open_set_type::element_type::iterator itr : itrs) {
        open_set->update(
            open_set_type::element_type::s_handle_from_iterator(itr),
            {
                ada_key(g, rhs, heuristic, itr->val, start, suboptimality),
                itr->val
            }
        );
    }

    ada_compute_or_improve_path(
        graph,
        g,
        rhs,
        open_set,
        closed_set,
        incons_set,
        start,
        goal,
        heuristic,
        suboptimality,
        weight_map,
        visited
    );

    // Apply updates
    for (E e : updates) {
        ada_update_state(
            source(e, graph),
            graph,
            g,
            rhs,
            weight_map,
            open_set,
            closed_set,
            incons_set,
            start,
            goal,
            heuristic,
            suboptimality,
            visited
        );
    }

    // Move states from INCONS into OPEN
    for (V v : incons_set) {
        open_set->push({
            ada_key(g, rhs, heuristic, v, start, suboptimality),
            v
        });
    }

}

template<typename G, typename WeightMap, typename GMap>
inline V_TYPE(G) ada_star_next_step(
    const G& graph,
    const V_TYPE(G) current,
    const WeightMap& weight_map,
    GMap& g
) {
    using namespace boost;
    typedef typename graph_traits<G>::vertex_descriptor V;
    typedef typename graph_traits<G>::edge_descriptor E;
    typedef typename graph_traits<G>::out_edge_iterator EItr;

    BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<G>));
    BOOST_CONCEPT_ASSERT((ReadablePropertyMapConcept<WeightMap, std::pair<V_TYPE(G), V_TYPE(G)>>));
    BOOST_CONCEPT_ASSERT((Mutable_LvaluePropertyMapConcept<GMap, V_TYPE(G)>));

    // Don't go back to this vertex unless we get new information (and update g)
    put(g, current, INFINITY);

    EItr begin, end;
    tie(begin, end) = out_edges(current, graph);
    EItr best = min_element_by(
        begin,
        end,
        [&graph, &g, &weight_map, &current](E e) {
            V v = target(e, graph);
            std::pair<V, V> pair = { current, v };
            float w = get(weight_map, pair);
            float gv = get(g, v);
            return w + gv;
        }
    );

    BOOST_ASSERT_MSG(best != end, "You have reached a dead-end.");

    return target(*best, graph);
}

template<typename G, typename GMap>
inline V_TYPE(G) ada_star_next_step(
    const G& graph,
    const V_TYPE(G) current,
    const GMap& g
) {
    return ada_star_next_step(
        graph,
        current,
        EuclideanDistanceFunctor<G, float>(graph),
        g
    );
}

}

#endif
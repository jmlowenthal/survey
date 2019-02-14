#include <catch.hpp>

#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/container_hash/hash.hpp>

#include <map>
#include <vector>
#include <exception>
#include <functional>

#define UNSAFE_FUNCTION_GRAPH

#include "functional_graph.h"
#include "EuclideanDistanceFunctor.h"
#include "map_property_map.h"

BOOST_CONCEPT_ASSERT((boost::BidirectionalGraphConcept<functional_graph<float>>));

namespace std {
template<typename A, typename B>
struct hash<std::pair<A, B>> {
    size_t operator()(std::pair<A, B> val) const {
        size_t seed = 0;
        boost::hash_combine(seed, val.first);
        boost::hash_combine(seed, val.second);
        return seed;
    }
};
}

TEST_CASE("Edge equality") {
    edge<int> a = {1, 2};
    edge<int> b = {1, 2};
    edge<int> c = {2, 1};
    REQUIRE(a == b);
    REQUIRE(b == a);
    REQUIRE(!(a == c));
    REQUIRE(!(c == a));
    REQUIRE(!(b == c));
    REQUIRE(!(c == b));
}

TEST_CASE("Edge inequality") {
    edge<int> a = {4, -1};
    edge<int> b = {4, -1};
    edge<int> c = {-11, -1};
    REQUIRE(a != c);
    REQUIRE(c != a);
    REQUIRE(b != c);
    REQUIRE(c != b);
    REQUIRE(!(a != b));
    REQUIRE(!(b != a));
}

using namespace boost;
typedef std::pair<float, float> V;
typedef functional_graph<V> G;
typedef edge<V> E;
typedef graph_traits<G>::out_edge_iterator OeItr;
typedef graph_traits<G>::in_edge_iterator IeItr;

std::vector<E> adj(V v) {
    return {
        {v, {v.first, v.second + 1}},
        {v, {v.first + 1, v.second}},
        {v, {v.first, v.second - 1}},
        {v, {v.first - 1, v.second}}
    };
};

std::vector<E> empty(V v) {
    return {};
}

std::vector<E> left(V v) {
    return { {v, {v.first - 1, v.second}} };
}

std::vector<E> right(V v) {
    return { {v, {v.first + 1, v.second}} };
}

TEST_CASE("Out edges") {
    G g(left, adj);
    V source = {0, 0};
    OeItr begin, end;
    tie(begin, end) = out_edges(source, g);
    edge<V> up = {source, {0, 1}};
    edge<V> down = {source, {0, -1}};
    edge<V> left = {source, {-1, 0}};
    edge<V> right = {source, {1, 0}};
    std::vector<edge<V>> edges(begin, end);
    REQUIRE(edges.size() == 4);
    REQUIRE(std::find(begin, end, up) != end);
    REQUIRE(std::find(begin, end, down) != end);
    REQUIRE(std::find(begin, end, left) != end);
    REQUIRE(std::find(begin, end, right) != end);
}

TEST_CASE("In edges") {
    G g(adj, left);
    V source = {0, 0};
    IeItr begin, end;
    tie(begin, end) = in_edges(source, g);
    edge<V> up = {source, {0, 1}};
    edge<V> down = {source, {0, -1}};
    edge<V> left = {source, {-1, 0}};
    edge<V> right = {source, {1, 0}};
    std::vector<edge<V>> edges(begin, end);
    REQUIRE(edges.size() == 4);
    REQUIRE(std::find(begin, end, up) != end);
    REQUIRE(std::find(begin, end, down) != end);
    REQUIRE(std::find(begin, end, left) != end);
    REQUIRE(std::find(begin, end, right) != end);
}

TEST_CASE("Source") {
    G g(adj, adj);
    V a = {0, 0};
    E edge = {a, {1, 1}};
    V v = source(edge, g);
    REQUIRE(v == a);
}

TEST_CASE("Target") {
    G g(adj, adj);
    V a = {0, 0};
    E edge = {{1, 1}, a};
    V v = target(edge, g);
    REQUIRE(v == a);
}

TEST_CASE("Out degree") {
    G g(adj, adj);
    V v = {0, 0};
    REQUIRE(out_degree(v, g) == 4);
}

TEST_CASE("In degree") {
    G g(adj, adj);
    V v = {0, 0};
    REQUIRE(in_degree(v, g) == 4);
}

TEST_CASE("Degree") {
    G g(adj, adj);
    V v = {0, 0};
    REQUIRE(degree(v, g) == 8);
}

struct goal_found {};

class goal_terminating_visitor : public default_astar_visitor {
private:
    const V _target;
public:
    goal_terminating_visitor(V v) : _target(v) {};
    void finish_vertex(V u, G g) {
        if (u == _target) {
            throw goal_found();
        }
    };  
};

float dist(V src, V dst) {
    float dx = src.first - dst.first;
    float dy = src.second - dst.second;
    return sqrt(dx * dx + dy * dy);
}

TEST_CASE("A* over functional graph") {
    G g(adj, adj);
    V start = {0, 0};
    V goal = {10, 10};

    std::function<float(E)> w_map = [&g](E e){
        V src = source(e, g);
        V dst = target(e, g);
        return dist(src, dst);
    };

    map_property_map<V, V> pred_map;
    map_property_map<V, float> dist_map(INFINITY);
    dist_map[start] = 0.0f;
    map_property_map<V, float> r_map;
    map_property_map<V, unsigned int> vi_map;
    map_property_map<V, default_color_type> c_map;

    goal_terminating_visitor vis(goal);

    REQUIRE_THROWS_AS(
        astar_search_no_init(
            g,
            start,
            [&goal](V v) {
                float d = dist(v, goal);
                return d;
            },
            visitor(vis).
            predecessor_map(pred_map).
            rank_map(r_map).
            distance_map(dist_map).
            weight_map(make_function_property_map<E>(w_map)).
            color_map(c_map).
            vertex_index_map(vi_map)
        ),
        goal_found
    );
}
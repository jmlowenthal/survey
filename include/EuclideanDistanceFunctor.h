#ifndef EUCLIDEAN_DISTANCE_FUNCTOR_H
#define EUCLIDEAN_DISTANCE_FUNCTOR_H

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/simple_point.hpp>

template<typename Graph, typename T>
class EuclideanDistanceFunctor {
private:
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
public:
    typedef std::pair<Vertex, Vertex> key_type;
    typedef const T reference;
    typedef T value_type;
    typedef boost::readable_property_map_tag category;
    EuclideanDistanceFunctor(const Graph&);
    const T operator[](std::pair<Vertex, Vertex>) const;
    const T operator[](Edge) const;
};

template<typename Graph>
class EuclideanDistanceFunctor<Graph, float> {

private:
    
    const Graph& _g;

    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;

public:

    typedef std::pair<Vertex, Vertex> key_type;
    typedef const float reference;
    typedef float value_type;
    typedef typename boost::readable_property_map_tag category;
    
    EuclideanDistanceFunctor(const Graph& g) : _g(g) {}

    const float operator[](std::pair<Vertex, Vertex> p) const {
        boost::simple_point<float> first = _g[p.first];
        boost::simple_point<float> second = _g[p.second];
        float dx = second.x - first.x;
        float dy = second.y - first.y;
        return sqrtf(dx * dx + dy * dy);
    }

    const float operator[](Edge e) const {
        boost::simple_point<float> first = _g[source(e, _g)];
        boost::simple_point<float> second = _g[target(e, _g)];
        float dx = second.x - first.x;
        float dy = second.y - first.y;
        return sqrtf(dx * dx + dy * dy);
    }
    
};


template<typename Graph>
class EuclideanDistanceFunctor<Graph, double> {

private:
    
    const Graph& _g;

    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;

public:

    typedef std::pair<Vertex, Vertex> key_type;
    typedef const double reference;
    typedef double value_type;
    typedef typename boost::readable_property_map_tag category;
    
    EuclideanDistanceFunctor(const Graph& g) : _g(g) {}

    const double operator[](key_type p) const {
        boost::simple_point<double> first = _g[p.first];
        boost::simple_point<double> second = _g[p.second];
        double dx = second.x - first.x;
        double dy = second.y - first.y;
        return sqrt(dx * dx + dy * dy);
    }

    const double operator[](Edge e) const {
        boost::simple_point<double> first = _g[source(e, _g)];
        boost::simple_point<double> second = _g[target(e, _g)];
        double dx = second.x - first.x;
        double dy = second.y - first.y;
        return sqrt(dx * dx + dy * dy);
    }
    
};

template<typename Graph, typename T>
inline typename EuclideanDistanceFunctor<Graph, T>::reference get(
    const EuclideanDistanceFunctor<Graph, T>& edf,
    typename EuclideanDistanceFunctor<Graph, T>::key_type p
) {
    return edf[p];
}

template<typename Graph, typename T>
inline typename EuclideanDistanceFunctor<Graph, T>::reference get(
    const EuclideanDistanceFunctor<Graph, T>& edf,
    typename boost::graph_traits<Graph>::edge_descriptor e
) {
    return edf[e];
}

#endif
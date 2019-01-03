#ifndef MAP_PROPERTY_MAP_H
#define MAP_PROPERTY_MAP_H

#include <boost/property_map/property_map.hpp>
#include <boost/concept/assert.hpp>
#include <map>

template<typename Key, typename Value>
class map_property_map
        : public boost::associative_property_map<std::map<Key, Value>> {
private:

    typedef boost::associative_property_map<std::map<Key, Value>> super;
    std::map<Key, Value> _map;

public:

    map_property_map() : super(_map) {};
    map_property_map(std::map<Key, Value>& map) : _map(map), super(_map) {};

    typedef Key key_type;
    typedef Value value_type;
    typedef Value& reference;
    typedef boost::lvalue_property_map_tag category;

};

#endif
#ifndef MAP_PROPERTY_MAP_H
#define MAP_PROPERTY_MAP_H

#include <boost/property_map/property_map.hpp>
#include <boost/concept/assert.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <map>
#include <memory>

template<typename Key, typename Value>
class map_property_map
        : public boost::put_get_helper<Value&, map_property_map<Key, Value>> {
private:

    boost::shared_ptr<std::map<Key, Value>> _map;

public:

    map_property_map() : _map(new std::map<Key, Value>) {};
    map_property_map(std::map<Key, Value>& map)
        : _map(new std::map<Key, Value>(map)) {};

    typedef Key key_type;
    typedef Value value_type;
    typedef Value& reference;
    typedef boost::lvalue_property_map_tag category;

    reference operator[](const key_type& k) const {
        return (*_map)[k];
    }

};

#endif
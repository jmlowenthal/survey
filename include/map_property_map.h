#ifndef MAP_PROPERTY_MAP_H
#define MAP_PROPERTY_MAP_H

#include <boost/concept/assert.hpp>
#include <boost/optional.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <unordered_map>
#include <memory>

template<typename Key, typename Value>
class map_property_map
        : public boost::put_get_helper<Value&, map_property_map<Key, Value>> {
private:

    const boost::shared_ptr<std::unordered_map<Key, Value>> _map;
    const boost::optional<Value> _default;

public:

    map_property_map(boost::optional<Value> def = boost::optional<Value>())
        : _map(new std::unordered_map<Key, Value>()), _default(def) {};
    map_property_map(
            std::map<Key, Value>& map, 
            boost::optional<Value> def = boost::optional<Value>())
        : _map(new std::unordered_map<Key, Value>(map)), _default(def) {};

    typedef Key key_type;
    typedef Value value_type;
    typedef Value& reference;
    typedef boost::lvalue_property_map_tag category;

    reference operator[](const key_type& k) const {
        bool replace = _map->count(k) == 0 && _default;
        reference r = (*_map)[k];
        if (replace) {
            r = _default.get();
        }
        return r;
    }

    typename std::unordered_map<Key, Value>::iterator begin() {
        return (*_map).begin();
    }

    typename std::unordered_map<Key, Value>::iterator end() {
        return (*_map).end();
    }

};

#endif
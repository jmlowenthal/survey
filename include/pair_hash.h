#ifndef PAIRHASH_H
#define PAIRHASH_H

#include <bits/functional_hash.h>

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

#endif
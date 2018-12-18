#ifndef TOUR_DISTANCE_H
#define TOUR_DISTANCE_H

template<typename WeightMap, typename Res, typename V>
inline Res tour_distance(const std::vector<V>& tour, WeightMap& weight_map) {
    Res total = 0;
    for (int i = 0; i < tour.size(); ++i) {
        total += weight_map[std::pair<V, V>(tour[i], tour[(i + 1) % tour.size()])];
    }
    return total;
};

#endif
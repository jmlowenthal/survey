#ifndef TOUR_DISTANCE_H
#define TOUR_DISTANCE_H

template<typename WeightMap, typename Res, typename V>
inline Res open_tour_distance(std::vector<V>& tour, WeightMap weight_map) {
    Res total = 0;
    for (int i = 0; i < tour.size() - 1; ++i) {
        total += weight_map[std::pair<V, V>(tour[i], tour[i + 1])];
    }
    return total;
};

template<typename WeightMap, typename Res, typename V>
inline Res closed_tour_distance(std::vector<V>& tour, WeightMap weight_map) {
    Res extra = weight_map[std::pair<V, V>(tour[tour.size() - 1], tour[0])];
    return open_tour_distance<WeightMap, Res, V>(tour, weight_map) + extra;
};

#endif
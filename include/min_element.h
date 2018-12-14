#ifndef MIN_ELEMENT_H
#define MIN_ELEMENT_H

template<typename Iter, typename Key>
Iter min_element(const Iter begin, const Iter end, const Key key) {
    Iter itr = begin;
    auto best = key(*itr);
    Iter best_itr = begin;
    ++itr;
    for (; itr != end; ++itr) {
        auto val = key(*itr);
        if (val < best) {
            best = val;
            best_itr = itr;
        }
    }
    return best_itr;
};

#endif
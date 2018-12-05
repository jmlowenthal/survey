#include "LinKernighanTspSolver.h"
#include <assert.h>
#include <limits>
#include "debug.h"
#include <debug/debug.h>

class Tour {

private:
    
    std::vector<Position2D> _points;
    
    float distance(const int i, const int j);

    int get_nearest_neighbour(const int index);

    void begin(const int t1, const int t2, const int t3);

    int get_next_ti(const std::vector<int>& ts);

    int get_next_y(const std::vector<int>& ts);

    std::vector<int> apply_changes(const std::vector<int>& ts, const int k);

    std::vector<Position2D> indices_to_tour(const std::vector<int>& tour);

    bool test_next(const std::vector<int>& ts, const int ti);

    bool test_y(const std::vector<int>& ts, const int ti, const int i);

    /**
     * Check whether the edge `x-y` is included in the proposed changes `ts`,
     * the disjunctive criteria.
     * @param ts Proposed changes.
     * @param x One end of the edge.
     * @param y The other end of the edge.
     * @return Returns true if the disjunctive criteria is met.
     */
    bool is_disjunctive(const std::vector<int>& ts, int x, int y);

    /**
     * Checks that the new y edge will make a positive difference to the gain.
     * @param ts Proposed changes.
     * @param i New vertex.
     * @return Returns true if the gain will be positive.
     */
    bool is_positive_gain(const std::vector<int>& ts, int i);

    /**
     * Checks that a new y edge will not get the algorithm stuck.
     * @param ts Proposed changes.
     * @param i New vertex.
     * @return Returns false if the vertex will cause LKH to get stuck.
     */
    bool is_deadend(const std::vector<int>& ts, int i);
    
public:

    Tour(const std::vector<Position2D>& points);

    std::vector<Position2D> get();

    void improve();

};

std::vector<Position2D> LinKernighanTspSolver::solve(
    const std::vector<Position2D>& points
) {
    Tour tour(points);
    tour.improve();
    return tour.get();
}

Tour::Tour(const std::vector<Position2D>& points) {
    _points = points;
}

std::vector<Position2D> Tour::get() {
    return _points;
}

float Tour::distance(int i, int j) {
    return Position2D::distance(_points[i], _points[j]);
}

void Tour::improve() {
    const int n = _points.size();
    for (int t1 = 0; t1 < n; ++t1) {
        int t2 = ((t1 + 1) % n);
        int t3 = get_nearest_neighbour(t2);
        float d2_3 = distance(t2, t3);
        float d1_2 = distance(t1, t2);
        if (t3 < 0 || d2_3 > d1_2) {
            t2 = ((t1 + n - 1) % n);
            t3 = get_nearest_neighbour(t2);
        }
        begin(t1, t2, t3);
    }
}

int Tour::get_nearest_neighbour(int t) {
    int bi = (t == 0 ? 1 : 0);
    float bd = distance(t, bi);
    for (int i = bi + 1; i < _points.size(); ++i) {
        if (i != t) {
            float d = distance(t, i);
            if (d < bd) {
                bi = i;
                bd = d;
            }
        }
    }
    return bi;
}

void Tour::begin(int t1, int t2, int t3) {
    std::vector<int> ts; // [...-(yn)-] t1 -(x1)- t2 -(y1)-...
    ts.push_back(t1);
    ts.push_back(t2);
    ts.push_back(t3);
    float initial_gain = distance(t1, t2) - distance(t2, t3);
    float g_star = 0;
    float gi = initial_gain;
    int k = 3;
    for (int i = 4;; i += 2) {

        int ti = get_next_ti(ts);
        if (ti < 0) {
            break;
        }
        ts.push_back(ti);
        int tip1 = get_next_y(ts);
        if (tip1 < 0) {
            break;
        }

        gi += distance(ts[ts.size() - 2], ti);
        float d = distance(ti, t1);
        if (gi - d > g_star) {
            g_star = gi - d;
            k = i;
        }

        ts.push_back(tip1);
        gi -= distance(ti, tip1);

    }
    if (g_star > 0) {
        ts[k] = ts[0];
        std::vector<int> tour = apply_changes(ts, k);
        _points = indices_to_tour(tour);
    }
}

std::vector<int> Tour::apply_changes(const std::vector<int>& ts, const int k) {
    int n = _points.size();
    int *edges = new int[3 * n]; // [..., count, nghbr1, nghbr2, ...] 
    for (int i = 0; i < n; ++i) {
        int *I = edges + i * 3;
        I[0] = 2; // count
        I[1] = (i + n - 1) % n; // neighbour A
        I[2] = (i + 1) % n; // neighbour B
    }

    // Remove Xs
    for (int i = 0; i < k; i += 2) {
        int a = ts[i];
        int b = ts[(i + 1) % k];
        int *A = edges + a * 3;
        A[0] = 1;
        A[1] = A[1] != b ? A[1] : A[2]; // pop
        int *B = edges + b * 3;
        B[0] = 1;
        B[1] = B[1] != a ? B[1] : B[2]; // pop
    }

    // Add Ys
    for (int i = 1; i < k; i += 2) {
        int a = ts[i];
        int b = ts[(i + 1) % k];
        edges[a * 3] = 2;
        edges[a * 3 + 2] = b; // push
        edges[b * 3] = 2;
        edges[b * 3 + 2] = a; // push
    }

    // Reconstruct tour from edges
    std::vector<int> tour;
    tour.reserve(_points.size());
    int current = 0;
    do {
        tour.push_back(current);
        int* C = edges + current * 3;
        assert(C[0] > 0);
        int next = C[C[0]];
        --C[0];
        int* N = edges + next * 3;
        N[0] = 1;
        N[1] = N[1] != current ? N[1] : N[2];
        current = next;
    }
    while (current != 0);

    return tour;

}

std::vector<Position2D> Tour::indices_to_tour(const std::vector<int>& tour) {
    std::vector<Position2D> result;
    result.reserve(tour.size());
    for (int i = 0; i < tour.size(); ++i) {
        result.push_back(_points[tour[i]]);
    }
    return result;
}

int Tour::get_next_ti(const std::vector<int>& ts) {
    int prev = ts[ts.size() - 1];
    int n = _points.size();
    int ti = (prev + n - 1) % n;
    if (test_next(ts, ti)) {
        return ti;
    }
    ti = (prev + 1) % n;
    if (test_next(ts, ti)) {
        return ti;
    }
    return -1;
}

bool Tour::test_next(const std::vector<int>& ts, const int ti) {
    std::vector<int> tour(ts);
    tour.push_back(ti);
    tour.push_back(tour[0]);
    std::vector<int> points = apply_changes(tour, tour.size());
    if (points.size() != _points.size()) {
        return false;
    }
    for (int i = 0; i < points.size(); ++i) {
        for (int j = i + 1; j < points.size(); ++j) {
            if (points[i] == points[j]) {
                return false;
            }
        }
    }
    return true;
}

int Tour::get_next_y(const std::vector<int>& ts) {
    int ti = ts[ts.size() - 1];
    float min_d = __FLT_MAX__;
    float min_i = -1;
    for (int i = 0; i < ts.size(); ++i) {
        if (test_y(ts, ti, i)) {
            float d = distance(ti, i);
            if (d < min_d) {
                min_d = d;
                min_i = i;
            }
        }
    }
    return min_i;
}

bool Tour::test_y(const std::vector<int>& ts, const int ti, const int i) {
    return is_disjunctive(ts, ti, i)
        && is_positive_gain(ts, i)
        && !is_deadend(ts, i);
}

bool Tour::is_disjunctive(const std::vector<int>& ts, const int x, const int y) {
    if (x == y) {
        return false;
    }
    int n = ts.size();
    for (int i = 0; i < n; ++i) {
        int a = ts[i];
        int b = ts[(i + 1) % n];
        if ((a == x && b == y) || (a == y && b == x)) {
            return false;
        }
    }
    return true;
}

bool Tour::is_positive_gain(const std::vector<int>& ts, const int i) {
    float gain = 0;
    for (int j = 1; j < ts.size() - 2; ++j) {
        int t1 = ts[j];
        int t2 = ts[j + 1];
        int t3 = (j == ts.size() - 3) ? i : ts[j + 2];
        gain += distance(t2, t3) - distance(t1, t2);
    }
    return gain > 0;
}

bool Tour::is_deadend(const std::vector<int>& ts, const int i) {
    int n = _points.size();
    return is_disjunctive(ts, i, (i + 1) % n)
        || is_disjunctive(ts, i, (i + n - 1) % n);
}
#include "AntColonySystem.h"
#include "SymmetricPairwiseMatrix.h"
#include <algorithm>
#include <cmath>
#include <cassert>

#define ANT_COUNT 16

class ACS {

private:

    SymmetricPairwiseMatrix<float> _pheromone;

    const std::vector<Position2D>& _points;

    std::vector<int> _best_tour;

    float distance(const int i, const int j) const;

    float tour_distance(const std::vector<int>& tour) const;

public:

    ACS(const std::vector<Position2D>& points);

    void iterate(int ant_count = 100, float beta = 2.0f, float q0 = 0.9, float p = 0.1, float a = 0.1);

    std::vector<Position2D> get() const;

};

std::vector<Position2D> AntColonySystem::solve(
    const std::vector<Position2D>& points
) {
    ACS acs(points);
    for (int i = 0; i < 100; ++i) {
        acs.iterate();
    }
    return acs.get();
}

ACS::ACS(const std::vector<Position2D>& points) : _points(points), _pheromone(points.size(), 0.0f) {
    
}

float ACS::distance(const int i, const int j) const {
    return Position2D::distance(_points[i], _points[j]);
}

float ACS::tour_distance(const std::vector<int>& tour) const {
    float d = 0.0f;
    for (int i = 0; i < tour.size() - 1; ++i) {
        d += distance(tour[i], tour[i + 1]);
    }
    return d;
}

void ACS::iterate(int ant_count, float beta, float q0, float p, float a) {
    std::vector<std::vector<int>> ants(ant_count);
    for (int j = 0; j < ant_count; ++j) {
        ants[j].reserve(_points.size());
        ants[j].push_back(rand() % _points.size());
    }
    for (int step = 0; step < _points.size(); ++step) {
        for (int i = 0; i < ant_count; ++i) {

            // Initialisation and auxilary computation
            std::vector<int>& ant = ants[i];
            std::vector<float> prob(_points.size());
            float normalising = 0.0f;
            int current = ant[ant.size() - 1];
            int best = -1;
            for (int j = 0; j < _points.size(); ++j) {
                if (std::find(ant.begin(), ant.end(), j) != ant.end()) {
                    continue;
                }
                float nu = 1.0f / distance(current, j);
                prob[j] = _pheromone.at(current, j) * powf(nu, beta);
                normalising += prob[j];
                if (best < 0 || prob[j] > prob[best]) {
                    best = j;
                }
            }

            // State transition rule
            float q = ((float)rand()) / __INT_MAX__;
            int j;
            if (q <= q0) {
                assert(best >= 0 && best < _points.size());
                j = best;
            }
            else {
                float choice = ((float)rand()) / __INT_MAX__ * normalising;
                j = 0;
                while (choice > prob[j]) {
                    choice -= prob[j];
                    ++j;
                }
                assert(j < _points.size());
            }
            ant.push_back(j);

            // Local updating rule
            _pheromone.at(current, j) = (1.0f - p) * _pheromone.at(current, j);// + p * initial_pheromone

        }
    }

    // Find the best tour
    float best_distance = 0.0f;
    for (std::vector<int>& ant : ants) {
        float d = tour_distance(ant);
        if (d < best_distance) {
            best_distance = d;
            _best_tour = ant;
        }
    }

    // Global updating rule
    for (int i = 0; i < _points.size(); ++i) {
        for (int j = i + 1; j < _points.size(); ++j) {
            _pheromone.at(i, j) = (1 - a) * _pheromone.at(i, j);
        }
    }
    float bonus = a / best_distance;
    for (int i = 0; i < _best_tour.size() - 1; ++i) {
        _pheromone.at(_best_tour[i], _best_tour[i + 1]) += bonus;
    }

}

std::vector<Position2D> ACS::get() const {
    std::vector<Position2D> res;
    res.reserve(_best_tour.size());
    for (int i = 0; i < _best_tour.size(); ++i) {
        res.push_back(_points[_best_tour[i]]);
    }
    return res;
}
#ifndef SYMMETRICPAIRWISEMATRIX_H
#define SYMMETRICPAIRWISEMATRIX_H

#include "PairwiseMatrix.h"
#include <vector>
#include <boost/assert.hpp>
#include <stdexcept>

template<typename T>
class SymmetricPairwiseMatrix : public PairwiseMatrix<T> {

private:

    std::vector<T> _data;

    const int _n;

public:

    SymmetricPairwiseMatrix(int n);

    SymmetricPairwiseMatrix(int n, T initial);


    virtual T& at(int i, int j) override;

};

template<typename T>
SymmetricPairwiseMatrix<T>::SymmetricPairwiseMatrix(int n) : _n(n), _data(n * (n - 1) / 2) {

}

template<typename T>
SymmetricPairwiseMatrix<T>::SymmetricPairwiseMatrix(int n, T initial) : _n(n), _data(n * (n - 1) / 2, initial) {

}


template<typename T>
T& SymmetricPairwiseMatrix<T>::at(int i, int j) {
    if (i >= _n || j >= _n) {
        throw std::invalid_argument("2D indices are out of range");
    }
    int a = std::min(i, j);
    int b = std::max(i, j);
    int index = a * _n + b;
    BOOST_ASSERT(index < _data.size());
    return _data[index];
}

#endif
#ifndef PAIRWISEMATRIX_H
#define PAIRWISEMATRIX_H

template<typename T>
class PairwiseMatrix {

public:

    virtual T& at(int i, int j) = 0;

};

#endif
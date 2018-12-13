#include "catch.hpp"
#include "SymmetricPairwiseMatrix.h"

TEST_CASE("[SymmetricPairwiseMatrix] Simple operation") {
    SymmetricPairwiseMatrix<float> mat(10);
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            REQUIRE_NOTHROW(mat.at(i, j));
            REQUIRE_NOTHROW(mat.at(j, i));
            REQUIRE(&mat.at(i, j) == &mat.at(j, i));
        }
    }
}
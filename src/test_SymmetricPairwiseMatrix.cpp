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

TEST_CASE("[SymmetricPairwiseMatrix] Out of bounds") {
    SymmetricPairwiseMatrix<float> mat(10);
    REQUIRE_THROWS(mat.at(10, 0));
    REQUIRE_THROWS(mat.at(0, 10));
    REQUIRE_THROWS(mat.at(-1, -1));
    REQUIRE_THROWS(mat.at(1, 10));
}
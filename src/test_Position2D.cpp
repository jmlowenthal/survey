#include "catch.hpp"
#include "Position2D.h"

TEST_CASE("Addition", "[!benchmark]") {

    Position2D a(1, 2);
    Position2D b(2, 3);
    Position2D c;
    BENCHMARK("Operator overloaded addition"){
        c = a + b;
    }
    REQUIRE(c.x == 3);
    REQUIRE(c.y == 5);

    float a_x = 1;
    float a_y = 2;
    float b_x = 2;
    float b_y = 3;
    float c_x;
    float c_y;
    BENCHMARK("Native addition") {
        c_x = a_x + b_x;
        c_y = a_y + b_y;
    }
    REQUIRE(c_x == 3);
    REQUIRE(c_y == 5);

}
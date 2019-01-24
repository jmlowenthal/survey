# Building

Run `cmake -DCMAKE_BUILD_TYPE=[Release|Debug]` to create Makefile, then run `make`.

# Unit tests

The `test` target (run `make test`) will execute the test but running the `tests` executable will give more verbose output. Run `tests [#filename] -#` to run the tests from a particular file, for example `tests [#test_Position2D] -#`.

# Profiling

```
valgrind --tool=callgrind <executable>
kcachegrind <callgrind output>
```
# Building

```
cmake
make
```

Optionally `cmake -DCMAKE_BUILD_TYPE=Debug` for debug builds.

# Profiling

```
valgrind --tool=callgrind <executable>
kcachegrind <callgrind output>
```
# C++ Code for Whole-Body Control with QP

### Authors

- Author/Maintainer: Konstantinos Chatzilygeroudis

### Features

- Generic tasks
- Generic constraints
- Generic QP solvers
    - qpOASES and EigenQLD supported
    - easy to add new
- Inverse Dynamics QP Solver
    - Support for floating base
- Inverse Kinemarics QP Solver
    - Operate in velocities (i.e., integrate to find positions)
- Using [DART](http://dartsim.github.io/) to get the dynamics equations


### Missing Features

- Automatic sparsification of problems for faster QP solving
- Benchmarking of QP solvers
- Unit tests

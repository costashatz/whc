# Whole-Body Control with QP (C++14)

### Authors

- Author/Maintainer: Konstantinos Chatzilygeroudis (costashatz@gmail.com)

### Features

- Generic tasks
- Generic constraints
- Generic QP solvers
    - [qpOASES](https://projects.coin-or.org/qpOASES) --- built along with `whc`
    - [OSQP](https://osqp.org/) --- optional dependency through [OsqpEigen](https://github.com/robotology/osqp-eigen) (highly recommended)
    - easy to add new
- Inverse Dynamics QP Solver
    - Support for floating base
- Inverse Kinemarics QP Solver
    - Operate in velocities (i.e., integrate to find positions)
- Using [DART](http://dartsim.github.io/) to get the dynamics equations


### Missing Features

- Benchmarking of QP solvers
- Unit tests

### Installing

#### Dependencies

**whc** depends on [DART](http://dartsim.github.io/) physics engine. We recommend installing **DART** from source:

```sh
cd /path/to/tmp/folder
git clone git://github.com/dartsim/dart.git
cd dart
git checkout v6.10.1

mkdir build
cd build
cmake -DDART_ENABLE_SIMD=ON -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr ..
make -j4
sudo make install
```

If you want to use the **OSQP** solver, please follow the installation instructions of [OsqpEigen](https://github.com/robotology/osqp-eigen/) and [OSQP](https://osqp.org/). If you want native flags (i.e., `march=native`) to be enabled (for performance), either do `export CXX_FLAGS="march=native -faligned-new"` before the cmake command for osqp installation or use our fork of [OsqpEigen](https://github.com/costashatz/osqp-eigen/tree/simd) (`simd` branch) and the `-DENABLE_SIMD=ON` cmake option.

For building and running the examples, [robot_dart](https://github.com/resibots/robot_dart) is required. Follow the [installation instructions](https://github.com/resibots/robot_dart/blob/master/docs/installation.md) provided by the developers.

#### Compilation

```sh
./waf configure --prefix=/path/to/installation (--prefix is optional, default is /usr/local)
./waf
./waf install (you might need sudo depending on the installation directory)
```

### Contributing

**whc** is being actively developed and the API is not stable. Please see [CONTRIBUTING](CONTRIBUTING.md) for more on how to help.

### Documentation

UNDER CONSTRUCTION

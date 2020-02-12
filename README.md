# Motion Planning library

## Installation

Install dependencies
```
sudo apt-get install libpcl-dev libeigen3-dev
```

Clone the repository, and create a build folder inside

```
git clone https://github.com/Bardo91/motion_planning
cd motion_planning
mkdir build && cd build
```

Configure compilation and compile the library

```
cmake .. && make -j4
```

Install the library

```
sudo make install
```

## Samples

The library has various example of the existing algorithms and the visualization tools.

1. RRSTAR example
2. Traveling Salesman Problem (TSP) example
3. 3D visualization example

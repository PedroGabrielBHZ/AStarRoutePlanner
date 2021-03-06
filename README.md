# Route Planning Project
This project implements a route planner that plots a path between two points on a map using real map data from the OpenStreeMap project.
The path is calculated with an implementation of the A* search algorithm.
The OpenStreetMap project is an open-source, collaborative endeavor to create free, user-generated maps of every part of the world. These maps are similar to the maps you might use in Google Maps or the Apple Maps app on your phone, but they are completely generated by individuals who volunteer to perform ground surveys of their local environment.

<img src="map.png" width="600" height="450" />

## Cloning

When cloning this project, be sure to use the `--recurse-submodules` flag. Using HTTPS:

```sh
git clone https://github.com/PedroGabrielBHZ/AStarRoutePlanner.git --recurse-submodules
```

or with SSH:

```sh
git https://github.com/PedroGabrielBHZ/AStarRoutePlanner.git --recurse-submodules
```

## Dependencies for Running Locally

* cmake >= 3.11.3
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 7.4.0
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same instructions as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* IO2D
  * Installation instructions for all operating systems can be found [here](https://github.com/cpp-io2d/P0267_RefImpl/blob/master/BUILDING.md)
  * This library must be built in a place where CMake `find_package` will be able to find it

## Compiling and Running

### Compiling

To compile the project, first, create a `build` directory and change to that directory:

```sh
mkdir build && cd build
```

From within the `build` directory, then run `cmake` and `make` as follows:

```sh
cmake ..
make
```

### Running

The executable will be placed in the `build` directory. From within `build`, you can run the project as follows:

```sh
./OSM_A_star_search
```

Or to specify a map file:

```sh
./OSM_A_star_search -f ../<your_osm_file.osm>
```

## Testing

The testing executable is also placed in the `build` directory. From within `build`, you can run the unit tests as follows:

```sh
./test
```

# Simulation - ARGoS3

This repository stores the code that defines the drones' behavior in the ARGoS3
simulator.

## Configure a container in a Wayland environment (e.g. argos3 is not installed)

If you want to launch ARGoS separately in a container without using the docker-compose do this :
```sh
# Create the container
./container.sh build

# Launch a specific arena
./container.sh {1,2,3,4,5}

# Launch a random arena
./container.sh 0

# Launch the with the last configuration
./container.sh
```

To install Weston use `$ apt-get install -y weston`.

## Launch argos simulation without container (We assume argos3 is installed)

5 arenas are predefined in the simulation. You can specify which one you want to use when launching it :
```sh
# Launch a specific arena
./launch.sh {1,2,3,4,5}

# Launch a random arena
./launch.sh 0

# Launch the with the last configuration
./launch.sh
```

## [Debug commands]
See https://gitlab.com/polytechnique-montr-al/inf3995/20211/equipe-203/crazyflie-project/-/blob/master/communication/protocols.md
```json
{"type":"startMission", "data":{"name": "simulation_1"}}
{"type":"startMission", "data":{"name": "simulation_2"}}
{"type":"returnToBase", "data":{"name": "simulation_1"}}
{"type":"returnToBase", "data":{"name": "simulation_2"}}
{"type":"land", "data":{"name": "simulation_1"}}
{"type":"land", "data":{"name": "simulation_2"}}
```

## Documentation
This project comes with an automatically generated documentation from the code's docstrings. An example is located in `doc/latex/refman.pdf`

To generate the project's documentation :

* Install Doxygen
  ```sh
  git clone https://github.com/doxygen/doxygen.git
  cd doxygen
  mkdir build && cd build
  cmake -G "Unix Makefiles" ..
  make
  make install
  ```
* Install the recommended extension in VSCode (Name: Doxygen Documentation Generator https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen)

* Once in a while, run Doxygen. Make sure you are in the root directory of the project (`/simulation`)
  ```sh
  doxygen doc/doxygen-config
  ```

  The output can be found in the `latex` folders located in `doc`.

  To generate a PDF :
  ```sh
  cd doc/latex
  make pdf
  ```

Example of a docstring :
```C++
/**
 * @brief Updates the status of the drone (battery, position, and speed)
 *
 * @param battery battery percentage
 * @param pos drone position in the simulation
 */
void RTStatus::update(std::float_t battery, const Vec4 &pos);
```

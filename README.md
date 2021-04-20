# Drone - ARGoS3

This repository stores the code that defines the drone behaviour in the ARGoS3
simulator.

## Launch argos simulation

5 arenas are predefined in the simulation. You can specify which one you want to use when launching it :
```bash
# Launch a specific arena
 $ ./launch.sh {1,2,3,4,5}

# Launch a random arena
 $ ./launch.sh 0

# Launch the with the last configuration
 $ ./launch.sh
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

## Documentation generation
To generate the project's documentation :

* Install Doxygen
  ```bash
  git clone https://github.com/doxygen/doxygen.git
  cd doxygen
  mkdir build && cd build
  cmake -G "Unix Makefiles" ..
  make
  make install
  ```
* Install the recommended extension in VSCode (Name: Doxygen Documentation Generator https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen)

* Once in a while, run Doxygen. Make sure you are in the root directory of the project (`/simulation`)
  ```bash
  doxygen doc/doxygen-config
  ```

  The output can be found in the `latex` folders located in `doc`.

  To generate a PDF :
  ```bash
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

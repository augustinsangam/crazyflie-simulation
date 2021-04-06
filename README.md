# Drone - ARGoS3

This repository stores the code that defines the drone behaviour in the ARGoS3
simulator.

## Launch argos simulation

```bash
# Build
sudo apt-get install rapidjson-dev
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make

# Launch
argos3 -c ../config.xml
```

## Docker
```bash
# Build
docker build -t argos3-sim .
# Run the simulator
x11docker -it --hostdisplay --user=RETAIN -- --network host -- --privileged argos3-sim argos3 -c /drone/config.xml
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

* Once in a while, run Doxygen. Make sure you are in the root directory of the project (`/drone`)
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

# Drone

Drone firmware

# Launch argos simulation

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

# Docker
```bash
# Build
docker build -t argos3-sim .
# Run the simulator
x11docker -it --hostdisplay --user=RETAIN -- --network host -- --privileged argos3-sim argos3 -c /drone/config.xml
```

# [Debug commands]
See https://gitlab.com/polytechnique-montr-al/inf3995/20211/equipe-203/crazyflie-project/-/blob/master/communication/protocols.md
```json
{"type":"takeOff", "data":{"name": "argos_drone_0"}}
{"type":"takeOff", "data":{"name": "argos_drone_1"}}
{"type":"land", "data":{"name": "argos_drone_0"}}
{"type":"land", "data":{"name": "argos_drone_1"}}
```

# Documentation generation
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
* Run Doxygen. Make sure you are in the root directory of the project (`/drone`)
  ```bash
  doxygen doc/doxygen-config
  ```

The output can be found in the `html` and `latex` folders located in `doc`.

To generate a PDF :
```bash
cd doc/latex
make pdf
```

# Drone

Drone software

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
x11docker -it --hostdisplay --user=RETAIN -- --privileged argos3-sim argos3 -c /drone/config.xml
```

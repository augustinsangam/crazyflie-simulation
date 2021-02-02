# Drone

Drone software

# Launch argos simulation

```bash
# Build
sudo apt-get install rapidjson-dev uuid-dev
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make

# Launch
cd ..
argos3 -c config.xml
```

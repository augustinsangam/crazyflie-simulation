#include "rapidjson/rapidjson.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <argos3/core/utility/datatypes/datatypes.h>
#include <bits/stdc++.h>
#include <cstring>
#include <ctime>
#include <iostream>
#include <map>
#include <math.h>
#include <ostream>
#include <string>
#include <unordered_map>

class RTStatus {
private:
	rapidjson::StringBuffer sb;
	rapidjson::Writer<rapidjson::StringBuffer> writer;
	/* Drone properties */
	std::string name;
	argos::Real batteryLevel;
	argos::Real posX;
	argos::Real posY;
	argos::Real posZ;
	int speed;
	bool isOn;
	// ...

public:
	RTStatus();
	RTStatus(const std::string &name);
	std::string encode();
	void update(argos::Real batteryLevel, argos::Real posX, argos::Real posY,
	            argos::Real posZ);
	void enable();
	void disable();
	void print() const;
};

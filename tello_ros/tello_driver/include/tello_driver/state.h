#pragma once

#include <string>
#include <map>

#define PITCH "pitch"
#define ROLL "roll"
#define YAW "yaw"
#define VELOCITY_X "vgx"
#define VELOCITY_Y "vgy"
#define VELOCITY_Z "vgz"
#define TEMP_LOW "templ"
#define TEMP_HIGH "temph"
#define TOF "tof"
#define HEIGHT "h"
#define BATTERY "bat"
#define BAROMETER "baro"
#define TIME "time"
#define ACCELERATION_X "agx"
#define ACCELERATION_Y "agy"
#define ACCELERATION_Z "agz"

void parseStateString(const std::string stateString, std::map<std::string, double>& state);

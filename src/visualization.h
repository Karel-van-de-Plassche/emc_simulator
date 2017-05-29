#ifndef PICO_SIMULATOR_VISUALIZATION_H_
#define PICO_SIMULATOR_VISUALIZATION_H_

#include "world.h"
#include <emc/io.h>

namespace visualization
{

void visualize(const World& world, Id robot_id, emc::LaserData scan);

}

#endif

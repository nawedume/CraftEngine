#pragma once

#include <vector>
#include "World.h"

namespace ce {

    extern void DetectContacts(World *world, std::vector<BodyPair> bodyIds);
};

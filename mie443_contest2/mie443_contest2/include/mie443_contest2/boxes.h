#pragma once

#include <vector>

class Boxes {
public:
    std::vector<std::vector<float>> coords;

    bool load_coords();
};

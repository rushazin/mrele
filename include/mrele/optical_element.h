#ifndef MRELE_OPTICAL_ELEMENT_H
#define MRELE_OPTICAL_ELEMENT_H

#include "ray_state.h"

class OpticalElement {
public:
    virtual void process(RayState& ray) = 0;
    virtual ~OpticalElement() = default;
};

#endif
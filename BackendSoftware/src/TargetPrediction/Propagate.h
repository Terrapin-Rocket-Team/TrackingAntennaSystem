/*
    Created by Jack yeulenski 3/28/26
    
    static method to propogate a state forward dt with a transition matrix.
*/

#ifndef PROPAGATE_H
#define PROPAGATE_H

#include "State.h"
#include <Math/Matrix.h>

class Propagate
{
public:
    static State propagate(const astra::Matrix& curState, double dt);
};

#endif // PROPAGATE_H
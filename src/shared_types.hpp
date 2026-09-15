#pragma once
struct Pos{
    double x,y,z;
};
struct Vel{
    double x,y,z;
};
struct TwistVel{
    float linear[3];
    float angular[3];
};
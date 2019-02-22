#include "Attractor.h"

Attractor::Attractor(float x, float y): m_x(x), m_y(y)
{
    //ctor
}

Attractor::~Attractor()
{
    //dtor
}

void Attractor::disp_params() const
{
    std::cout << "Attractor position: (" << m_x << "," << m_y << ")" << std::endl;
}

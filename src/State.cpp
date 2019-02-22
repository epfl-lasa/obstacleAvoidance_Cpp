#include "State.h"

State::State(float x, float y, float phi): m_x(x), m_y(y), m_phi(phi)
{
    //ctor
}

State::~State()
{
    //dtor
}

State::State(const State& other)
{
    //copy ctor
    m_x = other.m_x;
    m_y = other.m_y;
    m_phi = other.m_phi;
}

State& State::operator=(const State& rhs)
{
    if (this == &rhs) return *this; // handle self assignment
    //assignment operator
    m_x = rhs.m_x;
    m_y = rhs.m_y;
    m_phi = rhs.m_phi;
    return *this;
}

State operator+(State const& a, State const& b)
{
    State c(a);
    return c.sumStates(b);
}

State State::sumStates(const State& other)
{
    return State(m_x + other.m_x, m_y + other.m_y, m_phi + other.m_phi);
}

State operator-(State const& a, State const& b)
{
    State c(a);
    return c.diffStates(b);
}

State State::diffStates(const State& other)
{
    return State(m_x - other.m_x, m_y - other.m_y, m_phi - other.m_phi);
}


void State::disp_params() const
{
    std::cout << "State: (" << m_x << " , " << m_y << " , " << m_phi << ")" << std::endl;
}

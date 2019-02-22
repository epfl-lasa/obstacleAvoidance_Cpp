#ifndef OBSTACLE_H_INCLUDED
#define OBSTACLE_H_INCLUDED

#include <iostream>
#include <array>

class Obstacle
{
    public:

    Obstacle();
    Obstacle(std::array<float,2> a, std::array<float,2> p, std::array<float,2> x0, float th_r, float sf,
             std::array<float,2> xd, float xstart, float xend, float w);
    ~Obstacle();
    void disp_params() const;

    private:

    std::array<float,2> m_a;
    std::array<float,2> m_p;
    std::array<float,2> m_x0;
    float m_th_r;
    float m_sf;
    std::array<float,2> m_xd;
    float m_xstart;
    float m_xend;
    float m_w;
    };

#endif // OBSTACLE_H_INCLUDED

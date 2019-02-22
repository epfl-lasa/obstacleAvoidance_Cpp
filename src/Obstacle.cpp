#include "Obstacle.h"

using namespace std;

Obstacle::Obstacle()//: m_a(a), m_p(p), m_x0(x0), m_th_r(th_r), m_sf(sf), m_xd(xd), m_xstart(xstart), m_xend(xend), m_w(w)
{
    std::array<float,2> empty_array = {0, 0};
     m_a = empty_array;
     m_p = empty_array;
     m_x0 = empty_array;
     m_th_r = 0.0; m_sf = 0.0;
     m_xd = empty_array;
     m_xstart = 0.0; m_xend = 0.0; m_w = 0.0;
}

Obstacle::Obstacle(std::array<float,2> a, std::array<float,2> p, std::array<float,2> x0, float th_r, float sf,
                   std::array<float,2> xd, float xstart, float xend, float w):
                       m_a(a), m_p(p), m_x0(x0), m_th_r(th_r), m_sf(sf), m_xd(xd), m_xstart(xstart), m_xend(xend), m_w(w)
{ }

Obstacle::~Obstacle()
{
    // Destructor
}

void Obstacle::disp_params() const
{
    cout << "Obstacle parameters: " << endl;
    cout << "m_a: " << m_a[0] << "," << m_a[1] << endl;
    cout << "m_p: " << m_p[0] << "," << m_p[1] << endl;
    cout << "m_x0: " << m_x0[0] << "," << m_x0[1] << endl;
    cout << "m_th_r: " << m_th_r << endl;
    cout << "m_sf: " << m_sf << endl;
    cout << "m_xd: " << m_xd[0] << "," << m_xd[1] << endl;
    cout << "m_xstart: " << m_xstart << endl;
    cout << "m_xend: " << m_xend << endl;
    cout << "m_w: " << m_w << endl;
}

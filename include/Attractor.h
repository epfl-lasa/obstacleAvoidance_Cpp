#ifndef ATTRACTOR_H
#define ATTRACTOR_H

#include <iostream>

class Attractor
{
    public:
        Attractor(float x, float y);
        virtual ~Attractor();

        float Getx() { return m_x; }
        void Setx(float val) { m_x = val; }
        float Gety() { return m_y; }
        void Sety(float val) { m_y = val; }
        void disp_params() const;

    protected:

    private:
        float m_x;
        float m_y;
};

#endif // ATTRACTOR_H

#ifndef STATE_H
#define STATE_H

#include <iostream>

class State
{
    public:
        State(float x, float y, float phi);
        virtual ~State();
        State(const State& other);
        State& operator=(const State& other);

        State sumStates(const State& other);  // To perform the sum of two states
        State diffStates(const State& other); // To perform the difference of two states

        float Getx() { return m_x; }
        void Setx(float val) { m_x = val; }
        float Gety() { return m_y; }
        void Sety(float val) { m_y = val; }
        float Getphi() { return m_phi; }
        void Setphi(float val) { m_phi = val; }

        void disp_params() const;

    protected:

    private:
        float m_x;
        float m_y;
        float m_phi;
};

State operator+(State const& a, State const& b);
State operator-(State const& a, State const& b);

#endif // STATE_H

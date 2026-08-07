#pragma once

enum TrajType {
    LINEAR,
    MINJERK,
    BEZIER3,
    SPLINE
};

class Trajectory {
public:
    Trajectory() {}

    void set(TrajType type, float p0, float p1, float duration,
             float c1 = 0.0, float c2 = 0.0) {
        trajType = type;
        this->p0 = p0;
        this->p1 = p1;
        this->duration = duration;

        // Optional control points for Bézier/Spline
        cp1 = c1;
        cp2 = c2;
    }

    // t in Sekunden
    float evaluate(float t) {
        if (t <= 0) return p0;
        if (t >= duration) return p1;

        float u = t / duration;

        switch (trajType) {
        case LINEAR:
            return linear(u);
        case MINJERK:
            return minjerk(u);
        case BEZIER3:
            return bezier3(u);
        case SPLINE:
            return spline(u);
        }
        return p1;
    }

private:
    TrajType trajType;

    float p0 = 0, p1 = 0;
    float duration = 1.0;

    // Control points (Bézier, Spline)
    float cp1 = 0, cp2 = 0;

    // ---- Trajectories ----

    float linear(float u) {
        return p0 + (p1 - p0) * u;
    }

    float minjerk(float u) {
        // Smoothstart/stop: 10u³ - 15u⁴ + 6u⁵
        float u2 = u * u;
        float u3 = u2 * u;
        float u4 = u3 * u;
        float u5 = u4 * u;
        float s = 10*u3 - 15*u4 + 6*u5;
        return p0 + (p1 - p0) * s;
    }

    float bezier3(float u) {
        // Cubic Bézier: B(u) = P0*(1-u)^3 + C1*3u(1-u)^2 + C2*3u²(1-u) + P1*u³
        float u1 = (1 - u);
        return  p0 * (u1*u1*u1)
              + cp1 * (3 * u * u1 * u1)
              + cp2 * (3 * u*u * u1)
              + p1 * (u*u*u);
    }

    float spline(float u) {
        // Cubic Hermite spline with tangents cp1, cp2
        float u2 = u*u;
        float u3 = u2*u;

        float h00 = 2*u3 - 3*u2 + 1;
        float h10 = u3 - 2*u2 + u;
        float h01 = -2*u3 + 3*u2;
        float h11 = u3 - u2;

        return h00 * p0 + h10 * cp1 + h01 * p1 + h11 * cp2;
    }
};

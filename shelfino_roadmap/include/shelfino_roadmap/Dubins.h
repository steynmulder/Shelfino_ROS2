#ifndef Dubins
#define Dubins

#include <cmath>

// Utility functions mod2pi represent the angles [0, 2pi)
static double mod2pi(double angle) {
    while (angle < 0) angle += M_2_PI;
    while (angle >= M_2_PI) angle -= M_2_PI;
    return angle;
}

static double sinc(double t) {
    if (fabs(t) < 0.002) {
        return 1 - (t * t) / 6.0;
    }
    return sin(t) / t;
}



// Structures to represent arcs and curves
struct DubinsArc {
    double x0, y0, th0;  // Start position and angle
    double k;            // Curvature
    double L;            // Arc length

    double xf, yf, thf;  // End position and angle

    DubinsArc(double x0, double y0, double th0, double k, double L)
        : x0(x0), y0(y0), th0(th0), k(k), L(L) {
        double s = k * L / 2.0;
        xf = x0 + L * sinc(s) * cos(th0 + s);
        yf = y0 + L * sinc(s) * sin(th0 + s);
        thf = mod2pi(th0 + k * L);
    }
};

struct DubinsCurve {
    DubinsArc a1, a2, a3;
    double L;  // Total length

    DubinsCurve(DubinsArc a1, DubinsArc a2, DubinsArc a3)
        : a1(a1), a2(a2), a3(a3), L(a1.L + a2.L + a3.L) {}
};

DubinsCurve dubinsShortestPath(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);

#endif  // Dubins

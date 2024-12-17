#include "shelfino_followpath/Dubins.h"
#include <iostream>
#include <limits>
#include <vector>
#include <cmath>

// Dubins path calculations
bool LSL(double th0, double thf, double d, double& s1, double& s2, double& s3) {
    double C = cos(thf) - cos(th0);
    double S = 2 * d + sin(th0) - sin(thf);
    double temp1 = atan2(C, S);
    s1 = mod2pi(temp1 - th0);
    double temp2 = 2 + 4 * d * d - 2 * cos(th0 - thf) + 4 * d * (sin(th0) - sin(thf));
    if (temp2 < 0) return false;
    s2 = sqrt(temp2);
    s3 = mod2pi(thf - temp1);
    return true;
}

bool RSR(double th0, double thf, double d, double& s1, double& s2, double& s3) {
    double C = cos(th0) - cos(thf);
    double S = 2 * d - sin(th0) + sin(thf);
    double temp1 = atan2(C, S);
    s1 = mod2pi(th0 - temp1);
    double temp2 = 2 + 4 * d * d - 2 * cos(th0 - thf) - 4 * d * (sin(th0) - sin(thf));
    if (temp2 < 0) return false;
    s2 = sqrt(temp2);
    s3 = mod2pi(temp1 - thf);
    return true;
}

bool LSR(double th0, double thf, double d, double& s1, double& s2, double& s3) {
    double C = cos(th0) + cos(thf);
    double S = 2 * d + sin(th0) + sin(thf);
    double temp1 = atan2(-C, S);
    double temp3 = 4 * d * d - 2 + 2 * cos(th0 - thf) + 4 * d * (sin(th0) + sin(thf));
    if (temp3 < 0) return false;
    s2 = sqrt(temp3);
    s1 = mod2pi(temp1 - th0);
    s3 = mod2pi(thf - temp1 - s2);
    return true;
}

bool RSL(double th0, double thf, double d, double& s1, double& s2, double& s3) {
    double C = cos(th0) + cos(thf);
    double S = 2 * d - sin(th0) - sin(thf);
    double temp1 = atan2(C, S);
    double temp3 = 4 * d * d - 2 + 2 * cos(th0 - thf) - 4 * d * (sin(th0) + sin(thf));
    if (temp3 < 0) return false;
    s2 = sqrt(temp3);
    s1 = mod2pi(th0 - temp1 - s2);
    s3 = mod2pi(temp1 - thf);
    return true;
}

bool RLR(double th0, double thf, double d, double& s1, double& s2, double& s3) {
    double temp = (6 - 4 * d * d + 2 * cos(th0 - thf) + 4 * d * (sin(th0) - sin(thf))) / 8.0;
    if (fabs(temp) > 1) return false;
    s2 = mod2pi(M_2_PI - acos(temp));
    s1 = mod2pi(th0 - atan2(cos(thf) - cos(th0), 2 * d + sin(th0) - sin(thf)));
    s3 = mod2pi(th0 - thf + s2);
    return true;
}

bool LRL(double th0, double thf, double d, double& s1, double& s2, double& s3) {
    double temp = (6 - 4 * d * d + 2 * cos(th0 - thf) - 4 * d * (sin(th0) - sin(thf))) / 8.0;
    if (fabs(temp) > 1) return false;
    s2 = mod2pi(M_2_PI - acos(temp));
    s1 = mod2pi(-atan2(cos(th0) - cos(thf), 2 * d + sin(thf) - sin(th0)));
    s3 = mod2pi(thf - th0 + s2);
    return true;
}

DubinsCurve dubinsShortestPath(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax) {
    double dx = xf - x0, dy = yf - y0;
    double d = sqrt(dx * dx + dy * dy) * Kmax;
    double alpha = mod2pi(atan2(dy, dx) - th0);
    double beta = mod2pi(thf - atan2(dy, dx));

    double bestLength = std::numeric_limits<double>::infinity();
    DubinsCurve bestCurve(DubinsArc(0, 0, 0, 0, 0), DubinsArc(0, 0, 0, 0, 0), DubinsArc(0, 0, 0, 0, 0));

    double s1, s2, s3;
    for (auto& func : {LSL, RSR, LSR, RSL, RLR, LRL}) {
        if (func(alpha, beta, d, s1, s2, s3)) {
            DubinsArc arc1(x0, y0, th0, 1 / Kmax, s1);
            DubinsArc arc2(arc1.xf, arc1.yf, arc1.thf, 0, s2);
            DubinsArc arc3(arc2.xf, arc2.yf, arc2.thf, 1 / Kmax, s3);
            DubinsCurve curve(arc1, arc2, arc3);
            if (curve.L < bestLength) {
                bestLength = curve.L;
                bestCurve = curve;
            }
        }
    }
    return bestCurve;
}

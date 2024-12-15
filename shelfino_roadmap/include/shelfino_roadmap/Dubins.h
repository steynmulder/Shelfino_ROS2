#ifndef Dubins
#define Dubins

struct DubinsCurve;

DubinsCurve dubinsShortestPath(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);

#endif  // Dubins

#ifndef UTIL_H
#define UTIL_H

class Util
{
    public:
    Util(){}

    static double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew)
    {
      double dx = x - x0, dy = y - y0;
      double h = sqrt(dx * dx + dy * dy);
      double angle = atan2(dy, dx);
      double mx = cos(angle - skew) * h;
      double my = sin(angle - skew) * h;
      double f1 = pow(mx, 2.0) / (2.0 * varx),
             f2 = pow(my, 2.0) / (2.0 * vary);
      return A * exp(-(f1 + f2));
    }

    static double get_radius(double cutoff, double A, double var)
    {
      return sqrt(-2 * var * log(cutoff / A));
    }

};


#endif  // UTIL_H

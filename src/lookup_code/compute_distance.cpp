#include <iostream>
#include <fstream>
#include <cstdio>
#include <map>
#include <vector>
#include <math.h>

using namespace std;

struct Trajectory {
  double dtheta;
  double dx;
  double dy;
  Trajectory ()
  {
    dtheta = 0.0;
    dx = 0.0;
    dy = 0.0;
  }
};

const double w = 0.05236;
const double x = 1.0;

/* incr: step size, must be less than tf
 * tm: time in between at which buggy stops wheel rotation (sec)
 *     should be less than tf/2
 * tf: total time (sec)
 * displacement d(t) = at^2+bt+c
 */


Trajectory find_rotation(double incr, double tm, double tf, double a, double b) {
  double ang = 0.0;
  double t = 0.0;
  double mid = 0.0;
  Trajectory tr;
  for (;t<=tm;t+=incr) {
    ang = t * w;
    double d = incr * ((2 * a * t) + b);
    mid = d * sin ((ang / 2.0) + ((incr * w) / 4.0)) / x;
    tr.dx += d * sin (tr.dtheta + mid);
    tr.dy += d * cos (tr.dtheta + mid);
    tr.dtheta += 2 * mid;
  }
  for (;t<=tf-tm;t+=incr) {
    double d = incr * ((2 * a * t) + b);
    mid = d * sin ((ang / 2.0) + ((incr * w) / 4.0)) / x;
    tr.dx += d * sin (tr.dtheta + mid);
    tr.dy += d * cos (tr.dtheta + mid);
    tr.dtheta += 2 * mid;
  }
  for (;t<=tf;t+=incr) {
    ang = ((tf - t) * w);
    double d = incr * ((2 * a * t) + b);
    mid = d * sin ((ang / 2.0) - ((incr * w) / 4.0)) / x;
    tr.dx += d * sin (tr.dtheta + mid);
    tr.dy += d * cos (tr.dtheta + mid);
    tr.dtheta += 2 * mid;
  }
  return tr;
}

int main() {
  double ttl, dx, dy;
  for (double i = 0.001 ; i < 0.5 ; i += 0.01)
  {
    Trajectory tr2 = find_rotation(0.01, i, 1, 0, 60);
    printf("%f %f %f\n",tr2.dtheta,tr2.dx,tr2.dy);
  }
}
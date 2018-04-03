#include <iostream>
#include <fstream>
#include <cstdio>
#include <map>
#include <math.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <boost/functional/hash.hpp> //need Boost library

using namespace std;
#define M_PI 3.14159265358979323846

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

int numDigits(int x)
{
    if (x >= 10000) {
        if (x >= 10000000) {
            if (x >= 100000000) {
                if (x >= 1000000000)
                    return 10;
                return 9;
            }
            return 8;
        }
        if (x >= 100000) {
            if (x >= 1000000)
                return 7;
            return 6;
        }
        return 5;
    }
    if (x >= 100) {
        if (x >= 1000)
            return 4;
        return 3;
    }
    if (x >= 10)
        return 2;
    if (x == 0) 
      return 0;
    return 1;
}

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

template <typename T>

struct vectorHasher{
std::size_t operator()(const std::vector<T> &in) const
{
    using boost::hash_value;
    using boost::hash_combine;
    // Start with a hash value of 0
    std::size_t seed = 0;
    T value;
    for (int i=0; i< in.size(); i++)
    {
        value = static_cast<T>(in[i]);
        hash_combine(seed, hash_value(value));
    }
    return seed;
}
};


unordered_map<vector<int>, int, vectorHasher< int > > hashTbl;

ofstream dataFile;
const int vector_size = 6;
const double sig_fig = 50.0;
vector<int> key;

void init_store(char file[]) {
  for (int i = 0; i < vector_size; i++) {
    key.push_back(0);
  }
  dataFile.open(file, ios::out | ios::binary);
}

void store(vector<int> key) {
  for ( auto i = key.begin(); i != key.end(); i++ ) {
    dataFile << *i << " ";
    //dataFile.write((char*)(&(*i)), sizeof(int));
  }
  dataFile << endl;
}

void end_store() {
  dataFile.flush();
  dataFile.close();
}

int to_degrees(double theta) {
  return (((int)(theta * (180.0) /  M_PI) % 360) + 360) % 360;
}

const double step_size = 0.01;
int compute_count = 0;

void compute(char file[], double step_size) {
  init_store(file);
  for (double tf = 0; tf <= 0.00; tf += step_size)
  {
    for (double v = step_size; v <= 20; v += step_size)
    {
      for (double a = -10; a <= 10; a += step_size)
      {
        Trajectory tr = find_rotation(0.05, tf/2, tf, a/2, v);
        key[0] = (int)(v * sig_fig);
        key[1] = (int)(a * sig_fig);
        key[2] = ((int)(to_degrees(tr.dtheta))) % 360;
        key[3] = (int)(tr.dx * sig_fig);
        key[4] = (int)(tr.dy * sig_fig);
        key[5] = (int)(tf * sig_fig);
        store(key);
      }
    }
    cout << compute_count << " " << tf << "/5" << endl;
  }
  end_store();
  compute_count++;
}

int main() {
  char file5[] = "04082018.3.bin";
  compute(file5, 0.01);
  //compute(file2, 0.02);
  //compute(file1, 0.01);
  return 0;
}
#include <iostream>
#include <fstream>
#include <cstdio>
#include <map>
#include <math.h>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <string>
#include <boost/functional/hash.hpp> //need Boost library

using namespace std;
#define M_PI 3.14159265358979323846

struct Trajectory {
  bool fail;
  double dtheta;
  double dx;
  double dy;
  Trajectory ()
  {
    fail = false;
    dtheta = 0.0;
    dx = 0.0;
    dy = 0.0;
  }
};

const double w = 0.05236;
const double x = 0.8;
const double dtheta_limit = 2 * M_PI / 3;

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
    //cout << mid << endl;
    tr.dx += d * sin (tr.dtheta + mid);
    tr.dy += d * cos (tr.dtheta + mid);
    tr.dtheta += 2 * mid;
    if (tr.dtheta > dtheta_limit) {
      tr.fail = true;
      return tr;
    }
  }
  for (;t<=tf-tm;t+=incr) {
    double d = incr * ((2 * a * t) + b);
    mid = d * sin ((ang / 2.0) + ((incr * w) / 4.0)) / x;
    //cout << mid << endl;
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
    if (tr.dtheta > dtheta_limit) {
      tr.fail = true;
      return tr;
    }
  }
  if (tr.dtheta < 0) {
    tr.fail = true;
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
ofstream maxFile;
const int vector_size = 6;
const double sig_fig = 2.0;
vector<int> key;
vector<int> maxes;

void init_store(char file[], char file2[]) {
  for (int i = 0; i < vector_size; i++) {
    key.push_back(0);
    if (i != 0) {
      maxes.push_back(0);
    }
  }
  dataFile.open(file, ios::out | ios::binary);
  maxFile.open(file2, ios::out | ios::binary);
}

void store(vector<int> key) {
  for ( auto i = key.begin(); i != key.end(); i++ ) {
    dataFile << *i << " ";
    //dataFile.write((char*)(&(*i)), sizeof(int));
  }
  dataFile << endl;
}

void storeMax(vector<int> maxes) {
  for ( auto i = maxes.begin(); i != maxes.end(); i++ ) {
    maxFile << *i << " ";
    //dataFile.write((char*)(&(*i)), sizeof(int));
  }
  maxFile << endl;
}

void end_store() {
  dataFile << -100000;
  dataFile.flush();
  dataFile.close();
  maxFile << -100000;
  maxFile.flush();
  maxFile.close();
}

int to_degrees(double theta) {
  return (((int)(theta * (180.0) /  M_PI) % 360) + 360) % 360;
}

//const double step_size = 0.01;
int compute_count = 0;

void compute(char file[],char file2[], double step_size) {
  int count = 0;
  init_store(file,file2);
  for (double a = -10 + 0.01; a <= 20; a += step_size)
  {
    for (double v = step_size + 0.01; v <= 10; v += step_size)
    {
      maxes[0] = (int)(a * sig_fig);
      maxes[1] = (int)(v * sig_fig);
      for (int i = 2; i < vector_size - 1; i++) {
        maxes[i] = 0;
      }

      for (double tf = 0.21; tf <= 8; tf += step_size)
      {
        Trajectory tr = find_rotation(0.05, tf/2, tf, a/2, v);
        if (tr.fail)
          break;
        key[0] = (int)(v * sig_fig);
        key[1] = (int)(a * sig_fig);
        key[2] = (int)(to_degrees(tr.dtheta) / 5);
        key[3] = (int)(tr.dx * sig_fig);
        key[4] = (int)(tr.dy * sig_fig);
        key[5] = (int)(tf * sig_fig);
        store(key);
        maxes[2] = max(maxes[2], key[2]);
        maxes[3] = max(maxes[3], key[3]);
        maxes[4] = max(maxes[4], key[4]);
        count++;
      }
      storeMax(maxes);
    }
    cout << count << " " << (a+10) << "/30" << endl;
  }
  end_store();
  cout << count;
  compute_count++;
}

int main() {
  char file1[] = "04072018.3.bin";
  char file2[] = "04072018m.3.bin";
  compute(file1, file2, 0.05);
  //compute(file2, 0.02);
  //compute(file1, 0.01);
  return 0;
}
#include <iostream>
#include <fstream>
#include <cstdio>
#include <unordered_map>
#include <map>
#include <vector>
#include <string>
#include <boost/functional/hash.hpp> //need Boost library

using namespace std;

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
map<int,bool> visited;

const char file[] = "04072018.3.bin";
const char file2[] = "04072018m.3.bin";
const int size = 6;
vector<int> key;

int maxSum;
int dthetaScale;
int dxScale;
int dyScale;

bool checkSum(int dtheta, int dx, int dy) {
  return ((dthetaScale * dtheta) + (dxScale * dx) + (dyScale * dy) < maxSum);
}

int floodrecurse(int a, int v, int dtheta, int dx, int dy) {
  key[2] = dtheta;
  key[3] = dx;
  key[4] = dy;
  //cout << dtheta << " " << dx << " " << dy << endl;
  int access = hashTbl[key];
  if (access != 0)
    return access;
  if (!checkSum(dtheta,dx,dy))
    return 1000000;
  //cout << dtheta << " " << dx << " " << dy << endl;
  int dtp = floodrecurse(a,v,dtheta+1,dx,dy);
  int dxp = floodrecurse(a,v,dtheta,dx+1,dy);
  int dyp = floodrecurse(a,v,dtheta,dx,dy+1);
  hashTbl[key] = min(min(dtp,dxp),dyp);
  return min(min(dtp,dxp),dyp);
}

void floodfill(int a, int v, int dtheta, int dx, int dy) {
  if (visited[100000 * a + v])
    return;
  visited[100000 * a + v] = true;
  key[0] = v;
  key[1] = a;
  dthetaScale = dx * dy;
  dxScale = dtheta * dy;
  dyScale = dtheta * dx;
  maxSum = dtheta * dx * dy / 2;
  for (int dth = 0; checkSum(dth, 0, 0); dth++) {
    for (int dxi = 0; checkSum(dth, dxi, 0) ; dxi++) {
      for (int dyi = 0; checkSum(dth, dxi, dyi) ;dyi++) {
        floodrecurse(a,v,dth,dxi,dyi);
      }
    }
  }
}

void init() {
  int retrieve = 0;
  int value;
  ifstream dataFile;
  dataFile.open(file);
  for (int i = 0; i < size - 1; i++) {
    key.push_back(0);
  }

  //access table
  int count = 0;
  while (retrieve != -100000) {
    for (int i = 0; i < size - 1; i++) {
      dataFile >> retrieve;
      //cout << retrieve << endl;
      key[i]=retrieve;
    }
    dataFile >> value;
    hashTbl[key] = value;
    count++;
    if (count % 1000000 == 0) cout << count << endl;
  }
  cout << "final count: " << count << endl;

  ifstream maxFile;
  maxFile.open(file2);
  cout << "starting to flood fill" << endl;
  while (!maxFile.eof()) {
    int a, v, dtheta, dx, dy;
    maxFile >> a;
    maxFile >> v;
    maxFile >> dtheta;
    maxFile >> dx;
    maxFile >> dy;
    floodfill(a,v,dtheta,dx,dy);
    // cout << a << " " << v << endl;
  }

  cout << "ending flood fill" << endl;

  dataFile.close();
  maxFile.close();
}

int main() {
  init();
  vector<int> key;
  /*for (int i = 0; i < 1000; i++)
  {
    key.push_back(1995);
    key.push_back(230);
    key.push_back(121);
    key.push_back(4894);
    key.push_back(2508);
    cout << hashTbl[key];
  }*/
  return 0;
}
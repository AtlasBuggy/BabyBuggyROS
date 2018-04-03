#include <iostream>
#include <fstream>
#include <cstdio>
#include <unordered_map>
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

const char file[] = "step_size";
const int size = 6;

void init() {
  vector<int> key;
  int retrieve = 0;
  int value;
  ifstream dataFile;
  dataFile.open(file);
  for (int i = 0; i < size - 1; i++) {
    key.push_back(0);
  }
 
  while (retrieve != -100000) {
    for (int i = 0; i < size - 1; i++) {
      dataFile >> retrieve;
      //cout << retrieve << endl;
      key[i]=retrieve;
    }
    dataFile >> value;
    hashTbl[key] = value;
  }
 
  dataFile.close();
}

int main() {
  init();
  vector<int> key;
  key.push_back(1995);
  key.push_back(230);
  key.push_back(121);
  key.push_back(4894);
  key.push_back(2508);
  cout << hashTbl[key];
  return 0;
}
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

const char file[] = "steering.txt";
ofstream dataFile;

void init_store() {
  dataFile.open(file);
}

void store(vector<int> key) {
  for ( auto i = key.begin(); i != key.end(); i++ ) {
    dataFile << *i << " ";
  }
  dataFile << -1 << endl;
}

void end_store() {
  dataFile.close();
}

void init() {
  vector<int> key;
  int retrieve;
  ifstream dataFile;
  dataFile.open(file);
 
  while (retrieve != -1) {
    dataFile >> retrieve;
    key.push_back(retrieve);
    cout << retrieve << " ";
  }
  key.pop_back();
  int value = key.back();
  key.pop_back();

  hashTbl[key] = value;

  cout << hashTbl[key];
 
  dataFile.close();
}

int main() {
  init_store();
  vector<int> key;
  for (int j = 0; j < 100; j++) {
    for (int i = j; i <= 100; i++) {
      key.push_back(i);
    }
    store(key);
  }
  end_store();
  init();
  return 0;
}
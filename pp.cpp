#include <stdlib.h>
#include <iostream>
#include <utility>
#include <deque>
#include <iterator>
#include <cmath>

constexpr float LAD_COEFF = 1;
constexpr float WHEEL_BASE = 0.5;

using namespace std;

float distance(pair<float, float> x, pair<float, float> y)
{
	return sqrt(pow(x.first - y.first, 2.0) + pow(x.second - y.second, 2.0));
}

class robot {
public:
	float pp_control();
	float pid_control();
	void load_path(deque<pair<float, float>> data);
	void locate();

private:
	float x;
	float y;
	float z;
	float ori;
	float speed;
	float turn_angle;
	int index;
	deque<pair<float, float>> path;
};

void robot::load_path(deque<pair<float, float>> data)
{
	path = data;
}

void robot::locate()
{
	float dist, min_dist = -1;
	deque<pair<float, float>>::iterator i;
	for (i = path.begin(); i !=path.end(); ++i)
	{
		dist = distance(make_pair(x, y), *i);
		if (min_dist == -1 || dist < min_dist)
		{
			min_dist = dist;
			index = distance(path.begin(), i);
		}
	}
}

float robot::pp_control()
{
	float look_ahead_distance = LAD_COEFF;
	pair<float, float> curr_ind_pos = path[index];
	int target = index;
	while (look_ahead_distance > 0)
	{
		look_ahead_distance -= distance(path[target], path[(++target)%path.size]);
	}
	float global_x_diff = path[target].first - x;
	float global_y_diff = path[target].second - y;
	float dist_diff = distance(path[target], make_pair(x, y));
	float global_deg = atan(global_y_diff / global_x_diff);
	float pos_deg = global_deg - ori;
	float pos_x_diff = dist_diff * cos(pos_deg);
	float pos_y_diff = dist_diff * sin(pos_deg);
	float steer_angle = atan(2 * WHEEL_BASE*pos_x_diff / dist_diff);
}

int main()
{
	return 0;
}
#include <stdlib.h>
#include <iostream>
#include <utility>
#include <deque>
#include <iterator>
#include <cmath>

constexpr double LAD_COEFF = 10;
constexpr double WHEEL_BASE = 0.5;
constexpr double ORI_OFFSET = 0.0;

using namespace std;

double distance(pair<double, double> x, pair<double, double> y)
{
	return sqrt(pow(x.first - y.first, 2.0) + pow(x.second - y.second, 2.0));
}

class Robot {
	public:
		Robot (bool init);
		double pp_control();
		double pid_control();
		void load_path(deque<pair<double, double>> data);
		void load_dr_path(deque<pair<double, double>> data);
		void locate_on_map();
		void amcl_update_pose(double new_x, double new_y, double new_z, double new_ori);
		void dr_update_pose(double new_x, double new_y, double new_z, double new_ori);
		void update_speed(double new_speed);
		int index;
		int target_index;
		int vision_direction;
		bool use_amcl;
		bool is_manual;

	private:
		double x;
		double y;
		double z;
		double ori;
		double speed;
		double turn_angle;
		deque<pair<double, double>> path;
		deque<pair<double, double>> dr_path;
	};

	Robot::Robot(bool init)
	{
		use_amcl = init;
		is_manual = true;
		index = 0;
	}

	void Robot::amcl_update_pose(double new_x, double new_y, double new_z, double new_ori)
	{
		if (use_amcl) {
			x = new_x;
			y = new_y;
			z = new_z;
			ori = new_ori;
		}
	}

	void Robot::dr_update_pose(double new_x, double new_y, double new_z, double new_ori)
	{
		if (!use_amcl) {
			x = new_x;
			y = new_y;
			z = new_z;
			ori = new_ori + ORI_OFFSET;
		}
	}

	void Robot::update_speed(double new_speed)
	{
		speed = new_speed;
	}

	void Robot::load_path(deque<pair<double, double>> data)
	{
		path = data;
	}

	void Robot::load_dr_path(deque<pair<double, double>> data)
	{
		dr_path = data;
	}

	void Robot::locate_on_map()
	{
		deque<pair<double, double>> working_path;

		/*if (!use_amcl) {
			working_path = dr_path;
		}
		else {
			working_path = path;
		}*/

		double dist, min_dist = -1;
		deque<pair<double, double>>::iterator i;
		for (i = path.begin(); i != path.end(); ++i)
		{
			dist = distance(make_pair(x, y), *i);
			if (min_dist == -1 || dist < min_dist)
			{
				min_dist = dist;
				index = distance(path.begin(), i);
			}
		}
	}

	double Robot::pp_control()
	{
		double look_ahead_distance = LAD_COEFF;
		locate_on_map();
		pair<double, double> curr_ind_pos = path[index];
		int target = index;
		while (look_ahead_distance > 0)
		{
			look_ahead_distance -= distance(path[target], path[(target+1)%path.size()]);
			target++;
		}
		target_index = target;
		double global_x_diff = path[target].first - x;
		double global_y_diff = path[target].second - y;
		double dist_diff = distance(path[target], make_pair(x, y));
		double global_deg = atan(global_y_diff / global_x_diff);
		double pos_deg = global_deg - ori;
		double pos_y_diff = dist_diff * cos(pos_deg);
		double pos_x_diff = dist_diff * sin(pos_deg);

		double robot_y = cos(ori)*global_y_diff - sin(ori)*global_x_diff;
		double robot_x = cos(ori)*global_x_diff + sin(ori)*global_y_diff;

		double new_steer_angle = atan(2 * WHEEL_BASE*robot_y / dist_diff);

		double steer_angle = atan(2 * WHEEL_BASE*pos_x_diff / dist_diff);
		return new_steer_angle;
	}

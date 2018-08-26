

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// consts added for easy tuning
const int NUM_LANES = 3; // there are three lanes on the road.
const double LANE_WIDTH = 4.0; // lane width in meters
const double SPEED_LIMIT = 49.5;
const double MIN_ALLOWABLE_DIST_TO_CAR_INFRONT = 30.0; // dont get closer than this dist
const double SPEED_ADJUSTMENT = .224; // amount to accel / decel per waypoint update - will affect acceleration (and jerk)

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// utility funcs for planning
double getLaneMidPoint(int laneNo) {return LANE_WIDTH / 2.0 + LANE_WIDTH * laneNo; }

// determines if a given d value lies within a given lane number
bool isDInLane(int laneNo, double d) { return d < (LANE_WIDTH / 2.0 + LANE_WIDTH * laneNo + LANE_WIDTH / 2.0) && d >(LANE_WIDTH / 2.0 + LANE_WIDTH * laneNo - LANE_WIDTH / 2.0); }


struct vehicle_state
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;

	double acceleration;

	double speed() { return std::sqrt(vx * vx + vy * vy); }

	double predictNewSValue(double timeInFuture)
	{
		return s + speed() * timeInFuture + 0.5 * acceleration * pow(timeInFuture, 2.0);
	}

	int getLaneIndex(double laneWidth) { return static_cast<int>(d / laneWidth); }
};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  static int _myLane = 1; // lane to drive in. We start in lane 1

  // ref velocity to target
  static double ref_vel = 0; // mph

  static map<int, vehicle_state> _vehicleStates; // history of detected vehicles

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

			size_t prev_size = previous_path_x.size();

			double current_car_s = car_s;
			if (prev_size > 0)
			{
				car_s = end_path_s;
			}

			// path planning logic:
			// 1. if there is no can in front of us, or we can accelerate up to max speed, stay in lane
			// 2. if we get too close to can in front, switch lanes
			// 3. 

			bool too_close = false;

			double maxDeceleration = 0.0;

			double timeHorizon = prev_size * .02; // how far out into the future are we predicting

			// array to organize cars into lanes - we can use this to make safe lane changes if we want
			std::array<std::vector<int>, NUM_LANES> _laneList; // 3xN array (there are three lanes)

			// find ref_v to use
			for (size_t i = 0; i < sensor_fusion.size(); i++)
			{				
				// determine if detected vehicle is in my lane...

				// get all car vars
				vehicle_state state;
				state.id = sensor_fusion[i][0];
				state.x = sensor_fusion[i][1];
				state.y = sensor_fusion[i][2];
				state.vx = sensor_fusion[i][3];
				state.vy = sensor_fusion[i][4];
				state.s = sensor_fusion[i][5];
				state.d = sensor_fusion[i][6];

				// record car id aginst road lane

				int laneIdx = state.getLaneIndex(LANE_WIDTH);
				if(laneIdx >= 0 && laneIdx < NUM_LANES)
					_laneList[state.getLaneIndex(LANE_WIDTH)].push_back(state.id);

				// if car is not in my lane, we don't care so we can accelerate to max speed, check the next detection to see if that is in the way
				// if they are in lane we need to check further what to do (if anything)
				if (isDInLane(_myLane, state.d))
				{
					// this car is in my lane!

					vehicle_state last_state; // get the car's last state
					last_state = _vehicleStates[state.id];

					double changeInSpeed = 0.0;
					if (state.id == last_state.id) // checks if we had a value in the array from last loop round
					{
						changeInSpeed = state.speed() - last_state.speed();

						if (changeInSpeed < maxDeceleration)
							maxDeceleration = changeInSpeed;
					}

					state.acceleration = changeInSpeed / 0.02; // record car acceleration incase we need it for a lane change

					// we are keeping a running list of waypoints (up to 50 spread over 1 sec - 0.02 apart)
					// predict where other car will be at it's current speed in future (based off how far out we are plotting our next waypoint)

					double check_car_s = state.predictNewSValue(timeHorizon);

					// check s values greater than our own and s is not too close

					if (((check_car_s > car_s) && (check_car_s - car_s) < MIN_ALLOWABLE_DIST_TO_CAR_INFRONT) || 
						(check_car_s > current_car_s && check_car_s <= car_s))
					{
						too_close = true;

						// so at this point we are too close to the car in front. Options are:
						// 1. change lanes
						// 2. brake

						 // we deal with that below
					}
				}


				// record last state
				_vehicleStates[state.id] = state;
			}

			// speed up or slow down, or change lanes
			if (too_close)
			{
				// select lanes we could move to
				vector<int> possibleLanes;

				if (_myLane - 1 >= 0)
					possibleLanes.push_back(_myLane - 1);
				if(_myLane + 1 < NUM_LANES)
					possibleLanes.push_back(_myLane + 1);

				int lane1Idx = -1; // will be set to lane idx if safe option, otherwise -1
				int lane2Idx = -1; // will be set to lane idx if safe option, otherwise -1
				int lane1ClosestCarId = -1;
				int lane2ClosestCarId = -1;

				// loop through each possible lane
				for (size_t laneIdx = 0; laneIdx < possibleLanes.size(); laneIdx++)
				{
					bool laneSafe = true;

					double closestCarAheadOfUs = std::numeric_limits<double>::max();
					int closestCarId = -1;

					// loop through each car
					for (size_t carIdx = 0; carIdx < _laneList[possibleLanes[laneIdx]].size(); carIdx++)
					{
						int vehicle_id = _laneList[possibleLanes[laneIdx]][carIdx];

						vehicle_state vState = _vehicleStates[vehicle_id];

						// predict where the other car will be in x seconds from now
						double check_car_s = vState.predictNewSValue(timeHorizon);

						double sDiff = check_car_s - car_s;

						// if this car would be in front or behind within min margin, do not use this lane - we want to be safe as possible
						if (abs(sDiff) < MIN_ALLOWABLE_DIST_TO_CAR_INFRONT)
						{
							laneSafe = false;
							break;
						}

						// Record closest car that would be in front of our target waypoint in new lane. 
						// Use this to decide which is faster lane to select if both open
						if (sDiff > 0 && sDiff < closestCarAheadOfUs) // then the car is ahead of us
						{
							if(sDiff <= MIN_ALLOWABLE_DIST_TO_CAR_INFRONT *1.2)
								closestCarId = vehicle_id;
						}
					}

					if (laneSafe)
					{
						if (laneIdx == 0)
						{
							lane1Idx = possibleLanes[laneIdx];
							lane1ClosestCarId = closestCarId;
						}
						else if (laneIdx == 1)
						{
							lane2Idx = possibleLanes[laneIdx];
							lane2ClosestCarId = closestCarId;
						}
					}
				}

				int bestLane = -1; // this is the lane we choose, if no lane chosen, then we will brake car

				// now we can select optimal lane based on whatever criteria we want...

				if (lane1Idx > -1 && lane2Idx > -1)
				{
					if (lane1ClosestCarId == -1) // use first lane if clear (will prefer left lane if available due to ordering)
					{
						bestLane = lane1Idx;
					}
					else if (lane2ClosestCarId == -1) // use right lane if clear
					{
						bestLane = lane2Idx;
					}
					else
					{
						if (_vehicleStates[lane1ClosestCarId].acceleration >= 0 && _vehicleStates[lane1ClosestCarId].speed())
						{
							bestLane = lane1Idx;
						}
						else if (_vehicleStates[lane2ClosestCarId].acceleration >= 0 && _vehicleStates[lane2ClosestCarId].speed())
						{
							bestLane = lane2Idx;
						}
					}
				}
				else if (lane1Idx != -1)
				{
					// use this lane if no car in it, or it is faster than us and not decelerating
					if (lane1ClosestCarId == -1 ||
						(lane1ClosestCarId != -1 && _vehicleStates[lane1ClosestCarId].acceleration >= 0 && _vehicleStates[lane1ClosestCarId].speed() > car_speed))
					{
						bestLane = lane1Idx;
					}
				}
				else if (lane2Idx != -1)
				{
					// use this lane if no car in it, or it is faster than us and not decelerating
					if (lane2ClosestCarId == -1 ||
						(lane2ClosestCarId != -1 && _vehicleStates[lane2ClosestCarId].acceleration >= 0 && _vehicleStates[lane2ClosestCarId].speed() > car_speed))
					{
						bestLane = lane2Idx;
					}
				}

				// now either set the lane or brake

				if (bestLane > -1)
				{
					_myLane = bestLane;
				}
				else
				{
					// if we could not change lanes then brake...
					ref_vel -= max(SPEED_ADJUSTMENT*1.7, -maxDeceleration);
				}
			}
			else if(ref_vel < SPEED_LIMIT) // accelerate but dont break speed limit
			{
				ref_vel += SPEED_ADJUSTMENT*1.7;
			}
			

          	json msgJson;

			vector<double> ptsx;
			vector<double> ptsy;

			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);

			if (prev_size < 2)
			{
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);

				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			}
			else
			{
				ref_x = previous_path_x[prev_size - 1];
				ref_y = previous_path_y[prev_size - 1];

				double ref_x_prev = previous_path_x[prev_size - 2];
				double ref_y_prev = previous_path_y[prev_size - 2];
				ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
			}

			// in Frenet add evenly 30m spaced points of the starting reference

			vector<double> next_wp0 = getXY(car_s + 30, getLaneMidPoint(_myLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s + 60, getLaneMidPoint(_myLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s + 90, getLaneMidPoint(_myLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);

			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);

			for (size_t i = 0; i < ptsx.size(); i++)
			{
				// shift car ref angle to 0 degrees

				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
				ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
			}

			//create a spline
			tk::spline s;

			// set (x,y) points to the spline
			s.set_points(ptsx, ptsy);

			// actual x,y points we will use for the planner
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

			for (size_t i = 0; i < previous_path_x.size(); i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			// calculate how to break up spline points so that we travel at our desired ref velocity
			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

			double x_add_on = 0;

			// fill up the rest of the path planner after filling it with the previous points, here we will always output 50 pts
			for (size_t i = 1; i <= 50 - previous_path_x.size(); i++)
			{
				double N = (target_dist / (.02*ref_vel / 2.24));
				double x_point = x_add_on + (target_x) / N;
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				// rotate back to normal after rotating earlier
				x_point = (x_ref*cos(ref_yaw) - y_ref * sin(ref_yaw));
				y_point = (x_ref*sin(ref_yaw) + y_ref * cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

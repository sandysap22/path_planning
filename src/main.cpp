#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

  int lane = 1;

  double ref_vel = 0.0;

  h.onMessage([&lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

            int prev_size = previous_path_x.size();


          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

           //ref_vel = 49.5;

          	/* Sensor fusion code */

            if(prev_size>0){
                car_s = end_path_s;
            }

            bool too_close = false;
            bool zero_lane_open = true;
            bool one_lane_open = true;
            bool two_lane_open = true;

            double lower_margin = car_s - 50;
            double upper_margin = car_s + 50;
            double front_vehicle_speed = 50.0;

            double front_left_closet = 50.0;
            double front_mid_closet = 50.0;
            double front_right_closet = 50.0;
            double rare_left_closet = -50.0;
            double rare_mid_closet = -50.0;
            double rare_right_closet = -50.0;

            bool too_too_close = false;

            // 45.15
            for(int i=0;i < sensor_fusion.size();i++) {

              double check_car_s = sensor_fusion[i][5];

              if(check_car_s > lower_margin && check_car_s < upper_margin){

                float d = sensor_fusion[i][6];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);

                // 45.36 check when this car would be after 0.02 * prev_size time (~ 1 sec)
                check_car_s+=((double)prev_size*0.02*check_speed);

                if(d > (4*lane) && d < (4*lane + 4)) {

                  if( check_car_s > car_s && (check_car_s - car_s) < 30 ) {
                          std::cout << "there is car in front of me" << endl;
                          //ref_vel = 29.5; //mph
                          too_close = true;
                          front_vehicle_speed = check_speed;

                          if((check_car_s - car_s) < 15){
                            too_too_close=true;
                          }
                  }

                }
                double dist_of_other_car = check_car_s - car_s;
                if(d > 0 && d < 4) {
                  std::cout << "L :  "  << dist_of_other_car ;
                  zero_lane_open = false;
                  if(dist_of_other_car >= 0 && dist_of_other_car < front_left_closet){
                    front_left_closet = dist_of_other_car;
                  }else if (dist_of_other_car < 0 && dist_of_other_car > rare_left_closet){
                    rare_left_closet = dist_of_other_car;
                  }
                }
                if (d > 4 && d < 8) {
                  std::cout << "M :  " << dist_of_other_car;
                  one_lane_open = false;
                  if(dist_of_other_car >= 0 && dist_of_other_car < front_mid_closet){
                    front_mid_closet = dist_of_other_car;
                  }else if (dist_of_other_car < 0 && dist_of_other_car > rare_mid_closet){
                    rare_mid_closet = dist_of_other_car;
                  }
                }
                if (d > 8 && d < 12) {

                  std::cout << "R :  "  << dist_of_other_car;
                  two_lane_open = false;

                  if (dist_of_other_car >= 0 && dist_of_other_car < front_right_closet){
                    front_right_closet = dist_of_other_car;
                  } else if (dist_of_other_car < 0 && dist_of_other_car > rare_right_closet){
                    rare_right_closet = dist_of_other_car;
                  }

                } // this if
                std::cout << endl;
              } // if within margin
            }

          bool  multiple_lane_change = false; // from 0 to 2 or vise versa
          bool slow_down=false;
          if(too_close) { // 50.0
            if(too_too_close){
              if(lane==0) {
                if(one_lane_open){
                  lane=1;
                  std::cout << "changing from L to M"  << endl ;
                } else if(two_lane_open){
                  if(rare_mid_closet < -10.0 && front_mid_closet > 20){
                    std::cout << "================ savdan ======================== " << endl;
                    std::cout << "planning for L to M to R"  << endl ;
                    lane=1;
                    multiple_lane_change=true;
                  } else {
                    std::cout << "OK, right lane is open but there is car in mid lane" << endl;
                    slow_down = true;
                  }
                } else slow_down = true;
              } else if(lane==1) {
                if(zero_lane_open){
                  std::cout << "changing from M to L"  << endl ;
                  lane = 0;
                }else if(two_lane_open){
                  std::cout << "changing from M to R"  << endl ;
                  lane = 2;
                } else slow_down = true;
              } else if (lane==2) {
                if(one_lane_open) {
                  std::cout << "changing from R to M" << endl ;
                  lane=1;
                } else if(zero_lane_open) {
                  if(rare_mid_closet < -10.0 && front_mid_closet > 20){
                    std::cout << "================ savdan ======================== " << endl;
                    std::cout << "Planning for R to M to L" << endl ;
                    lane=1;
                    multiple_lane_change=true;
                  } else {
                    std::cout << "OK, left lane is open but there is car in mid lane" << endl;
                    slow_down = true;
                  }
                } else slow_down = true;

              } else  slow_down = true;

            } // too_too_close
            if(slow_down) { // if lane change is not feasible
              // as other lanes also not have scope
              std::cout << "there is no scope but to slow down " << endl ;

              if( front_vehicle_speed < ref_vel * 0.8){
                  std::cout << "next vehicle is too slow " << endl;
                  ref_vel -= 0.35;
                  if( front_vehicle_speed < ref_vel * 0.7){
                    ref_vel -= 0.45;
                  }
              }  else {
                ref_vel -= 0.224; // 5 m/sec*sec
              }
            }

          } else if ( ref_vel < 49.5) {
            // there is no vehicle close in front so can increase speed
            if(ref_vel < 15) {
              ref_vel +=0.35;
            } else {
              ref_vel +=0.224;
            }
          }

            /**
            if(too_close){ // 50.0
              ref_vel -=0.224; // 5 m/sec*sec
            }
            */

            vector<double> ptx; // 25.29
            vector<double> pty;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // std::cout << " prev size " << previous_path_x.size() << " " << endl;
            //
            // for(int i=0; i < previous_path_x.size(); i++){
            //   std::cout << "(" << previous_path_x[i] << "," << previous_path_y[i] << ")," ;
            // }

            std::cout << endl;

            // get car orientation based on previous points

            // if there are no old waypoint references

            if ( prev_size < 2){

              std::cout << " less than 2 prv points " << endl;

              double prev_car_x =  car_x - cos(car_yaw); // point tangent to car path
              double prev_car_y =  car_y - sin(car_yaw);


              ptx.push_back(prev_car_x);
              pty.push_back(prev_car_y);

              //ptx.push_back(car_x);
              //pty.push_back(car_y);

              ptx.push_back(ref_x);
              pty.push_back(ref_y);


            } else {

              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              ptx.push_back(ref_x_prev);
              pty.push_back(ref_y_prev);

              ptx.push_back(ref_x);
              pty.push_back(ref_y); // 26.3
            }

            double sharp_turn_factor = 1;

            if (multiple_lane_change){
                sharp_turn_factor = 1.5;
            }

            // create sparse points  28.00
            for(int i = 0; i < 3; i++)
            {
              double next_s = car_s + 30 * (i+1) * sharp_turn_factor;
              double next_d = 2.0+lane*4.0;  // 2, 6, 10
              vector<double>  xy= getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);

              ptx.push_back(xy[0]);
              pty.push_back(xy[1]);
            }


            // std::cout << " ptx points " <<endl;
            // for(int i=0; i < ptx.size(); i++){
            //   std::cout << "(" << ptx[i] << "," << pty[i] << ")," ;
            // }
            // std::cout << endl;

            // assume initial angle as zero and change other angles in 5 referance points
            // 29.30
            for(int i = 0;i <ptx.size();i++){

              double shift_x = ptx[i] - ref_x;
              double shift_y = pty[i] - ref_y;

              ptx[i]= (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              pty[i]= (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));

            }

            // std::cout << " ptx points after anlge change " <<endl;
            // for(int i=0; i < ptx.size(); i++){
            //   std::cout << "(" << ptx[i] << "," << pty[i] << ")," ;
            // }
            // std::cout << endl;

            tk::spline s; // 29.33

            s.set_points(ptx,pty);

            // add previous points to next path  29.54
            for(int i=0; i < previous_path_x.size(); i++){

              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);

            }

            // 30.48
            double target_x = 30;

            if (multiple_lane_change){
              std::cout << "multiple lane change " << std::endl;
              target_x = 50;
            }

            double target_y = s(target_x);
            double target_dist = distance(0,0,target_x,target_y);

            double x_add_on = 0;

            // 38.04
            double N = (target_dist/(0.02*ref_vel/2.24)); // 2.24 for miles to m/s
            double one_segment = target_x / N;

            //std::cout << "N " << N << " one_segment "<< one_segment  << endl;

            // 31.49
            for(int i = 1; i <= 50 -  previous_path_x.size(); i++){
              // 38.04
              double x_point = x_add_on + one_segment;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;


              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));  // covert to original co ordinate system
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point +=ref_x;
              y_point +=ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

            } // 38.55

            // std::cout << " next points :" << next_x_vals.size() << endl;
            // for(int i=0; i < next_x_vals.size(); i++){
            //   std::cout << "(" << next_x_vals[i] << "," << next_y_vals[i] << ")," ;
            // }
            //
            // std::cout << endl;
            std::cout << "===========================================" << endl;

            /**

            double dist_inc = 0.3;


            for(int i = 0; i < 50; i++)
            {
                  double next_s = car_s + dist_inc * (i+1);
                  double next_d = 6; // 14.25
                  vector<double>  xy= getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);

                  next_x_vals.push_back(xy[0]);
                  next_y_vals.push_back(xy[1]); // 16.19
            }
            **/

            // end
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
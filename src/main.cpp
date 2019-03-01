#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    //lane id for start
    int lane_id = 1;
    //target velocity
    double tgt_vel = 5; //mph, set it to be low to have better start, and it is changing in the program

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
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



    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
                        &map_waypoints_dx,&map_waypoints_dy,&lane_id,&tgt_vel]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
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
                    double car_speed = j[1]["speed"];//mph

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals; //to feed in the path output
                    vector<double> next_y_vals; //to feed in the path output

                    vector<double> pts_x; //to store points sequence to generate spline
                    vector<double> pts_y; //to store points sequence to generate spline

                    //reference car position
                    double pos_x = car_x;
                    double pos_y = car_y;
                    double angle = deg2rad(car_yaw);

                    int pre_path_size = previous_path_x.size();

                    if(pre_path_size>0){
                        car_s = end_path_s;
                    }

                    bool too_close = false;
                    bool left_front_ok = false;
                    bool left_back_ok = false;
                    bool right_front_ok = false;
                    bool right_back_ok = false;

                    double safe_dist = 25;//meters
                    double safe_spd_diff = 5;//mph
                    double mps2mph = 2.24; //m/s to mile per hour

                    double straight_front_dist = 500;//meters
                    double straight_front_spd = 100; //mph

                    double left_front_dist = 500;//meters
                    double left_front_spd = 100; //mph
                    double left_back_dist = 500;//meters
                    double left_back_spd = 1; //mph

                    double right_front_dist = 500;//meters
                    double right_front_spd = 100; //mph
                    double right_back_dist = 500;//meters
                    double right_back_spd = 1; //mph

                    double d0 = 2+4*(lane_id-1)-2;
                    double d1 = 2+4*(lane_id)-2;
                    double d2 = 2+4*(lane_id+1)-2;
                    double d3 = 2+4*(lane_id+2)-2;

                    for(int i=0;i<sensor_fusion.size();i++){

                        float d = sensor_fusion[i][6];
                        if (d<0) continue; //don't care about cars on the other side of the road

                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_speed = sqrt(vx*vx + vy*vy);//m/s
                        double check_s = sensor_fusion[i][5];

                        check_s += check_speed*0.02*pre_path_size;//predict the target's position

                        //target is right one lane
                        if(d > d2 && d < d3 ){
                            if( (check_s-car_s)>0 && (check_s-car_s)<right_front_dist){
                                right_front_dist = check_s-car_s;
                                right_front_spd = check_speed*mps2mph;
                            }else if( (car_s-check_s)>0 && (car_s-check_s)<right_back_dist){
                                right_back_dist = car_s-check_s;
                                right_back_spd = check_speed*mps2mph;
                            }
                            continue;
                        }

                        //target is left one lane
                        if(d < d1  && d > d0){
                            if( (check_s-car_s)>0 && (check_s-car_s)<left_front_dist){
                                left_front_dist = check_s-car_s;
                                left_front_spd = check_speed*mps2mph;
                            }else if( (car_s-check_s)>0 && (car_s-check_s)<left_back_dist){
                                left_back_dist = car_s-check_s;
                                left_back_spd = check_speed*mps2mph;
                            }
                            continue;
                        }

                        //target is in the same lane
                        if(d < d2 && d > d1){
                            if( (check_s-car_s)>0 && (check_s-car_s)<straight_front_dist ){
                                straight_front_dist = check_s-car_s;
                                straight_front_spd = check_speed*mps2mph;
                            }
                        }

                    }

                    if (straight_front_dist<safe_dist){
                        too_close = true;
                    }

                    if (   (left_front_dist>0.5*safe_dist && left_front_spd>car_speed)
                        || (left_front_dist>safe_dist && (car_speed-left_front_spd)<safe_spd_diff) ) {

                        if (lane_id>0) { //left lane change not available for left most lane
                            left_front_ok = true;
                        }
                    }

                    if (   (left_back_dist>0.5*safe_dist && left_back_spd<car_speed)
                        || (left_back_dist>safe_dist && (left_back_spd-car_speed)<safe_spd_diff) ) {

                        if (lane_id>0) { //left lane change not available for left most lane
                            left_back_ok = true;
                        }
                    }

                    if (   (right_front_dist>0.5*safe_dist && right_front_spd>car_speed)
                        || (right_front_dist>safe_dist && (car_speed-right_front_spd)<safe_spd_diff) ) {

                        if (lane_id<2) { //right lane change not available for right most lane
                            right_front_ok = true;
                        }
                    }

                    if ( (right_back_dist>0.5*safe_dist && right_back_spd<car_speed)
                         || (right_back_dist>safe_dist && (right_back_spd-car_speed)<safe_spd_diff) ) {

                        if (lane_id<2) { //right lane change not available for right most lane
                            right_back_ok = true;
                        }
                    }

                    //state machine
                    if(too_close && left_front_ok && left_back_ok
                        && straight_front_dist>0.4*safe_dist){

                        lane_id -= 1;//change to left lane
                        std::cout << "Changed to lane: "<<lane_id << '\n';

                    }else if(too_close && right_front_ok && right_back_ok
                        && straight_front_dist>0.4*safe_dist){

                        lane_id += 1;//change to right lane
                        std::cout << "Changed to lane: "<<lane_id << '\n';

                    } else if (too_close && car_speed>0.85*straight_front_spd) {     //stay in lane

                        tgt_vel -= 0.3; //tgt_vel is in mph
                        std::cout << "Too close to front car, slow down! In Lane "<<lane_id << '\n';

                    } else if(car_speed < 49.0){    //speed up
                        tgt_vel += 0.3;
                        if (tgt_vel>49.5) tgt_vel = 49.5;
                        std::cout << "Speed up in the same lane. In lane "<<lane_id << '\n';
                        std::cout<<"current speed is: "<<car_speed<<std::endl;
                    }


                    //store the unfinished points from last path and combine it with new path later
                    for (int i = 0; i < pre_path_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    if (pre_path_size < 2) { //the previous calculated path almost end, use the car's current position as starting reference
                        double pre_car_x = car_x - cos(car_yaw);
                        double pre_car_y = car_y - sin(car_yaw);
                        pts_x.push_back(pre_car_x);
                        pts_x.push_back(car_x);

                        pts_y.push_back(pre_car_y);
                        pts_y.push_back(car_y);

                        angle = deg2rad(car_yaw);
                    } else { //use the previous path's end point as starting reference
                        pos_x = previous_path_x[pre_path_size-1];
                        pos_y = previous_path_y[pre_path_size-1];

                        double pos_x2 = previous_path_x[pre_path_size-2];
                        double pos_y2 = previous_path_y[pre_path_size-2];
                        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

                        pts_x.push_back(pos_x2);
                        pts_x.push_back(pos_x);

                        pts_y.push_back(pos_y2);
                        pts_y.push_back(pos_y);
                    }

                    //add another 3 points ahead of starting reference to pts_x/y which are 30 m apart in Frenet coordinates
                    vector<double> next_wp0 = getXY(car_s+30,(2+4*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s+60,(2+4*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s+90,(2+4*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
                    pts_x.push_back(next_wp0[0]);
                    pts_x.push_back(next_wp1[0]);
                    pts_x.push_back(next_wp2[0]);

                    pts_y.push_back(next_wp0[1]);
                    pts_y.push_back(next_wp1[1]);
                    pts_y.push_back(next_wp2[1]);

                    //Convert the pts_x/y into local coordinates(i.e.ego vehicle coordinates) to do the spline interpolation
                    for(int i=0;i<pts_x.size();i++){
                        double shift_x = pts_x[i] - pos_x;
                        double shift_y = pts_y[i] - pos_y;

                        pts_x[i] = (shift_x*cos(0-angle) - shift_y*sin(0-angle));
                        pts_y[i] = (shift_x*sin(0-angle) + shift_y*cos(0-angle));
                    }

                    //create the spline
                    tk::spline s;
                    s.set_points(pts_x,pts_y);

                    //interpolate the spline so that the car will drive at expected speed
                    double tgt_x = 30.0;
                    double tgt_y = s(tgt_x);
                    double tgt_dist = sqrt((tgt_x*tgt_x) + (tgt_y*tgt_y));
                    //calculate the number of points to be interpolated based on the target speed
                    double N = tgt_dist/(tgt_vel/mps2mph * 0.02); //tgt_dist is in meters, 0.02s is the simulation excution time

                    //combine the spline interpolated points with unfinished previous path
                    for (int i = 0; i < 50-pre_path_size; ++i) {
                        double x_point = tgt_x/N * (i+1);
                        double y_point = s(x_point);

                        double x_interp = x_point*cos(angle) - y_point*sin(angle);
                        double y_interp = x_point*sin(angle) + y_point*cos(angle);

                        next_x_vals.push_back(x_interp + pos_x);
                        next_y_vals.push_back(y_interp + pos_y);
                    }



                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

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

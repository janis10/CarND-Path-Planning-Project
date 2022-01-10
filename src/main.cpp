#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/QR"

#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Initial conditions and constraints:
// Car's initial lane.
int lane = 1;
// Initial velocity.
double curr_vel = 0.0;
// Velocity increment (+/-).
double vel_step = 0.224;
// Acceleration constraint.
const double max_accel = 49.5;
// We consider cars within 30.0 meters range.
const double dist_thresh = 30.0;
// Planning horizon
const int plan_horizon = 50;
// Time step
const double dt = 0.02;

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

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                 &map_waypoints_dx,
                 &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                    size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message
        // event. The 4 signifies a websocket message The 2 signifies a
        // websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data -- provided by the
                    // simulator.
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

                    // Sensor Fusion Data, a list of all other cars on the same
                    // side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    // MAIN IMPLEMENTATION.
                    /*
                     * We define a path made up of (x,y) points that the car
                     * will visit sequentially every .02 seconds.
                     */

                    int prev_size = previous_path_x.size();

                    // 1. PREDICTION STEP.
                    /* Goal: predict the actions of other objects.
                     * e.g., for a vehicle predict its future trajectory.
                     */

                    // For this project, we only alert whether or not an
                    // identified car (left, right, ahead) is too close.
                    // In the future, we should replace with predicting the
                    // trajectory.

                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }

                    bool car_left = false, car_right = false, car_ahead = false;
                    // Iterate over sensor fusion data:
                    // a list of all other cars on the same side of the road.
                    // ["sensor_fusion"]:
                    // [unique ID, x position, y position, x velocity m/s,
                    // y velocity m/s, s position in frenet space,
                    // d position in frenet space].

                    for (int i = 0; i < sensor_fusion.size(); i++) {
                        // Get values from sensor fusion data:
                        double vx = sensor_fusion[i][3],
                               vy = sensor_fusion[i][4],
                               s_pos = sensor_fusion[i][5],
                               d_pos = sensor_fusion[i][6];

                        // This project assumes that the road has three lanes
                        // (4m width).
                        int detected_car_lane;
                        if (d_pos > 0.0 && d_pos < 4.0) {
                            detected_car_lane = 0;
                        } else if (d_pos > 4.0 && d_pos < 8.0) {
                            detected_car_lane = 1;
                        } else if (d_pos > 8.0 && d_pos < 12.0) {
                            detected_car_lane = 2;
                        }

                        // Predict next s_pos using measured velocities
                        s_pos +=
                            (double)prev_size * dt * sqrt(vx * vx + vy * vy);

                        if (detected_car_lane == lane) {
                            // A vehicle is on the same line.
                            // car_ahead is true, if s_pos > car_s and within
                            // the distance threshold.
                            car_ahead |= (s_pos > car_s) &&
                                         (s_pos - car_s < dist_thresh);

                        } else if ((detected_car_lane - lane) == -1) {
                            // A vehicle is on the left lane.
                            // car_left is true, if within the distance
                            // threshold.
                            car_left |= (car_s - s_pos > -dist_thresh) &&
                                        (car_s - s_pos < dist_thresh);

                        } else if ((detected_car_lane - lane) == 1) {
                            // A vehicle is on the right lane.
                            // car_right is true, if within the distance
                            // threshold.
                            car_right |= (car_s - s_pos > -dist_thresh) &&
                                         (car_s - s_pos < dist_thresh);
                        }
                    }

                    // 2. BEHAVIOR STEP.
                    /* Goal: determine the ego vehicle's behavior over time.
                     * e.g., stop at a traffic light or intersection,
                     * change lanes, accelerate/break, take a turn or make a
                     * maneuver.
                     */

                    // In this project we consider whether or not a car is
                    // ahead, and if we can overtake it.
                    // In the future we should replace it with a real-world
                    // behavior planner that decides the trajectory based on
                    // optimization problem cost functions.
                    if (car_ahead) {
                        // If a car is ahead, see if left or right lane is empty
                        // and change.
                        // If we cannot change, start breaking.
                        if (!car_left && lane > 0) {
                            lane--;
                        } else if (!car_right && lane != 2) {
                            lane++;
                        } else if (!car_left && lane != 2) {
                            lane++;
                        } else {
                            curr_vel -= vel_step;
                        }
                    } else if (curr_vel < max_accel) {
                        // If no car ahead, increase if possible.
                        curr_vel += vel_step;
                    }

                    // 3. TRAJECTORY STEP
                    /* Goal: determine the ego vehicle's trajectory based on the
                     * behavior decided above.
                     */
                    vector<double> ptsx, ptsy;

                    // Refrence state: [x, y, yaw].
                    vector<double> ref_state(3);

                    if (prev_size < 2) {
                        ref_state[0] = car_x;
                        ref_state[1] = car_y;
                        ref_state[2] = deg2rad(car_yaw);
                        // If we don't have enough data yet. Use the current
                        // position and backtrack one step. Use two points thats
                        // makes path tangent to the car.
                        double prev_car_x = car_x - cos(car_yaw);
                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        double prev_car_y = car_y - sin(car_yaw);
                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);

                    } else {
                        // Else, reference point is previous point.
                        ref_state[0] = previous_path_x[prev_size - 1];
                        ref_state[1] = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        // ref_state[2] = atan2(ref_state[1] - ref_y_prev,
                        //                      ref_state[0] - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_state[0]);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_state[1]);
                    }

                    // Setting up target points in the future.
                    for (int i = 0; i < 3; i++) {
                        vector<double> next_wp = getXY(
                            car_s + (i + 1) * 30, 2 + 4 * lane, map_waypoints_s,
                            map_waypoints_x, map_waypoints_y);

                        ptsx.push_back(next_wp[0]);
                        ptsy.push_back(next_wp[1]);
                    }
                    // vector<double> next_wp0 =
                    //     getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s,
                    //           map_waypoints_x, map_waypoints_y);
                    // vector<double> next_wp1 =
                    //     getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s,
                    //           map_waypoints_x, map_waypoints_y);
                    // vector<double> next_wp2 =
                    //     getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s,
                    //           map_waypoints_x, map_waypoints_y);
                    //
                    // ptsx.push_back(next_wp0[0]);
                    // ptsx.push_back(next_wp1[0]);
                    // ptsx.push_back(next_wp2[0]);
                    // ptsy.push_back(next_wp0[1]);
                    // ptsy.push_back(next_wp1[1]);
                    // ptsy.push_back(next_wp2[1]);

                    // Shift to local car coordinates.
                    for (int i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_state[0];
                        double shift_y = ptsy[i] - ref_state[1];

                        ptsx[i] = shift_x * cos(0 - ref_state[2]) -
                                  shift_y * sin(0 - ref_state[2]);
                        ptsy[i] = shift_x * sin(0 - ref_state[2]) +
                                  shift_y * cos(0 - ref_state[2]);
                    }

                    // Here we will store the generated waypoints from the
                    // planner.
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    // For smooth transition, add the previous path points
                    for (int i = 0; i < prev_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // Create the spline.
                    tk::spline target_spline;
                    target_spline.set_points(ptsx, ptsy);

                    // Get target point at 30m ahead.
                    double target_x = 30.0, target_y = target_spline(target_x);
                    double target_dist =
                        sqrt(target_x * target_x + target_y * target_y);

                    double x_add_on = 0;

                    for (int i = 1; i < plan_horizon - prev_size; i++) {

                        double N = target_dist / (dt * curr_vel / 2.24);
                        double x_point = x_add_on + target_x / N;
                        double y_point = target_spline(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        // Rotate back to normal after rotating it earlier
                        x_point = x_ref * cos(ref_state[2]) -
                                  y_ref * sin(ref_state[2]);
                        y_point = x_ref * sin(ref_state[2]) +
                                  y_ref * cos(ref_state[2]);

                        x_point += ref_state[0];
                        y_point += ref_state[1];

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }

                    // Send back the values:
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                } // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        } // end websocket if
    });   // end h.onMessage

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
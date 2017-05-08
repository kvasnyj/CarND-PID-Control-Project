#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <err.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

void move(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode, bool print, PID &pid)
{
    auto s = hasData(std::string(data).substr(0, length));
    if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());
            double steer_value;

            steer_value = pid.UpdateError(cte);

            // DEBUG
            if (print)
                std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            if (print) std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
    } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
}

void run(double p[], bool optimize) {
    PID pid;
    pid.Init(p[0], p[1], p[2]);

    uWS::Hub h;
    int i = 0, it=0, par=0;
    double dp[] = {.1, .01, 1.};
    int state = 0;
    double best_err = 0;

    h.onMessage(
            [&pid, &i, &dp, &state, &best_err, &it, &par, &optimize, &p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2') {
                    if (i >= 100 && optimize) {
                        i = 0;
                        it++;

                        double err = pid.TotalError();
                        double sum_dp = std::accumulate(std::begin(dp), std::end(dp), 0.0, std::plus<double>());

                        std::cout << "iteration: " << it << ", error: " << err << ", sum_dp: " << sum_dp << std::endl;
                        std::cout << "Kp = " << p[0] << ", Ki = " << p[1] << ", Kd = " << p[2] << std::endl;

                        //twiddle
                        if (sum_dp > 0.2) {
                            //std::cout << "state = " << state << ", par = " << par << std::endl;
                            switch (state) {
                                case 0: // init
                                    best_err = err;
                                    state = 1;
                                    p[par] += dp[par];
                                    pid.Init(p[0], p[1], p[2]);
                                    break;
                                case 1:
                                    if (err < best_err) {
                                        best_err = err;
                                        dp[par] *= 1.1;

                                        par = (par+1)%3;
                                        p[par] += dp[par];
                                        pid.Init(p[0], p[1], p[2]);
                                    } else {
                                        p[par] -= 2 * dp[par];
                                        pid.Init(p[0], p[1], p[2]);
                                        state = 2;
                                    }
                                    break;
                                case 2:
                                    if (err < best_err) {
                                        best_err = err;
                                        dp[par] *= 1.1;
                                    } else {
                                        p[par] += dp[par];
                                        dp[par] *= 0.9;
                                    }
                                    par = (par+1)%3;
                                    state = 1;
                                    p[par] += dp[par];
                                    pid.Init(p[0], p[1], p[2]);
                                    break;
                            }
                        }
                        else
                        {
                            optimize = false;
                        }
                    }

                    if (i++ == 0) {
                        std::string reset_msg = "42[\"reset\", {}]";
                        ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                    }

                    move(ws, data, length, opCode, !optimize, pid);
                }
            });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
    }

    h.run();
}


int main() {
    double p[] = {0, 0, 0};
    //double p[] = {0.2, 0.004, 3.0};
    run(p, true);

    return 0;
}

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

double run(PID &pid_, int n = 100) {
    PID pid = pid_;
    uWS::Hub h;
    int i = 0;

    h.onMessage([&pid, &i, &n, &h](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            if (i == 0) {
                std::string reset_msg = "42[\"reset\", {}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            }
            std::cout << i << std::endl;
            if (++i > n && n > 0) {
                // https://github.com/uWebSockets/uWebSockets/issues/366
                auto uv_async_callback = [](uv_async_t* handle) {
                    auto uv_walk_callback = [](uv_handle_t* handle, void* /*arg*/) {
                        if (!uv_is_closing(handle))
                            uv_close(handle, nullptr);
                    };

                    auto loop = handle->loop;
                    auto hub = (uWS::Hub *)handle->data;
                    //hub->getDefaultGroup<true>().close();

                    uv_stop(loop);
                    uv_walk(loop, uv_walk_callback, nullptr);
                    uv_run(loop, UV_RUN_DEFAULT);
                    uv_loop_close(loop);
                };

                auto loop = h.getLoop();

                uv_async_t asyncHandle;
                asyncHandle.data = &h;
                uv_async_init(loop, &asyncHandle, uv_async_callback);
                uv_async_send(&asyncHandle);

                return;
            }

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
                    if (n == 0) std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    if (n == 0) std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
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

    //h.getDefaultGroup<true>().close();

    return pid.TotalError();
}

void twiddle() {
    //constant
    int n = 10;
    double tol = 0.2;

    //variables
    int it = 0;
    double sum_dp = 3;
    int p[] = {0, 0, 0};
    int dp[] = {1, 1, 1};
    double err = 0;

    //init PID
    PID pid;
    pid.Init(p[0], p[1], p[2]);
    double best_err = run(pid, n);

    while (sum_dp > tol) {

        for (int i = 0; i < 3; i++) {
            p[i] += dp[i];
            pid.Init(p[0], p[1], p[2]);
            err = run(pid, n);

            if (err < best_err) {
                best_err = err;
                dp[i] *= 1.1;
            } else {
                p[i] -= 2 * dp[i];
                pid.Init(p[0], p[1], p[2]);
                err = run(pid, n);

                if (err < best_err) {
                    best_err = err;
                    dp[i] *= 1.1;
                } else {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
        }


        sum_dp = std::accumulate(std::begin(dp), std::end(dp), 0, std::plus<double>());
        std::cout << "iteration: " << it++ << " error: " << err << "sum_dp: " << sum_dp << std::endl;
        std::cout << "Kp = " + p[0] << ", Ki = " << p[1] << ", Kd = " << p[2] << std::endl;
    }

    std::cout << "Kp = " + p[0] << ", Ki = " << p[1] << ", Kd = " << p[2] << std::endl;
}

int main() {
    twiddle();

    return 0;

}

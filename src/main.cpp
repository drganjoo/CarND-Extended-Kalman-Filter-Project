#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

#ifndef _WIN32

#include <unistd.h>
#include <uWS/uWS.h>

#else

// disable warnings in uWS/uWS code
#pragma warning(push)
#pragma warning(disable:4251)
#pragma warning(disable:4800)
#pragma warning(disable:4996)

#include <uWS/uWS.h>

#pragma warning(pop)

#endif

#include <sstream>
#include "measurements.h"
#include "FusionEKF.h"
#include "tools.h"
#include "json.hpp"

using namespace std;
using namespace Eigen;
using json = nlohmann::json;

vector<VectorXd> estimations;
vector<VectorXd> ground_truth;

SensorFusion sf;
uWS::Hub h;

Radar ParseRadar(istream &tokens) {
    Radar r;

    // sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
    tokens >> r.rho >> r.phi >> r.rhodot >> r.timestamp;
    return r;
}

Laser ParseLaser(istream &tokens) {
    Laser l;

    // sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
    tokens >> l.x >> l.y >> l.timestamp;
    return l;
}

Eigen::Vector4d GetGroundTruth(istream &tokens) {
    double x;
    double y;
    double x_dot;
    double y_dot;
    //double rho;
    //double rho_dot;

    tokens >> x >> y >> x_dot >> y_dot; /*>> rho >> rho_dot;*/

    Vector4d gt_values;
    gt_values << x, y, x_dot, y_dot;

    return gt_values;
}

//
//void ReadFromFile()
//{
//	//ifstream is("../data/obj_pose-laser-radar-synthetic-input.txt");
//	string filename = "../EKF_Data/obj_pose-laser-radar-synthetic-input.txt";
//	ifstream is(filename);
//
//	if (is.is_open()) {
//
//		string line;
//		while (getline(is, line)) {
//			istringstream iss(line);
//
//			string type;
//			iss >> type;
//
//			if (type[0] == 'R') {
//				Radar r = ParseRadar(iss);
//				sf.ProcessMeasurement(r);
//			}
//			else if (type[0] == 'L') {
//				Laser l = ParseLaser(iss);
//				sf.ProcessMeasurement(l);
//			}
//
//			AddToGround(iss);
//
//			estimations.push_back(sf.GetState());
//			cout << "X:" << sf.GetState() << endl << "P:" << sf.GetStateCovariance() << endl;
//
//			VectorXd RMSE = Tools::CalculateRMSE(estimations, ground_truth);
//			cout << "RMSE: " << RMSE << endl;
//		}
//
//		is.close();
//	}
//	else {
//		char error[1024];
//#ifndef _WIN32
//		strerror_r(errno, error, sizeof(error));
//#else
//		strerror_s(error, sizeof(error), errno);
//#endif
//		cerr << "Could not open: " << filename << endl << "Error:" << error << endl;
//	}
//}


bool GetMeasurementLine(uWS::WebSocket<uWS::SERVER> &ws, char *data, size_t length, string *measurement_line) {
    bool found = false;

    // Message should have 42 in the beginning
    if (length > 2 && data[0] == '4' && data[1] == '2') {
        data[length] = 0;

        if (strstr(data, "null")) {
            // send response back to the simulator
            const char msg[] = "42[\"manual\",{}]";
            constexpr size_t length = sizeof(msg) - 1;
            ws.send(msg, length, uWS::OpCode::TEXT);
        } else {
            // look for json message inside [ and ]
            char *start_bracket = strchr(data, '[');
            if (start_bracket) {
                auto end_bracket = strrchr(data, ']');
                if (end_bracket) {
                    *(end_bracket + 1) = 0;

                    // convert  message into a JSon object, get the measurement line in case
                    // it is a telemetry message
                    auto j = json::parse(start_bracket);
                    auto wsEvent = j[0].get<std::string>();

                    if (wsEvent == "telemetry") {
                        *measurement_line = j[1]["sensor_measurement"];
                        found = true;
                    }
                }
                else {
                    // looks like a bad message, it doesn't have a ]
                    assert(0);
                }
            }
        }
    }

    return found;
}

void SendEstimates(uWS::WebSocket<uWS::SERVER> &ws, double px, double py, VectorXd &rmse) {
    json msgJson;
    msgJson["estimate_x"] = px;
    msgJson["estimate_y"] = py;
    msgJson["rmse_x"] = rmse(0);
    msgJson["rmse_y"] = rmse(1);
    msgJson["rmse_vx"] = rmse(2);
    msgJson["rmse_vy"] = rmse(3);

    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

template<class T>
void ComputeRMSEAndSend(uWS::WebSocket<uWS::SERVER> &ws, T &&ground) {
    ground_truth.push_back(ground);
    auto state = sf.GetState();
    estimations.push_back(state);

    VectorXd rmse = Tools::CalculateRMSE(estimations, ground_truth);
    SendEstimates(ws, state(0), state(1), rmse);
}

void ProcessMeasurement(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    string measurement;
    if (GetMeasurementLine(ws, data, length, &measurement)) {
        //cout << measurement << endl;
        istringstream iss(measurement);

        string type;
        iss >> type;

        if (type[0] == 'R') {
            Radar r = ParseRadar(iss);
            sf.ProcessMeasurement(r);
        } else if (type[0] == 'L') {
            Laser l = ParseLaser(iss);
            sf.ProcessMeasurement(l);
        }

        ComputeRMSEAndSend(ws, GetGroundTruth(iss));
    }
}

void Initialize(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    string measurement;

    // get a measurement line and call Initialize on SensorFusion for Radar / Laser
    // change function to be called to
    if (GetMeasurementLine(ws, data, length, &measurement)) {
        if (!measurement.empty()) {
            istringstream iss(measurement);

            string type;
            iss >> type;

            if (type[0] == 'R') {
                Radar r = ParseRadar(iss);
                sf.Initialize(r);
            } else if (type[0] == 'L') {
                Laser l = ParseLaser(iss);
                sf.Initialize(l);
            }

            if (sf.IsInitialized())
                h.onMessage(ProcessMeasurement);

            ComputeRMSEAndSend(ws, GetGroundTruth(iss));
        }
    }
}

void ReadFromSim() {
    h.onMessage(Initialize);

    h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen("127.0.0.1", port)) {
        cout << "Listening on: " << port << endl;
        h.run();
    } else
        cout << "Could not start listening";
}

int main() {
    //ReadFromFile();
    ReadFromSim();
}

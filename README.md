# Extended Kalman Filter Project

## Chanes from Original Source

1. Measurement has been broken down into two different structures **Radar** and **Laser**. This has been done to make it easier for separating code for initialization as well as updates

```
struct Measurement
{
    long long timestamp;
};

struct Radar : Measurement
{
    double rho;
    double phi;
    double rhodot;
};

struct Laser : Measurement
{
    double x;
    double y;
};

```
2. Hub Calls have been changed in main.cpp to accomodate for initialization

## Initialization

Hub is initially configured to call function **Initialize**, which calls SensorFusion::Initialize. Once initialization is done, hub is configured to call **ProcessMeasurement**

```
void Initialize(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    string measurement;

   ...
   ...

            if (sf.IsInitialized())
                h.onMessage(ProcessMeasurement);

            ComputeRMSEAndSend(ws, GetGroundTruth(iss));
        }
    }
}
```

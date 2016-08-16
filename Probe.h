#ifndef PROBE_H
#define PROBE_H

#include <string>
#include <thread>
#include <chrono>
#include <atomic>

#include <glm/gtc/quaternion.hpp>
#include "myahrs_plus.hpp"

class Probe
{
public:
#ifdef _WIN32
    Probe(const std::string & port_name="COM3",
          int baud_rate=115200);
#else
    Probe(const std::string & port_name="/dev/ttyACM0",
          int baud_rate=115200);
#endif

    void run();

    void stop();

    glm::quat get_orientation() const;

protected:
    const unsigned short polling_frequency = 100; // in times/second [Hz]
    const std::chrono::milliseconds polling_interval_duration {1000 / polling_frequency};

    void update(float ellapsed_time_in_millis);
    void update_loop();
    bool connect();

    glm::quat orientation;
    WithRobot::MyAhrsPlus sensor;
    WithRobot::SensorData sensor_data;

    // Internal timers
    std::chrono::time_point<std::chrono::high_resolution_clock> to_sleep_time, process_start_time;
    std::atomic<bool> should_terminate;
    std::thread update_thread;

    std::string port_name;
    int baud_rate;
    bool connected = false;
};

#endif // PROBE_H

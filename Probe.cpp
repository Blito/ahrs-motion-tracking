#include "Probe.h"

Probe::Probe(const std::string & port_name, int baud_rate) :
    orientation(1.0f,0.0f,0.0f,0.0f),
    sensor(),
    sensor_data(),
    should_terminate(false),
    port_name(port_name),
    baud_rate(baud_rate),
    connected(false)
{}

void Probe::run()
{
    if (connect())
    {
        std::cout << "Connected." << std::endl;

        should_terminate = false;

        // Spawn new thread with updateLoop
        update_thread = std::thread(&Probe::update_loop, this);
    }
}

void Probe::stop()
{
    if (connected && update_thread.joinable())
    {
        should_terminate = true;
        update_thread.join();
    }
}

bool Probe::connect()
{
    connected = true;

    // Start communication with the sensor
    if(!sensor.start(port_name, baud_rate)) {
        connected = false;
    }
    else
    {
        // Set binary output format. Select Quaternion and IMU data.
        if(!sensor.cmd_binary_data_format("QUATERNION, IMU")
           ||!sensor.cmd_divider("1")
           ||!sensor.cmd_mode("BC"))
        {
            connected = false;
        }
    }

    return connected;
}

void Probe::update(float ellapsed_time_in_millis)
{
    sensor.get_data(sensor_data);
    WithRobot::Quaternion& q = sensor_data.quaternion;
    orientation.w = q.w;
    orientation.x = q.x;
    orientation.y = q.y;
    orientation.z = q.z;
}

void Probe::update_loop()
{
    using namespace std::chrono;

    std::cout << "update_loop start" << std::endl;

    to_sleep_time = high_resolution_clock::now();
    int i = 0;
    while (connected && !should_terminate)
    {
        if (i == 100)
        {
            std::cout << orientation.w << " "
                      << orientation.x << " "
                      << orientation.y << " "
                      << orientation.z << std::endl;
            i = 0;
        }
        // Measure processing time
        process_start_time = high_resolution_clock::now();
        auto time_since_last_process = duration_cast<milliseconds>(to_sleep_time - process_start_time);

        // Process data
        update(time_since_last_process.count());

        // Put the thread to sleep for the remaining time within the interval
        to_sleep_time = high_resolution_clock::now();
        auto process_duration = duration_cast<milliseconds>(to_sleep_time - process_start_time);
        std::this_thread::sleep_for(polling_interval_duration - process_duration);
        i++;
    }

    std::cout << "update_loop end" << std::endl;
}

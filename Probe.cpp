#include "Probe.h"

Probe::Probe(const std::string & port_name, int baud_rate) :
    sensor(),
    sensor_data(),
    last_accel_read{},
    acceleration{},
    velocity{},
    position{},
    should_terminate{false},
    port_name{port_name},
    baud_rate{baud_rate},
    connected{false}
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
    float dt = ellapsed_time_in_millis / 1000.0f;

    sensor.get_data(sensor_data);
    WithRobot::Quaternion& sensor_orientation = sensor_data.quaternion;
    WithRobot::ImuData<float>& imu_data = sensor_data.imu;

    glm::vec3 accel_local(imu_data.ax, imu_data.ay, imu_data.az);
    glm::quat orientation(sensor_orientation.w, sensor_orientation.x, sensor_orientation.y, sensor_orientation.z);

    glm::vec3 accel_world = orientation * accel_local;

    glm::vec3 gravity_vector{0.0, 0.0, -1.0};

    accel_world -= gravity_vector;

    if (std::abs(accel_world.x) < sensitivity_threshold) { accel_world.x = 0.0f; }
    if (std::abs(accel_world.y) < sensitivity_threshold) { accel_world.y = 0.0f; }
    if (std::abs(accel_world.z) < sensitivity_threshold) { accel_world.z = 0.0f; }

    velocity += 9.81f * accel_world * dt;
    position += velocity * dt;

    std::cout << position.x << " " << position.y << " " << position.z << std::endl;
}

void Probe::update_loop()
{
    using namespace std::chrono;

    std::cout << "update_loop start" << std::endl;

    to_sleep_time = high_resolution_clock::now();
    while (connected && !should_terminate)
    {
        // Measure processing time
        process_start_time = high_resolution_clock::now();
        auto time_since_last_process = duration_cast<milliseconds>(process_start_time - to_sleep_time);

        // Process data
        update(time_since_last_process.count());

        // Put the thread to sleep for the remaining time within the interval
        to_sleep_time = high_resolution_clock::now();
        auto process_duration = duration_cast<milliseconds>(to_sleep_time - process_start_time);
        std::this_thread::sleep_for(polling_interval_duration - process_duration);
    }

    std::cout << "update_loop end" << std::endl;
}

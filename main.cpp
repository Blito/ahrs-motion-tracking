#include <iostream>
#include <chrono>

#include "Probe.h"

int main(int argc, char const *argv[])
{
    using namespace std::chrono;
    using namespace std::literals::chrono_literals;

    Probe probe;

    std::cout << "Starting probe..." << std::endl;

    probe.run();

    auto start = steady_clock::now();

    auto run_duration = 120s;

    while (steady_clock::now() - start < run_duration)
    {
        // ...
    }

    std::cout << "Stopping probe..." << std::endl;

    probe.stop();

    return 0;
}

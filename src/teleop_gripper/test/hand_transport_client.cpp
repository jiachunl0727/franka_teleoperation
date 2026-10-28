// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
// Robot-free fixture: connects exclusively to a loopback fake Hand server.
#include <franka/exception.h>
#include <franka/gripper.h>

#include <chrono>
#include <future>
#include <iostream>
#include <thread>

int main(int argc, char**) {
  using namespace std::chrono_literals;
  try {
    franka::Gripper hand("127.0.0.1");
    if (argc > 1) {
      try {
        hand.stop();
      } catch (const franka::Exception&) {
        return 0;  // Malformed/closed transport must fail rather than hang.
      }
      return 2;
    }
    for (int cycle = 0; cycle < 12; ++cycle) {
      auto motion = std::async(std::launch::async, [&] {
        try {
          if (cycle % 2)
            hand.move(.08, .01);
          else
            hand.grasp(0, .01, 5, .001, .007);
          return false;  // The server only completes motion by aborting it.
        } catch (const franka::CommandException&) {
          return true;
        }
      });
      std::this_thread::sleep_for(20ms);
      const auto started = std::chrono::steady_clock::now();
      if (!hand.stop() || !motion.get())
        return 3;
      std::cout << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                             started)
                       .count()
                << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}

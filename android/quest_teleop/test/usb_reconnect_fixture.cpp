#include <chrono>
#include <thread>

#include "usb_transport.h"

int main() {
  quest_usb::UsbTransport transport;
  // Deliberately produce no OpenXR/controller samples. The transport must
  // establish and re-establish its clock handshake independently of tracking.
  std::this_thread::sleep_for(std::chrono::seconds(5));
  return transport.connected() && transport.clock_snapshot(quest_usb::monotonic_time_ns()).valid &&
                 transport.take_disarm_request()
             ? 0
             : 1;
}

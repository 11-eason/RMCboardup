#ifndef GIMBAL_CAN_HPP
#define GIMBAL_CAN_HPP

namespace gimbal_can {
// Shared transport only. Calling this never selects or starts a motor program.
// Idempotent; false means another task is initializing, or initialization failed.
bool start();
}

#endif

// Motorctl/include/Motorctl.h
#ifndef MOTORCTL_H_
#define MOTORCTL_H_

namespace motor_ctl {

struct Comms {
    //static inline constexpr const char* PORT_F = "/dev/ttyUSB0";
    //static inline constexpr const char* PORT_R = "/dev/ttyUSB1";
    static inline constexpr int BAUDRATE = 115200;
    static inline constexpr int MOTOR_ID = 0x01;
};

struct Kinematics {
    static inline constexpr double PI      = 3.14159265358979323846;
    static inline constexpr double RAD2REV = 1.0 / (2.0 * PI);
    static inline constexpr double DEG2RAD = PI / 180.0;
    static inline constexpr double W       = 0.45;
    static inline constexpr double r       = 0.17;
    static inline constexpr double to_RPM  = (60.0 / r) * RAD2REV;
};

struct Control {
    static inline constexpr double Kv1       = 2.003;
    static inline constexpr double Kv0       = -0.01504;
    static inline constexpr double Ky1       = 4.285;
    static inline constexpr double Ky0       = -0.2784;
    static inline constexpr int RPM_FWD   = 60;
    static inline constexpr int RPM_REV   = -60;
    static inline constexpr int RPM_TFWD  = 40;
    static inline constexpr int RPM_TREV  = -40;
};

} // namespace motor_ctl
#endif

#ifndef IMU_ATTITUDE_HPP
#define IMU_ATTITUDE_HPP

#include <cmath>

// Body-to-world quaternion, ENU-like axes with gravity measured along +Z.
// No encoder or trajectory inputs are used by this estimator.
class ImuAttitude {
public:
  bool begin(const float a[3]) {
    const float n = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (!std::isfinite(n) || n < 1.0f) return false;
    const float roll = std::atan2(a[1], a[2]);
    const float pitch = std::atan2(-a[0], std::sqrt(a[1]*a[1] + a[2]*a[2]));
    const float cr = std::cos(roll*0.5f), sr = std::sin(roll*0.5f);
    const float cp = std::cos(pitch*0.5f), sp = std::sin(pitch*0.5f);
    q_[0]=cr*cp; q_[1]=sr*cp; q_[2]=cr*sp; q_[3]=-sr*sp;
    previous_yaw_ = 0.0f;
    yaw = 0.0f;
    return angles(false);
  }

  bool update(const float a[3], const float w[3], float dt, float kp) {
    if (!std::isfinite(dt) || dt <= 0.0f || dt > 0.01f) return false;
    float wx=w[0], wy=w[1], wz=w[2];
    const float n = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    // Strong linear acceleration is not a reliable gravity observation.
    if (std::isfinite(n) && std::fabs(n-9.80665f) < 1.5f) {
      const float ax=a[0]/n, ay=a[1]/n, az=a[2]/n;
      const float vx=2.0f*(q_[1]*q_[3]-q_[0]*q_[2]);
      const float vy=2.0f*(q_[0]*q_[1]+q_[2]*q_[3]);
      const float vz=q_[0]*q_[0]-q_[1]*q_[1]-q_[2]*q_[2]+q_[3]*q_[3];
      wx += kp*(ay*vz-az*vy);
      wy += kp*(az*vx-ax*vz);
      wz += kp*(ax*vy-ay*vx);
    }
    wx*=0.5f*dt; wy*=0.5f*dt; wz*=0.5f*dt;
    const float q0=q_[0]-q_[1]*wx-q_[2]*wy-q_[3]*wz;
    const float q1=q_[1]+q_[0]*wx+q_[2]*wz-q_[3]*wy;
    const float q2=q_[2]+q_[0]*wy-q_[1]*wz+q_[3]*wx;
    const float q3=q_[3]+q_[0]*wz+q_[1]*wy-q_[2]*wx;
    const float norm=std::sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
    if (!std::isfinite(norm) || norm < 0.1f) return false;
    q_[0]=q0/norm; q_[1]=q1/norm; q_[2]=q2/norm; q_[3]=q3/norm;
    return angles(true);
  }

  float yaw = 0.0f;    // Unwrapped; never jumps at +/-pi.
  float pitch = 0.0f;
private:
  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float previous_yaw_ = 0.0f;
  bool angles(bool accumulate) {
    constexpr float pi=3.14159265359f;
    const float wrapped=std::atan2(2.0f*(q_[0]*q_[3]+q_[1]*q_[2]),
      1.0f-2.0f*(q_[2]*q_[2]+q_[3]*q_[3]));
    float sine=2.0f*(q_[0]*q_[2]-q_[1]*q_[3]);
    if (sine > 1.0f) sine=1.0f;
    if (sine < -1.0f) sine=-1.0f;
    pitch=std::asin(sine);
    float delta=wrapped-previous_yaw_;
    if (delta > pi) delta-=2.0f*pi;
    if (delta < -pi) delta+=2.0f*pi;
    if (accumulate) yaw+=delta;
    previous_yaw_=wrapped;
    return std::isfinite(yaw) && std::isfinite(pitch);
  }
};

#endif

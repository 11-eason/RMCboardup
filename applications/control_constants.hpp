#ifndef CONTROL_CONSTANTS_HPP
#define CONTROL_CONSTANTS_HPP

namespace control_constants
{
// Initial tuning values only; NOT calibrated for this chassis.
// Torque: gearbox output shaft N*m; speed: same shaft rad/s.
// Refit K1/K2 if the shaft convention or actual gear ratio changes.
constexpr float POWER_K1 = 1.98f;   // W / (N*m)^2
constexpr float POWER_K2 = 0.009f;  // W / (rad/s)^2
constexpr float POWER_K3 = 4.0f;    // W, TOTAL chassis idle loss; measure before calibration.
constexpr bool POWER_MODEL_CALIBRATED = false;

constexpr float CHASSIS_POWER_LIMIT_W = 80.0f;
// Optional reserved margin; effective limit = limit - margin.
constexpr float POWER_LIMIT_MARGIN_W = 3.0f;
constexpr unsigned POWER_LIMIT_SEARCH_STEPS = 24;
static_assert(CHASSIS_POWER_LIMIT_W > 0.0f, "Power limit must be positive");
static_assert(
  POWER_LIMIT_MARGIN_W >= 0.0f && POWER_LIMIT_MARGIN_W < CHASSIS_POWER_LIMIT_W,
  "Invalid power margin");
static_assert(POWER_LIMIT_SEARCH_STEPS > 0, "Search needs at least one step");

// =====================================================================
// ===== 云台轨迹跟随 =====
// 结构：位置环 P(+可选 I/D，速度前馈) → 速度限幅/斜坡 → 速度环 PI → 力矩限幅
//       前馈两级：速度前馈（参考角速度）、力矩前馈（J·α + 重力）
// 单位：角度 rad、角速度 rad/s、角加速度 rad/s^2、力矩 N·m、惯量 kg·m^2
// =====================================================================

// ---- 电机与装配 ----
constexpr unsigned FOLLOW_YAW_MOTOR_ID = 1U;    // CAN1，GM6020 电流模式
constexpr unsigned FOLLOW_PITCH_MOTOR_ID = 2U;
constexpr float FOLLOW_YAW_RATIO = 1.0f;        // 直接驱动
constexpr float FOLLOW_PITCH_RATIO = 1.0f;
// 关节角随编码器角增长。首次上电必须低速（时间缩放 4 以上）确认符号，
// 若电机反向追、误差越追越大，把对应方向改成 -1。
constexpr float FOLLOW_YAW_DIRECTION = 1.0f;
constexpr float FOLLOW_PITCH_DIRECTION = 1.0f;

// ---- 运行模式 ----
// 台架上只接了一台电机时设为 true（只跟随 yaw 轴）。
constexpr bool FOLLOW_SINGLE_YAW = false;
// 恒力矩台架测试：不做任何闭环，只输出固定力矩。
// 用于测静摩擦（逐级加力矩看何时连续转动）和确认转向，测完必须改回 false。
constexpr bool FOLLOW_CONSTANT_TORQUE_TEST = false;
constexpr float FOLLOW_TEST_TORQUE_NM = 0.12f;
constexpr float FOLLOW_TEST_TRIP_SPEED_RAD_S = 5.23598776f;  // 50 rpm，超速即锁存停机
// 机械行程保护：裸电机阶段关闭，装上云台后打开并填真实行程。
constexpr bool FOLLOW_ENFORCE_ANGLE_LIMITS = false;
// 时间缩放：4 = 轨迹放慢 4 倍（2720 ms 一个周期），裸电机调试从这里开始；
// 稳定后依次改 2、1。缩放会同时按比例降低速度/加速度。
constexpr unsigned FOLLOW_TRAJECTORY_TIME_SCALE = 4U;
constexpr bool FOLLOW_START_ENABLED = true;              // 上电即开始跟随（台架用）
constexpr bool FOLLOW_AUTO_CAPTURE_INITIAL_POSE = true;  // 上电自动记录初始姿态
constexpr bool FOLLOW_USE_RC_STOP = false;               // 用遥控器右拨杆下档急停

// ---- 时序 ----
constexpr unsigned FOLLOW_PERIOD_MS = 1U;           // 控制周期 1 ms
// 电流帧发送周期。这是整个方案的关键参数：
// 电机只在收到新电流帧时更新力矩，所以它等于"执行器的采样周期"。
// 闭环时间常数 τ 必须 ≥ 5~10 倍这个值，否则会自激振荡（嗡嗡响）。
// 总线负载考虑取 2 ms（500 Hz）；若总线余量足够可分到 1 ms。
constexpr unsigned FOLLOW_CAN_TX_INTERVAL_MS = 2U;
constexpr unsigned FOLLOW_COMMAND_TIMEOUT_MS = 20U;
constexpr unsigned FOLLOW_FEEDBACK_TIMEOUT_MS = 100U;
constexpr unsigned FOLLOW_MAX_STEP_MS = 10U;        // 调度间隔超过它就重新初始化 PID

// ---- 安全判据 ----
constexpr float FOLLOW_MAX_TRACKING_ERROR_RAD = 0.17453293f;  // 10°，超过即判跟随失败
constexpr unsigned FOLLOW_TRACKING_ERROR_MS = 50U;            // 连续超限 50 ms 才停机
constexpr float FOLLOW_OVERSPEED_FACTOR = 1.5f;               // 实际转速超上限 1.5 倍即停机

// ---- 前馈（装云台后再填，裸电机阶段保持 0）----
// 力矩前馈 = J·α_ref + K_g·cos(pitch)。裸电机 J 很小、无重力，所以两者都为 0。
// 填法见 docs/gimbal_measurement.md。
constexpr float FOLLOW_YAW_ACCEL_FF_J = 0.0f;       // kg·m^2
constexpr float FOLLOW_PITCH_ACCEL_FF_J = 0.0f;
constexpr float FOLLOW_PITCH_GRAVITY_NM = 0.0f;     // m·g·r，0 = 关闭

// ---- 增益与限幅 ----
// 增益量级的来历（已验证）：
//   速度环 Kp ≈ 0.015 N·m/(rad/s) = 0.02 A/(rad/s)。再高就会自激：
//   闭环时间常数 τ = J/(Kt·Kp)，J≈1.3e-4、Kt=0.741、Kp=0.04 时 τ≈4.4 ms，
//   而电流帧周期 5 ms 时 τ < 周期 → 采样环必然振荡（这就是之前"震动"的根因）。
//   Ki 必须能让积分爬到超过静摩擦（≈0.07 N·m），所以积分上限不能低于 0.2 N·m。
constexpr float FOLLOW_YAW_ANGLE_KP = 9.0f;    // (rad/s) / rad
constexpr float FOLLOW_YAW_ANGLE_KI = 0.0f;
constexpr float FOLLOW_YAW_ANGLE_KD = 0.0f;
constexpr float FOLLOW_YAW_ANGLE_INTEGRAL_LIMIT = 0.0f;   // rad/s
constexpr float FOLLOW_YAW_SPEED_KP = 0.015f;  // N·m / (rad/s)
constexpr float FOLLOW_YAW_SPEED_KI = 0.45f;   // N·m / rad
constexpr float FOLLOW_YAW_SPEED_KD = 0.0f;
constexpr float FOLLOW_YAW_SPEED_INTEGRAL_LIMIT = 0.25f;  // N·m
constexpr float FOLLOW_YAW_ANGLE_MIN_RAD = 0.0f;
constexpr float FOLLOW_YAW_ANGLE_MAX_RAD = 0.0f;
constexpr float FOLLOW_YAW_MAX_SPEED_RAD_S = 4.0f;        // 时间缩放 4 时峰值 1.79
constexpr float FOLLOW_YAW_MAX_ACCEL_RAD_S2 = 15.0f;      // 时间缩放 4 时峰值 6.7
constexpr float FOLLOW_YAW_MAX_TORQUE_NM = 0.40f;         // ≈0.54 A，约 5 倍静摩擦
constexpr float FOLLOW_YAW_VELOCITY_FF = 1.0f;
constexpr bool FOLLOW_YAW_CONFIGURED = true;

constexpr float FOLLOW_PITCH_ANGLE_KP = 9.0f;
constexpr float FOLLOW_PITCH_ANGLE_KI = 0.0f;
constexpr float FOLLOW_PITCH_ANGLE_KD = 0.0f;
constexpr float FOLLOW_PITCH_ANGLE_INTEGRAL_LIMIT = 0.0f;
constexpr float FOLLOW_PITCH_SPEED_KP = 0.015f;
constexpr float FOLLOW_PITCH_SPEED_KI = 0.45f;
constexpr float FOLLOW_PITCH_SPEED_KD = 0.0f;
constexpr float FOLLOW_PITCH_SPEED_INTEGRAL_LIMIT = 0.25f;
constexpr float FOLLOW_PITCH_ANGLE_MIN_RAD = 0.0f;
constexpr float FOLLOW_PITCH_ANGLE_MAX_RAD = 0.0f;
constexpr float FOLLOW_PITCH_MAX_SPEED_RAD_S = 4.0f;
constexpr float FOLLOW_PITCH_MAX_ACCEL_RAD_S2 = 15.0f;
constexpr float FOLLOW_PITCH_MAX_TORQUE_NM = 0.40f;
constexpr float FOLLOW_PITCH_VELOCITY_FF = 1.0f;
constexpr bool FOLLOW_PITCH_CONFIGURED = true;

// ---- 绘图 ----
// 0 = 底盘功率（2 通道），1 = 跟随诊断（14 通道），2 = 原始参考（2 通道）。
// 跟随诊断的通道顺序见 plot_task.cpp。
constexpr unsigned FOLLOW_PLOT_MODE = 1U;

static_assert(
  FOLLOW_YAW_MOTOR_ID == 1U && FOLLOW_PITCH_MOTOR_ID == 2U,
  "This CAN1 packing reserves GM6020 current-mode slots 1 and 2");
static_assert(
  FOLLOW_CAN_TX_INTERVAL_MS > 0U && FOLLOW_COMMAND_TIMEOUT_MS > FOLLOW_CAN_TX_INTERVAL_MS,
  "Invalid CAN timing");
static_assert(
  FOLLOW_PERIOD_MS > 0U && FOLLOW_TRACKING_ERROR_MS > 0U,
  "Invalid control timing");
static_assert(
  (FOLLOW_YAW_DIRECTION == 1.0f || FOLLOW_YAW_DIRECTION == -1.0f) &&
    (FOLLOW_PITCH_DIRECTION == 1.0f || FOLLOW_PITCH_DIRECTION == -1.0f),
  "Joint directions must be +1 or -1");
static_assert(
  FOLLOW_TRAJECTORY_TIME_SCALE > 0U && FOLLOW_TRAJECTORY_TIME_SCALE <= 100U,
  "Invalid trajectory time scale");
static_assert(FOLLOW_PLOT_MODE <= 2U, "Invalid plot mode");
static_assert(
  FOLLOW_MAX_TRACKING_ERROR_RAD > 0.0f && FOLLOW_OVERSPEED_FACTOR > 1.0f,
  "Protection thresholds must be positive and above 1");

struct FollowPidConfig
{
  float kp;
  float ki;
  float kd;
  float integral_limit;  // 积分项输出的绝对值上限（角度环单位 rad/s，速度环单位 N·m）
};

struct FollowAxisConfig
{
  FollowPidConfig angle_pid;   // 输入关节角 rad，输出目标转速 rad/s
  FollowPidConfig speed_pid;   // 输入转速 rad/s，输出力矩 N·m
  float angle_min_rad;         // 机械行程，不是轨迹范围
  float angle_max_rad;
  float max_speed_rad_s;
  float max_accel_rad_s2;      // 目标转速斜坡，必须 ≥ 轨迹峰值加速度
  float max_output;            // 对称力矩限幅 N·m
  float velocity_feedforward_gain;
  float accel_feedforward_j;   // 加速度前馈的转动惯量 kg·m^2，0 = 关闭
  float gravity_coefficient_nm;  // 重力矩系数 m·g·r，0 = 关闭；仅 pitch 轴使用
  bool configured;
};

constexpr FollowAxisConfig FOLLOW_YAW = {
  {FOLLOW_YAW_ANGLE_KP, FOLLOW_YAW_ANGLE_KI, FOLLOW_YAW_ANGLE_KD,
   FOLLOW_YAW_ANGLE_INTEGRAL_LIMIT},
  {FOLLOW_YAW_SPEED_KP, FOLLOW_YAW_SPEED_KI, FOLLOW_YAW_SPEED_KD,
   FOLLOW_YAW_SPEED_INTEGRAL_LIMIT},
  FOLLOW_YAW_ANGLE_MIN_RAD,
  FOLLOW_YAW_ANGLE_MAX_RAD,
  FOLLOW_YAW_MAX_SPEED_RAD_S,
  FOLLOW_YAW_MAX_ACCEL_RAD_S2,
  FOLLOW_YAW_MAX_TORQUE_NM,
  FOLLOW_YAW_VELOCITY_FF,
  FOLLOW_YAW_ACCEL_FF_J,
  0.0f,
  FOLLOW_YAW_CONFIGURED};

constexpr FollowAxisConfig FOLLOW_PITCH = {
  {FOLLOW_PITCH_ANGLE_KP, FOLLOW_PITCH_ANGLE_KI, FOLLOW_PITCH_ANGLE_KD,
   FOLLOW_PITCH_ANGLE_INTEGRAL_LIMIT},
  {FOLLOW_PITCH_SPEED_KP, FOLLOW_PITCH_SPEED_KI, FOLLOW_PITCH_SPEED_KD,
   FOLLOW_PITCH_SPEED_INTEGRAL_LIMIT},
  FOLLOW_PITCH_ANGLE_MIN_RAD,
  FOLLOW_PITCH_ANGLE_MAX_RAD,
  FOLLOW_PITCH_MAX_SPEED_RAD_S,
  FOLLOW_PITCH_MAX_ACCEL_RAD_S2,
  FOLLOW_PITCH_MAX_TORQUE_NM,
  FOLLOW_PITCH_VELOCITY_FF,
  FOLLOW_PITCH_ACCEL_FF_J,
  FOLLOW_PITCH_GRAVITY_NM,
  FOLLOW_PITCH_CONFIGURED};

// 微分滤波时间常数（当前 Kd = 0，保留以便以后启用）
constexpr float FOLLOW_D_FILTER_S = 0.05f;
static_assert(
  FOLLOW_D_FILTER_S >= 0.0f && FOLLOW_D_FILTER_S <= 1.0f,
  "Invalid derivative filter time constant");

}  // namespace control_constants

#endif  // CONTROL_CONSTANTS_HPP

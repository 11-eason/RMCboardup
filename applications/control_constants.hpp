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
static_assert(CHASSIS_POWER_LIMIT_W > 0.0f, "Power limit must be positive");
static_assert(
  POWER_LIMIT_MARGIN_W >= 0.0f && POWER_LIMIT_MARGIN_W < CHASSIS_POWER_LIMIT_W,
  "Invalid power margin");
constexpr unsigned POWER_LIMIT_SEARCH_STEPS = 24;
static_assert(POWER_LIMIT_SEARCH_STEPS > 0, "Search needs at least one step");

// =====================================================================
// ===== 云台轨迹跟随 =====
// 结构：位置环 P(+可选 I/D，速度前馈) → 速度限幅/斜坡 → 速度环 PI → 力矩限幅
//       力矩前馈：加速度 J·α + 重力 K_g·cosθ + 摩擦 τ_c·sgn(ω)
// 单位：角度 rad、角速度 rad/s、角加速度 rad/s^2、力矩 N·m、惯量 kg·m^2
// =====================================================================

// ---- 电机与装配 ----
// GM6020 的 ID 由电机上的拨码开关决定：1/2/3 位是 ID 的二进制位，第 4 位是 120 Ω 终端电阻。
//   拨成 ID=1：1 位 ON，2、3 位 OFF
//   拨成 ID=2：2 位 ON，1、3 位 OFF
//   拨成 ID=4：3 位 ON，1、2 位 OFF
// 当前硬件是 ID 2 和 4，所以这里按 2/4 配置。若要把两台电机统一改成 1 和 2：
// 改拨码后，把下面两个数字改成 1 和 2 重烧。
// CAN 滤波器是"全通"（掩码全 0），任何 ID 都能收到，不需要改滤波配置。
constexpr unsigned FOLLOW_YAW_MOTOR_ID = 1U;
constexpr unsigned FOLLOW_PITCH_MOTOR_ID = 2U;
constexpr float FOLLOW_YAW_RATIO = 1.0f;
constexpr float FOLLOW_PITCH_RATIO = 1.0f;
// 关节角随编码器角增长。若某轴反向追、误差越追越大，把对应方向改成 -1。
constexpr float FOLLOW_YAW_DIRECTION = 1.0f;
constexpr float FOLLOW_PITCH_DIRECTION = 1.0f;

// ---- 运行模式 ----
constexpr bool FOLLOW_SINGLE_YAW = false;  // 两轴同时跟随；只接一台电机时改 true
// 恒力矩台架测试：不做闭环，只输出固定力矩（测静摩擦、确认转向）。测完必须改回 false。
constexpr bool FOLLOW_CONSTANT_TORQUE_TEST = false;
constexpr float FOLLOW_TEST_TORQUE_NM = 0.12f;
constexpr float FOLLOW_TEST_TRIP_SPEED_RAD_S = 5.23598776f;  // 50 rpm，超速即锁存停机
// 机械行程保护：裸电机阶段关闭；装上云台后打开并填真实行程。
constexpr bool FOLLOW_ENFORCE_ANGLE_LIMITS = false;
// 时间缩放：1 = 原始 680 ms 周期（最快）；2/4 = 放慢并同时降低速度、加速度。
constexpr unsigned FOLLOW_TRAJECTORY_TIME_SCALE = 1U;
constexpr bool FOLLOW_START_ENABLED = true;              // 上电即开始跟随（台架用）
constexpr bool FOLLOW_AUTO_CAPTURE_INITIAL_POSE = true;  // 上电自动记录初始姿态
constexpr bool FOLLOW_USE_RC_STOP = false;               // 用遥控器右拨杆下档急停

// ---- 时序 ----
constexpr unsigned FOLLOW_PERIOD_MS = 1U;  // 控制周期 1 ms
// 电流帧发送周期 = 执行器的采样周期。电机只在收到新帧时更新力矩，
// 所以闭环时间常数 τ 必须 ≥ 5~10 倍这个值，否则采样环自激（嗡嗡响）。
// 取 2 ms（500 Hz）是总线负载与带宽的折中；总线余量足够时可试 1 ms。
constexpr unsigned FOLLOW_CAN_TX_INTERVAL_MS = 2U;
constexpr unsigned FOLLOW_COMMAND_TIMEOUT_MS = 20U;
constexpr unsigned FOLLOW_FEEDBACK_TIMEOUT_MS = 100U;
constexpr unsigned FOLLOW_MAX_STEP_MS = 10U;  // 调度间隔超过它就重新初始化 PID

// ---- 安全判据 ----
constexpr float FOLLOW_MAX_TRACKING_ERROR_RAD = 0.34906585f;  // 20°
constexpr unsigned FOLLOW_TRACKING_ERROR_MS = 50U;            // 连续超限 50 ms 即停机
constexpr float FOLLOW_OVERSPEED_FACTOR = 1.5f;

// ---- 前馈 ----
// 力矩前馈 = J·α_ref + K_g·cos(pitch) + τ_c·sgn(ω)
// 裸电机 J 很小(≈1.3e-4)、无重力，所以前两项为 0；摩擦项是裸电机跟踪的关键。
constexpr float FOLLOW_YAW_ACCEL_FF_J = 0.0f;  // kg·m^2
constexpr float FOLLOW_PITCH_ACCEL_FF_J = 0.0f;
constexpr float FOLLOW_PITCH_GRAVITY_NM = 0.0f;  // m·g·r
// 库仑摩擦前馈（N·m）。裸电机实测启动摩擦约 0.1 A ≈ 0.074 N·m；
// 取 0.05 既能把低速"粘住"补掉，又不会在停顿反向点造成震动（实测最优）。
constexpr float FOLLOW_YAW_FRICTION_NM = 0.05f;
constexpr float FOLLOW_PITCH_FRICTION_NM = 0.05f;
// 平滑符号函数的过渡带（rad/s）：信号小于它时前馈按比例减小，避免过零抖动
constexpr float FOLLOW_FRICTION_BLEND_RAD_S = 0.02f;
// 摩擦前馈方向信号的低通时间常数（s）。轨迹每周期有 6 个"停住再反向"的点，
// 速度指令会在零点逐拍抖动；直接取符号会让力矩每拍翻转 ±τ_c → 震动。
// 先低通再取符号，前馈变成缓变的 ±τ_c，震动消失而补偿能力基本保留。设 0 = 不滤波。
constexpr float FOLLOW_FRICTION_FILTER_S = 0.03f;
// 静摩擦脱困（带滞环的辅助力矩）：误差 > ON 且几乎不动 → 沿误差方向补 τ_c；
// 误差 < OFF 或已经动起来 → 撤销。设为 0 即关闭（当前关闭，改用摩擦前馈）。
constexpr float FOLLOW_BREAKAWAY_ON_RAD = 0.0f;
constexpr float FOLLOW_BREAKAWAY_OFF_RAD = 0.010f;
constexpr float FOLLOW_BREAKAWAY_SPEED_RAD_S = 0.15f;

// ---- 增益与限幅 ----
// 增益量级的来历（已验证）：
//   速度环 Kp ≈ 0.015 N·m/(rad/s) = 0.02 A/(rad/s)。再高会自激：
//   闭环时间常数 τ = J/(Kt·Kp)，J≈1.3e-4、Kt=0.741、Kp=0.04 时 τ≈4.4 ms，
//   而电流帧周期 5 ms 时 τ < 周期 → 采样环必然振荡（之前"震动"的根因）。
//   积分上限必须 ≥ 静摩擦对应的力矩，否则积分顶不过摩擦。
//   位置环 Kp：12 → 15 用于压缩高速段的"切角"滞后。
constexpr float FOLLOW_YAW_ANGLE_KP = 12.0f;  // (rad/s)/rad（15 会变差，已回退到 12）
constexpr float FOLLOW_YAW_ANGLE_KI = 0.4f;
constexpr float FOLLOW_YAW_ANGLE_KD = 0.0f;
constexpr float FOLLOW_YAW_ANGLE_INTEGRAL_LIMIT = 0.0f;  // rad/s
constexpr float FOLLOW_YAW_SPEED_KP = 0.015f;            // N·m/(rad/s)
constexpr float FOLLOW_YAW_SPEED_KI = 0.45f;             // N·m/rad
constexpr float FOLLOW_YAW_SPEED_KD = 0.0f;
constexpr float FOLLOW_YAW_SPEED_INTEGRAL_LIMIT = 0.25f;  // N·m
constexpr float FOLLOW_YAW_ANGLE_MIN_RAD = 0.0f;
constexpr float FOLLOW_YAW_ANGLE_MAX_RAD = 0.0f;
constexpr float FOLLOW_YAW_MAX_SPEED_RAD_S = 10.0f;    // 原始周期峰值 7.15
constexpr float FOLLOW_YAW_MAX_ACCEL_RAD_S2 = 130.0f;  // 原始周期峰值 106.9
constexpr float FOLLOW_YAW_MAX_TORQUE_NM = 0.40f;      // ≈0.54 A
constexpr float FOLLOW_YAW_VELOCITY_FF = 1.0f;
constexpr bool FOLLOW_YAW_CONFIGURED = true;

constexpr float FOLLOW_PITCH_ANGLE_KP = 12.15f;  // 与 yaw 同参数（已验证）
constexpr float FOLLOW_PITCH_ANGLE_KI = 0.0f;
constexpr float FOLLOW_PITCH_ANGLE_KD = 0.1f;
constexpr float FOLLOW_PITCH_ANGLE_INTEGRAL_LIMIT = 0.0f;
constexpr float FOLLOW_PITCH_SPEED_KP = 0.015f;
constexpr float FOLLOW_PITCH_SPEED_KI = 0.45f;
constexpr float FOLLOW_PITCH_SPEED_KD = 0.0f;
constexpr float FOLLOW_PITCH_SPEED_INTEGRAL_LIMIT = 0.25f;
constexpr float FOLLOW_PITCH_ANGLE_MIN_RAD = 0.0f;
constexpr float FOLLOW_PITCH_ANGLE_MAX_RAD = 0.0f;
constexpr float FOLLOW_PITCH_MAX_SPEED_RAD_S = 10.0f;    // 原始周期峰值 8.31
constexpr float FOLLOW_PITCH_MAX_ACCEL_RAD_S2 = 130.0f;  // 原始周期峰值 69.0
constexpr float FOLLOW_PITCH_MAX_TORQUE_NM = 0.40f;
constexpr float FOLLOW_PITCH_VELOCITY_FF = 1.0f;
constexpr bool FOLLOW_PITCH_CONFIGURED = true;

// 微分滤波时间常数（当前 Kd = 0，保留以便以后启用）
constexpr float FOLLOW_D_FILTER_S = 0.05f;

// ---- 绘图 ----
// 0 = 底盘功率，1 = 跟随曲线（4 通道：两轴的目标/实际），2 = 原始参考
constexpr unsigned FOLLOW_PLOT_MODE = 1U;

static_assert(
  FOLLOW_YAW_MOTOR_ID >= 1U && FOLLOW_YAW_MOTOR_ID <= 4U && FOLLOW_PITCH_MOTOR_ID >= 1U &&
    FOLLOW_PITCH_MOTOR_ID <= 4U && FOLLOW_YAW_MOTOR_ID != FOLLOW_PITCH_MOTOR_ID,
  "GM6020 current mode: IDs 1-4 share frame 0x1FE and must be different");
static_assert(
  FOLLOW_CAN_TX_INTERVAL_MS > 0U && FOLLOW_COMMAND_TIMEOUT_MS > FOLLOW_CAN_TX_INTERVAL_MS,
  "Invalid CAN timing");
static_assert(FOLLOW_PERIOD_MS > 0U && FOLLOW_TRACKING_ERROR_MS > 0U, "Invalid control timing");
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
static_assert(
  FOLLOW_D_FILTER_S >= 0.0f && FOLLOW_D_FILTER_S <= 1.0f,
  "Invalid derivative filter time constant");

struct FollowPidConfig
{
  float kp;
  float ki;
  float kd;
  float integral_limit;  // 积分项输出的绝对值上限（角度环 rad/s，速度环 N·m）
};

struct FollowAxisConfig
{
  FollowPidConfig angle_pid;  // 输入关节角 rad，输出目标转速 rad/s
  FollowPidConfig speed_pid;  // 输入转速 rad/s，输出力矩 N·m
  float angle_min_rad;        // 机械行程（不是轨迹范围）
  float angle_max_rad;
  float max_speed_rad_s;
  float max_accel_rad_s2;  // 目标转速斜坡，必须 ≥ 轨迹峰值加速度
  float max_output;        // 对称力矩限幅 N·m
  float velocity_feedforward_gain;
  float accel_feedforward_j;     // 加速度前馈惯量 kg·m^2，0 = 关闭
  float gravity_coefficient_nm;  // 重力矩系数 m·g·r，0 = 关闭
  float friction_nm;             // 库仑摩擦前馈 N·m，0 = 关闭
  bool configured;
};

constexpr FollowAxisConfig FOLLOW_YAW = {
  {FOLLOW_YAW_ANGLE_KP, FOLLOW_YAW_ANGLE_KI, FOLLOW_YAW_ANGLE_KD, FOLLOW_YAW_ANGLE_INTEGRAL_LIMIT},
  {FOLLOW_YAW_SPEED_KP, FOLLOW_YAW_SPEED_KI, FOLLOW_YAW_SPEED_KD, FOLLOW_YAW_SPEED_INTEGRAL_LIMIT},
  FOLLOW_YAW_ANGLE_MIN_RAD,
  FOLLOW_YAW_ANGLE_MAX_RAD,
  FOLLOW_YAW_MAX_SPEED_RAD_S,
  FOLLOW_YAW_MAX_ACCEL_RAD_S2,
  FOLLOW_YAW_MAX_TORQUE_NM,
  FOLLOW_YAW_VELOCITY_FF,
  FOLLOW_YAW_ACCEL_FF_J,
  0.0f,
  FOLLOW_YAW_FRICTION_NM,
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
  FOLLOW_PITCH_FRICTION_NM,
  FOLLOW_PITCH_CONFIGURED};

}  // namespace control_constants

#endif  // CONTROL_CONSTANTS_HPP

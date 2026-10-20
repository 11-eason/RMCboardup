#ifndef CONTROL_CONSTANTS_HPP
#define CONTROL_CONSTANTS_HPP

namespace control_constants
{
// GY-56 receive test: physical 3-pin UART1 = USART6.
constexpr bool INFRARED_PLOT_TEST = false;
constexpr bool INFRARED_PLOT_RAW_FIELDS = true;   // 8 channels; false restores 4.
constexpr unsigned INFRARED_UART_BAUD = 115200U;  // Set 115200 if module was configured so.
constexpr bool INFRARED_AUTO_BAUD = true;
constexpr unsigned INFRARED_BAUD_PROBE_MS = 2000U;
// 0: accept 3/4-byte payloads; nonzero: require this exact payload length.
constexpr unsigned INFRARED_FRAME_DATA_LENGTH = 0U;
// Assumed GY-56 distance field is cm. Use 0.1 if its manual specifies mm.
constexpr float INFRARED_RAW_TO_CM = 1.0f;
constexpr unsigned INFRARED_VALID_TIMEOUT_MS = 500U;
static_assert(
  INFRARED_UART_BAUD == 9600U || INFRARED_UART_BAUD == 115200U, "Unsupported GY-56 baud rate");
static_assert(
  (INFRARED_FRAME_DATA_LENGTH == 0U ||
   (INFRARED_FRAME_DATA_LENGTH >= 2U && INFRARED_FRAME_DATA_LENGTH <= 8U)),
  "Invalid GY-56 payload length");
static_assert(INFRARED_RAW_TO_CM > 0.0f, "Invalid distance unit conversion");

// Initial tuning values only; NOT calibrated for this chassis.
// Torque: gearbox output shaft N*m; speed: same shaft rad/s.
// Refit K1/K2 if the shaft convention or actual gear ratio changes.
// K1 是铜损系数；按参考文档它还承担"加减速阶段由加速度引起的额外能耗"。
// 【2026-10 标定尝试记录】用 data_1（1291 采样，扣除 7.4% 刹车/再生）最小二乘得
//   K1 = 3.97, K2 ≈ 0, K3 = 4.82，残差 ±1.49 W、R² = 0.981。
//   但该批数据最高只到 62 W，76~80 W 属于外推 → 实机表现反而比下面这组经验值差，
//   所以暂时回到"经验值 + 8 W 裕量"的方案。
//   若要让标定真正生效：把 CHASSIS_POWER_LIMIT_W 临时设 200、绘图模式设 4，
//   录一段真正跑到 90~100 W 的数据重新拟合，再用 3~4 W 裕量。
// K2 must be calibrated in the actual shaft-speed coordinates.
// Rotor-to-output conversion multiplies the speed-squared coefficient by ratio^2.
constexpr float POWER_K1 = 2.03f;   // W / (N*m)^2
constexpr float POWER_K2 = 0.009f;  // W / (rad/s)^2
constexpr float POWER_K3 = 4.1f;    // W, 整机空载损耗

constexpr bool POWER_MODEL_CALIBRATED = false;

constexpr float CHASSIS_POWER_LIMIT_W = 80.0f;
// Measured-power PI target = LIMIT - MARGIN; model budget = target + bounded PI trim.
// 标定后模型误差只有 ±1.5 W（最大 6.5 W），所以裕量可以从 8 W 降回 4 W：
//   限幅点 76 W，均值贴 76，峰值约 80~82。
// 想更激进用 3；想更保守用 5~6。
constexpr float POWER_LIMIT_MARGIN_W = 2.0f;
static_assert(CHASSIS_POWER_LIMIT_W > 0.0f, "Power limit must be positive");
static_assert(
  POWER_LIMIT_MARGIN_W >= 0.0f && POWER_LIMIT_MARGIN_W < CHASSIS_POWER_LIMIT_W,
  "Invalid power margin");
constexpr unsigned POWER_LIMIT_SEARCH_STEPS = 24;
static_assert(POWER_LIMIT_SEARCH_STEPS > 0, "Search needs at least one step");

// 功率计多久没更新就视为无效（不再参与修正）
constexpr unsigned POWER_METER_TIMEOUT_MS = 200U;

// Measured-power PI (budget correction in W), matching power_model.cpp.
constexpr float POWER_PI_KP = 0.4f;
constexpr float POWER_PI_KI = 1.5f;
constexpr float POWER_PI_FILTER_S = 0.03f;
constexpr float POWER_PI_DEADBAND_W = 0.3f;
constexpr float POWER_PI_TRIM_MIN_W = -40.0f;
constexpr float POWER_PI_TRIM_MAX_W = 12.0f;
constexpr float POWER_PI_RISE_W_PER_S = 20.0f;
constexpr float POWER_FAST_DROP_KP = 2.0f;
constexpr float POWER_FAST_OVER_MAX_SCALE = 0.95f;
static_assert(
  POWER_PI_KP >= 0.0f && POWER_PI_KI >= 0.0f && POWER_PI_FILTER_S > 0.0f &&
    POWER_PI_DEADBAND_W >= 0.0f && POWER_PI_TRIM_MIN_W <= 0.0f && POWER_PI_TRIM_MAX_W >= 0.0f &&
    POWER_PI_RISE_W_PER_S > 0.0f,
  "Invalid measured-power PI");
static_assert(
  POWER_FAST_DROP_KP > 0.0f && POWER_FAST_OVER_MAX_SCALE > 0.0f && POWER_FAST_OVER_MAX_SCALE < 1.0f,
  "Invalid fast power guard");

// Fast measured-power trend guard, separate from the original prediction formula.
constexpr float POWER_TREND_LEAD_S = 0.05f;    // Forecast horizon.
constexpr float POWER_TREND_FILTER_S = 0.04f;  // Filter power slope only.
constexpr float POWER_TREND_RECOVER_PER_S =
  2.0f;  // Recovery rate: up to two scale units per second.
constexpr float POWER_TREND_RECOVER_BAND_W = 0.25f;
static_assert(
  POWER_TREND_LEAD_S >= 0.0f && POWER_TREND_LEAD_S <= 0.5f, "Invalid power forecast horizon");
static_assert(
  POWER_TREND_FILTER_S > 0.0f && POWER_TREND_RECOVER_PER_S > 0.0f &&
    POWER_TREND_RECOVER_BAND_W >= 0.0f,
  "Invalid power trend guard parameters");

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
constexpr unsigned FOLLOW_YAW_MOTOR_ID = 4U;
constexpr unsigned FOLLOW_PITCH_MOTOR_ID = 2U;
constexpr float FOLLOW_YAW_RATIO = 1.0f;
constexpr float FOLLOW_PITCH_RATIO = 1.0f;
// 关节角随编码器角增长。若某轴反向追、误差越追越大，把对应方向改成 -1。
constexpr float FOLLOW_YAW_DIRECTION = 1.0f;
constexpr float FOLLOW_PITCH_DIRECTION = 1.0f;

// ---- 运行模式 ----
// false = 两轴都跟随（当前测 pitch 需要它，否则 pitch 力矩恒为 0）
constexpr bool FOLLOW_SINGLE_YAW = false;
// 恒力矩台架测试：不做闭环，只输出固定力矩（测静摩擦、确认转向）。测完必须改回 false。
// 【方向符号已验证：正力矩 → 实际角度增大，符号正确，DIRECTION 保持 1.0】
constexpr bool FOLLOW_CONSTANT_TORQUE_TEST = false;
constexpr float FOLLOW_TEST_TORQUE_NM = 0.12f;
constexpr float FOLLOW_TEST_TRIP_SPEED_RAD_S = 5.23598776f;  // 50 rpm，超速即锁存停机
// 机械行程保护：裸电机阶段关闭；装上云台后打开并填真实行程。
constexpr bool FOLLOW_ENFORCE_ANGLE_LIMITS = false;
// 时间缩放：1 = 原始 680 ms 周期（最快）；2/4 = 放慢并同时降低速度、加速度。
constexpr unsigned FOLLOW_TRAJECTORY_TIME_SCALE = 2U;  // 1360 ms/圈（缩放 4 已验证 OK，提速一档）
constexpr bool FOLLOW_START_ENABLED =
  true;  // Start yaw tracking after fresh feedback and pose capture.
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
constexpr float FOLLOW_YAW_ANGLE_KP = 15.0f;  // (rad/s)/rad【原始周期：带宽需 ≥3×1.47Hz，10 太小】
constexpr float FOLLOW_YAW_ANGLE_KI = 3.00f;  // 消除峰谷处的静态误差（纯 P 压不掉）
constexpr float FOLLOW_YAW_ANGLE_KD =
  0.0f;  // 与前馈(1.0)冲突：KD 必须为 0 或 1.0，中间值会吃掉驱动
constexpr float FOLLOW_YAW_ANGLE_INTEGRAL_LIMIT =
  0.30f;                                      // rad/s（积分最多贡献 0.3 rad/s 的速度指令）
constexpr float FOLLOW_YAW_SPEED_KP = 0.13f;  // N·m/(rad/s)【装云台后：0.015→0.100】
constexpr float FOLLOW_YAW_SPEED_KI = 0.45f;  // N·m/rad（恢复验证值）
constexpr float FOLLOW_YAW_SPEED_KD = 0.0f;
constexpr float FOLLOW_YAW_SPEED_INTEGRAL_LIMIT = 0.25f;  // N·m
constexpr float FOLLOW_YAW_ANGLE_MIN_RAD = 0.0f;
constexpr float FOLLOW_YAW_ANGLE_MAX_RAD = 0.0f;
constexpr float FOLLOW_YAW_MAX_SPEED_RAD_S = 10.0f;    // 原始周期峰值 7.15
constexpr float FOLLOW_YAW_MAX_ACCEL_RAD_S2 = 130.0f;  // 原始周期峰值 106.9
constexpr float FOLLOW_YAW_MAX_TORQUE_NM = 0.40f;      // ≈0.54 A
constexpr float FOLLOW_YAW_VELOCITY_FF = 1.0f;
constexpr bool FOLLOW_YAW_CONFIGURED = true;

// pitch 轴：与 yaw 同步为"装云台"整定值（内环必须加强，否则跟随严重滞后）
constexpr float FOLLOW_PITCH_ANGLE_KP = 20.0f;  // pitch 惯量小，yaw 的 15 会超调 60%，先降到 6
constexpr float FOLLOW_PITCH_ANGLE_KI = 2.0f;
constexpr float FOLLOW_PITCH_ANGLE_KD = 0.0f;
constexpr float FOLLOW_PITCH_ANGLE_INTEGRAL_LIMIT = 0.30f;
constexpr float FOLLOW_PITCH_SPEED_KP = 0.080f;  // pitch 惯量小，内环适当降低（yaw 用 0.13）
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
// 0 = 底盘功率（实测 / 期望 / 限幅系数），1 = 跟随曲线（单yaw 2通道，双轴4通道），
// 2 = 原始参考，3 = 底盘诊断（FL 目标速度 / 实际速度 / 力矩 / 静止释放标志），
// 4 = 功率模型标定（Στω / Στ²×100 / Σω²÷20 / 实测功率）
constexpr unsigned FOLLOW_PLOT_MODE = 1U;
constexpr bool FOLLOW_YAW_DIAGNOSTICS = true;  // 当前：双轴反馈帧数诊断（查 pitch 电机是否在线）
// 模式 1 画哪根轴：0 = yaw（目标/实际），1 = pitch（目标/实际）
constexpr unsigned FOLLOW_PLOT_AXIS = 1U;

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
static_assert(FOLLOW_PLOT_MODE <= 4U, "Invalid plot mode");
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

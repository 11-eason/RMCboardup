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
constexpr float POWER_K1 = 2.03f;   // W / (N*m)^2（经验值，实测表现优于 3.97）
constexpr float POWER_K2 = 0.009f;  // W / (rad/s)^2
constexpr float POWER_K3 = 4.1f;    // W, 整机空载损耗

// ---- 峰值修正（直接对实测功率做快速削弱）----
// 模型永远有误差，而"实测超过限幅"是确定的。这里按实测超限量立刻再打一折：
//   k = 1 − KP × (P实测 − 允许) / 允许,  限幅在 [MIN, 1]
// 超 10 W（允许 78）时 k≈0.87，超 20 W 时 k≈0.74，能把尖峰压回去。
// 因为闭环建立在"功率计读数"上，KP 太大会跟着测量延迟来回摆，调小即可。
constexpr float POWER_PEAK_KP = 0.0f;   // 0 = 关闭。实测：开启后与限幅环打架，抖动变大
constexpr float POWER_PEAK_MIN = 0.6f;  // 最多再削到 60%
static_assert(
  POWER_PEAK_KP >= 0.0f && POWER_PEAK_MIN > 0.0f && POWER_PEAK_MIN <= 1.0f,
  "Invalid peak correction gains");
constexpr bool POWER_MODEL_CALIBRATED = false;

constexpr float CHASSIS_POWER_LIMIT_W = 80.0f;
// 名义裕量：允许功率 = (LIMIT - MARGIN) × bias。
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

// ---- 实测功率慢速修正（路径 B）----
// 用功率计实测值修正允许功率，使**实测均值**收敛到限幅点。
// 关键：先对实测功率做低通（看均值），否则峰值一超就降额，会把功率压得很低。
// DOWN：实测均值每超限 1 W，系数每毫秒减小多少（10 W 误差约 0.2 s 响应）
constexpr float POWER_BIAS_DOWN_PER_W = 5e-5f;
// UP：解除过度降额/抬高允许值的速率（10 W 偏差约 0.5 s 响应）
constexpr float POWER_BIAS_UP_PER_W = 4e-5f;
// Slow correction can lower the budget and recover to nominal, never above nominal.
constexpr float POWER_BIAS_MIN = 0.6f;
constexpr float POWER_BIAS_MAX = 1.0f;  // Never raise the model budget above limit - margin.
// 实测均值低于限幅多少瓦以上才开始上调
constexpr float POWER_BIAS_RECOVER_BAND_W = 0.5f;
// 实测功率低通时间常数（s）：它决定 bias 盯的是"均值"还是"峰值"
constexpr float POWER_BIAS_FILTER_S = 0.3f;
// Keep existing gains in their nominal-1ms units; scale by actual meter interval.
constexpr float POWER_BIAS_MAX_STEP = 0.01f;  // Bound a single new-frame update.
static_assert(
  POWER_BIAS_MAX_STEP > 0.0f && POWER_BIAS_MAX_STEP <= 0.05f,
  "Invalid measured-power feedback step");

// ---- 加速瞬间的前瞻预留（抑制功率超调）----
// 限幅器改的是指令力矩，而真实功率是"力矩×转速"的物理结果：加速瞬间四轮同时提速，
// 机械功率陡增，会出现 100 ms 级的超调（实测 78 W 均值下峰值冲到 100~110 W）。
// 做法：期望功率上升越快，越提前压低允许值。
//   预留量 = 期望功率上升速率(W/s) × POWER_RESERVE_LEAD_S，上限 POWER_RESERVE_MAX_W
// 代价：加速略缓。LEAD 设 0 即关闭该功能。
constexpr float POWER_RESERVE_LEAD_S = 0.02f;  // 前瞻预留（实测有效，保留）
constexpr float POWER_RESERVE_MAX_W = 8.0f;    // 最多预留 15 W
static_assert(POWER_RESERVE_LEAD_S >= 0.0f && POWER_RESERVE_MAX_W >= 0.0f, "Invalid reserve");
// 功率计多久没更新就视为无效（不再参与修正）
constexpr unsigned POWER_METER_TIMEOUT_MS = 200U;

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

static_assert(
  POWER_BIAS_MIN > 0.0f && POWER_BIAS_MIN <= POWER_BIAS_MAX, "Invalid power bias range");
static_assert(
  POWER_BIAS_DOWN_PER_W > 0.0f && POWER_BIAS_UP_PER_W >= 0.0f && POWER_BIAS_RECOVER_BAND_W >= 0.0f,
  "Invalid power bias gains");

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
constexpr bool FOLLOW_START_ENABLED =
  false;  // Sensor bench test: no automatic motor start.              // 上电即开始跟随（台架用）
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
// 0 = 底盘功率（实测 / 期望 / 限幅系数），1 = 跟随曲线（4 通道：两轴的目标/实际），
// 2 = 原始参考，3 = 底盘诊断（FL 目标速度 / 实际速度 / 力矩 / 静止释放标志），
// 4 = 功率模型标定（Στω / Στ²×100 / Σω²÷20 / 实测功率）
constexpr unsigned FOLLOW_PLOT_MODE = 0U;

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

# 云台 IMU 打分

板载 BMI088 使用 SPI1，ACC 片选 PA4、GYRO 片选 PB0。沿用用户参考工程中带返回状态检查的 SPI 驱动。加速度计 ±6g/1600Hz，陀螺仪 ±2000°/s/1000Hz；SPI 运行时设为 /16，即 5.25MHz。

## 启动和测量

仅在 HOMING_ENABLED=false、PITCH_START_ENABLED=false 且 FOLLOW_START_ENABLED=true 时，执行以下跟随启动流程：保持云台静止上电，先预热3秒，再采集至少2秒静止数据校准三轴陀螺仪零偏。晃动会重启静止采样窗口。完成校准并收到 yaw 电机反馈后，以当前姿态对齐轨迹初始角度并启动 yaw 跟随。小幅不水平通过重力初始化处理。homing 和 pitch_start 是独立测试，不依赖这段跟随启动流程，也不是它的必经步骤。

IMU 采集、姿态解算、打分使用现有 `grading → StartTask13` 的1ms任务，未新增或改动 FreeRTOS 任务配置。传感器或采样发生故障时暂停评分，并停止跟随，重新上电后重新校准。轨迹跟随故障码6表示IMU不可用。

三轴陀螺仪积分四元数，用重力修正 roll/pitch；yaw 作多圈展开，跨 ±π 不跳变。yaw 无磁力计绝对航向修正，会有积分漂移，温度变化也会影响零偏；此版本用于短时跟随评估，未启用温控。

PID 实际角度仍为编码器。打分 `yaw_detect`/`pitch_detect` 的实际角度只取自融合后的 IMU 角度，无编码器回退。分数仍使用用户给定阈值和 alpha=0.001，范围0～1；未运行时暂停更新并保留已有分数。

## 串口

当前测试使用四针 UART2（USART1/huart1），921600、8N1，小端Float32，包头 AA BB，后接1字节有效载荷字节数与浮点数据。当前3通道：

1. yaw目标角度，rad；按IMU样本时刻取同一目标轨迹相位。
2. yaw IMU实际角度，rad；未校准或数据无效时为NaN，可能显示空白。
3. yaw分数，0～1。

电机编码器诊断模式仍保留编码器数据；普通跟随曲线已切换为IMU数据。评分频率1kHz，串口输出约50Hz。

## 安装和参数

`control_constants.hpp`：

- `IMU_YAW_DIRECTION`：IMU yaw与目标相反时改为 -1。只影响IMU角度坐标，不改电机PID方向。
- `IMU_PITCH_DIRECTION`：pitch符号。
- `IMU_SENSOR_TO_GIMBAL`：传感器到云台的安装旋转矩阵，默认沿用中间件示例：C板平放、CAN一侧朝前。只做yaw时，板面内的安装朝向不影响相对yaw。开始pitch测试前必须核对前方、左右轴关系；水平并不能确定pitch对应哪根轴。
- `IMU_WARMUP_MS=3000`、`IMU_CALIBRATION_MS=2000`：预热与静止校准。
- `IMU_MAX_GAP_MS=10`：数据过期/采样间隔上限。
- `IMU_ATTITUDE_KP=0.5`：重力修正增益。强线性加速度时跳过该次重力修正。

读 `gimbal_imu::read()` 可查看 phase、fault、init_error、calibration_samples、calibration_restarts、max_interval_ms。phase：0启动、1预热、2静止校准、3就绪、4故障。fault：1初始化、2SPI、3采样间隔、4数值/量程。init_error：1SPI、2ACC芯片ID、3GYRO芯片ID、4ACC配置、5GYRO配置、6HAL SPI初始化。

IMU零点只在跟随起点与编码器零点同时捕获，不在跟随过程中偷偷重新设零。是否执行独立测试由 `HOMING_ENABLED` 与 `PITCH_START_ENABLED` 决定；两者都关闭时，跟随由 `FOLLOW_START_ENABLED` 决定，IMU校准不触发任何找零程序。

## 启动排障（临时12通道）

`IMU_STARTUP_DIAGNOSTICS=true` 时，前三通道保持目标/实际/分数。串口工具须设12通道，其他参数不变；设为false恢复3通道。

- 4：IMU阶段，0初始化、1预热、2校准、3就绪、4故障。
- 5：IMU故障，0正常、1初始化、2SPI、3采样间隔、4量程/数值。
- 6：初始化错误码，见上文 init_error。
- 7：静止校准时间进度0～1；晃动/噪声/采样间断会使进度重置。
- 8：每次串口输出期间收到的yaw反馈帧数；0表示该时段没有ID4反馈。
- 9：跟随故障码，0正常，3反馈，4调度，5参数，6IMU。
- 10：加速度模长，静止时应约9.8m/s²；与板子是否水平无关。
- 11：三轴最大原始角速度绝对值，°/s；静止校准要求小于3°/s。
- 12：上次校准重启原因，1运动、2重力模长、3整体噪声/分段均值漂移、4采样间断；恢复后保留历史原因。

校准条件改为真实连续静止时间≥2000ms且有效样本≥500。之前同时要求2000个样本，实际任务若不足1kHz，会使等待明显超过2秒；此修正不放宽重力、静止或数据有效性保护。SPI超时上限改为5ms，以免1ms边界下过早判失败，正常读取速度不因超时上限改变。

## 校准噪声判定修正

旧版要求整个两秒窗口的单点最大值减最小值不超过0.8°/s，易被少量孤立尖峰反复打断。现在使用 Welford 方差统计：整个窗口最大轴标准差不超过0.30°/s，同时每100ms分段均值的最大差值不超过0.8°/s。

原有瞬时3°/s、重力模长、数据时效和连续2秒要求保留。零偏仍由全部有效原始样本均值求得；没有改动实际角度、PID或打分公式的滤波。可调参数在 control_constants.hpp 中：IMU_CALIBRATION_STD_RAD_S、IMU_CALIBRATION_BLOCK_MS、IMU_CALIBRATION_MEAN_SPREAD_RAD_S。

串口仍为12通道，不需要更改工具配置。Channel7应逐步到1，Channel4随后到3，开始yaw跟随。Channel12为历史重启原因，即使校准成功也可能保留3；是否完成应看Channel4和7。

数值仿真验证了静止倾斜初始化、倾斜下多圈yaw展开、yaw/pitch耦合运动。仿真不替代C++编译或实机精度验证。

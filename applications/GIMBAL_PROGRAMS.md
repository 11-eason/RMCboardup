# 独立测试程序选择

`homing` 与 `pitch_start` 是同级测试程序，不构成初始化链或前后步骤。

在 `control_constants.hpp` 顶部选择：

| HOMING_ENABLED | PITCH_START_ENABLED | 执行 |
|---|---|---|
| true | false | 只执行homing |
| false | true | 只执行pitch_start |
| false | false | 两个测试均不执行；跟随是否开启由FOLLOW_START_ENABLED决定 |
| true | true | 编译报错；两者控制同一组电机，不能同时发送指令 |

当前保留用户选择：`HOMING_ENABLED=false`、`PITCH_START_ENABLED=true`，跟随开关保留原值false。因此上电直接执行pitch_start，不执行homing。

每个测试的true表示上电自动启动该测试；false表示其任务只阻塞等待，不初始化总线、不运行控制器、不接收占用电机反馈、不发送电机指令。旧HOME_MODE与HOME_AUTO_START已移除，不再由一个程序的开关间接决定另一个程序。

CAN初始化和唯一接收回调位于独立的gimbal_can.cpp。任何实际运行的程序均可初始化公共总线，无需先执行homing。接收分发分别检查两个测试的选中状态；未被测试接收的反馈交给跟随模块。

任意测试选中时，轨迹跟随不计算控制、不发送指令，评分也暂停；测试完成或发生故障后仍留在该程序，不自动切换到另一个程序或轨迹跟随。完成后的停止/保持时刻由各自cpp原有Controller逻辑决定，此次没有改动。

沿用现有homing、pitch_start、trajectory_follow、grading任务，没有修改Src/freertos.c或cboard.ioc。现有两套测试仍共用HOME_PITCH等电机/PID常量；共享参数不意味着执行关系。

若以后要配置某个测试完成后的正式启动流程，应另行明确安排，而不是把homing隐含为所有程序的必经步骤。

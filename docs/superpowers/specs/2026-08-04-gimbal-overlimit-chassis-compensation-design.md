# 云台极限 → 底盘大步补偿设计

## 背景

车沿果树行行驶，树在侧面。到达航点后云台旋转至 ±90° 扫描果子。当果子位于云台机械极限（±90°）之外时，现有逻辑只能通过 `arm:5` 跳过该果子，造成漏采。

## 目标

当视觉系统发送 `arm:1`（目标偏右）或 `arm:2`（目标偏左）且云台已在极限位置时，改用底盘前进 50mm 替代跳过，尝试将果子纳入云台范围。超过重试上限仍失败则放弃。

## 改动范围

- **文件**：`Core/Src/UpperCP.c`
- **函数**：`Arm_func()` 中 `temp_num == 1` 和 `temp_num == 2` 两个分支
- **新增**：`static uint8_t s_retry_count` 重试计数器

## 详细设计

### arm:1（目标偏右）

```
读 PCA9685_Get180Angle(7U)
├─ angle >= 90.0f  → 云台已到右极限
│   ├─ s_retry_count++ → ≥5 → 跳过（同 arm:5 清理流程）
│   └─ <5 → Emm_V5_Chassis_Pos_Control(0, 50, 20, 50.0f) + osDelay(500ms)
│
└─ angle < 90.0f   → PCA9685_Set180Angle(7U, angle + 1.0f)  // 云台右微调 +1°
                      osDelay(500ms)
```

### arm:2（目标偏左）

```
读 PCA9685_Get180Angle(7U)
├─ angle <= -90.0f → 云台已到左极限
│   ├─ s_retry_count++ → ≥5 → 跳过（同 arm:5 清理流程）
│   └─ <5 → Emm_V5_Chassis_Pos_Control(0, 50, 20, 50.0f) + osDelay(500ms)
│
└─ angle > -90.0f  → PCA9685_Set180Angle(7U, angle - 1.0f)  // 云台左微调 -1°
                      osDelay(500ms)
```

### arm:0（抓取成功）和 arm:5（跳过）

入口处添加 `s_retry_count = 0;` 清零计数器。

### 放弃流程（≥5 次重试）

```
Arm_ExtendZero();                          // 伸缩归零
osDelay(1000ms);
Arm_SetRotateAngle(0.0f);                  // 云台回中
osDelay(200ms);
App_NotifyGrabDone();                      // 通知跳到下一个点
```

## 参数汇总

| 参数 | 值 | 说明 |
|------|-----|------|
| 云台极限阈值 | `== 90.0f` / `== -90.0f` | 精确匹配，不用余量 |
| 云台微调步长 | ±1° | `PCA9685_Set180Angle` |
| 大步距离 | 50.0mm | `Emm_V5_Chassis_Pos_Control` 前进 |
| 最大重试次数 | 5 | 5×50mm = 250mm |
| 大步后等待 | 500ms | 给视觉重新检测的时间 |

## 视觉系统

**无需改动。** 视觉系统继续正常发送 `arm:1`/`arm:2`/`arm:0`/`arm:5`。STM32 侧拦截云台极限情况做底盘补偿。

## 自检

- [x] 无 TBD / TODO
- [x] arm:1 和 arm:2 逻辑对称一致
- [x] 重试计数器在 arm:0/arm:5 时清零，不会跨果子累积
- [x] 放弃流程与 arm:5 现有清理逻辑一致
- [x] 方向固定前进（不根据云台左右反转），用户确认

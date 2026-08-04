# 云台极限 → 底盘大步补偿 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 云台到 ±90° 极限时，视觉仍报 arm:1/arm:2 则底盘前进 50mm 再试，超过 5 次放弃。

**Architecture:** 只改 `Arm_func()` 中 arm:1/arm:2 两个分支，加一个 static 重试计数器，arm:0/arm:5 时清零。

**Tech Stack:** STM32F407, FreeRTOS, PCA9685, Emm_V5 步进驱动

## Global Constraints

- 云台极限阈值：`gimbal_angle >= 90.0f` / `gimbal_angle <= -90.0f`
- 大步距离：50.0mm，固定前进
- 最大重试：5 次
- 放弃动作：与 arm:5 现有逻辑一致（`Move_up(5.0f)` + 伸缩归零 + 云台回中 + `App_NotifyGrabDone`）
- 视觉系统无改动

---

### Task 1: 新增重试计数器 + 清零逻辑

**Files:**
- Modify: `Core/Src/UpperCP.c`

**Interfaces:**
- Produces: `static uint8_t s_retry_count` （全局文件级可见，Arm_func 内读写）

- [ ] **Step 1: 在 Arm_func 上方新增 static 计数器**

在 `uint8_t CameraFlag = 0;`（[UpperCP.c:104](Core/Src/UpperCP.c#L104)）之后添加：

```c
static uint8_t s_retry_count = 0;  /**< 云台极限重试计数器 */
```

- [ ] **Step 2: arm:0 入口清零计数器**

在 `if(temp_num == 0)` 分支开头（[UpperCP.c:288](Core/Src/UpperCP.c#L288) 的 `{` 之后）插入：

```c
s_retry_count = 0;
```

- [ ] **Step 3: arm:5 入口清零计数器**

在 `} else if (temp_num == 5)` 分支开头（[UpperCP.c:310](Core/Src/UpperCP.c#L310) 的 `{` 之后）插入：

```c
s_retry_count = 0;
```

- [ ] **Step 4: 提交**

```bash
git add Core/Src/UpperCP.c
git commit -m "feat: 新增云台极限重试计数器及清零逻辑"
```

---

### Task 2: 改造 arm:1 分支（目标偏右）

**Files:**
- Modify: `Core/Src/UpperCP.c:252-263`

**Interfaces:**
- Consumes: `s_retry_count`（Task 1）, `PCA9685_Get180Angle(7U)`（已有）, `gimbal_angle`（线 250 已读取）
- Produces: 无新增接口

- [ ] **Step 1: 替换 arm:1 分支逻辑**

将 [UpperCP.c:252-263](Core/Src/UpperCP.c#L252-L263)：

```c
        if(temp_num == 1)       //目标偏右：整车向前移动 1cm (10.0mm)
        {
            // if (gimbal_angle > 0.0f)   // 云台在右侧：目标偏右 = 车前进
            // {
            //     Emm_V5_Chassis_Pos_Control(0, 50, 20, 10.0f);
            // }
            // else                        // 云台在左侧：方向反转，目标偏右 = 车后退
            // {
            //     Emm_V5_Chassis_Pos_Control(1, 20, 50, 10.0f);
            // }
             PCA9685_Set180Angle(7U,s_pca9685_180_angles[7]+1);
            osDelay(pdMS_TO_TICKS(500U));
        }else
```

替换为：

```c
        if(temp_num == 1)       //目标偏右
        {
            if (gimbal_angle >= 90.0f)   // 云台到右极限：大步前移
            {
                s_retry_count++;
                if (s_retry_count >= 5)
                {
                    /* 放弃：同 arm:5 清理流程 */
                    if (upordownFlag == 0)
                    {
                        Move_up(5.0f);
                        vTaskDelay(pdMS_TO_TICKS(500U));
                        Arm_ExtendZero();
                        vTaskDelay(pdMS_TO_TICKS(1000U));
                        Arm_SetRotateAngle(0.0f);
                        vTaskDelay(pdMS_TO_TICKS(200U));
                    }
                    App_NotifyGrabDone();
                    return;
                }
                Emm_V5_Chassis_Pos_Control(0, 50, 20, 50.0f);  // 前进 50mm
            }
            else
            {
                PCA9685_Set180Angle(7U, gimbal_angle + 1.0f);   // 云台右微调 +1°
            }
            osDelay(pdMS_TO_TICKS(500U));
        }else
```

- [ ] **Step 2: 提交**

```bash
git add Core/Src/UpperCP.c
git commit -m "feat: arm:1 云台极限→底盘前进50mm补偿"
```

---

### Task 3: 改造 arm:2 分支（目标偏左）

**Files:**
- Modify: `Core/Src/UpperCP.c:265-277`

**Interfaces:**
- Consumes: `s_retry_count`（Task 1）, `gimbal_angle`（线 250 已读取）
- Produces: 无新增接口

- [ ] **Step 1: 替换 arm:2 分支逻辑**

将 [UpperCP.c:265-277](Core/Src/UpperCP.c#L265-L277)：

```c
        if(temp_num == 2)       //目标偏左：整车向后移动 1cm (10.0mm)
        {
            // if (gimbal_angle > 0.0f)   // 云台在右侧：目标偏左 = 车后退
            // {
            //     Emm_V5_Chassis_Pos_Control(1, 20, 50, 10.0f);
            // }
            // else                        // 云台在左侧：方向反转，目标偏左 = 车前进
            // {
            //     Emm_V5_Chassis_Pos_Control(0, 50, 20, 10.0f);
            // }
            PCA9685_Set180Angle(7U,s_pca9685_180_angles[7]-1);
            osDelay(pdMS_TO_TICKS(500U));
        }else
```

替换为：

```c
        if(temp_num == 2)       //目标偏左
        {
            if (gimbal_angle <= -90.0f)  // 云台到左极限：大步前移
            {
                s_retry_count++;
                if (s_retry_count >= 5)
                {
                    /* 放弃：同 arm:5 清理流程 */
                    if (upordownFlag == 0)
                    {
                        Move_up(5.0f);
                        vTaskDelay(pdMS_TO_TICKS(500U));
                        Arm_ExtendZero();
                        vTaskDelay(pdMS_TO_TICKS(1000U));
                        Arm_SetRotateAngle(0.0f);
                        vTaskDelay(pdMS_TO_TICKS(200U));
                    }
                    App_NotifyGrabDone();
                    return;
                }
                Emm_V5_Chassis_Pos_Control(0, 50, 20, 50.0f);  // 前进 50mm
            }
            else
            {
                PCA9685_Set180Angle(7U, gimbal_angle - 1.0f);   // 云台左微调 -1°
            }
            osDelay(pdMS_TO_TICKS(500U));
        }else
```

- [ ] **Step 2: 提交**

```bash
git add Core/Src/UpperCP.c
git commit -m "feat: arm:2 云台极限→底盘前进50mm补偿"
```

---

## Self-Review

1. **Spec coverage:** ✓ arm:1 极限判断 → arm:2 极限判断 → 重试计数 → 放弃清理 → arm:0/arm:5 清零，全部覆盖
2. **Placeholder scan:** ✓ 无 TBD/TODO/占位符
3. **Type consistency:** ✓ `gimbal_angle`（float，线 250 已读）在 Task 2/3 复用，`s_retry_count`（uint8_t）在 Task 1 定义、Task 2/3 消费，`upordownFlag`（uint8_t）已存在

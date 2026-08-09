# C 区二维码导航 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 A 区结束与 C 区导航之间加入 10 秒二维码扫描阶段，根据 8 个水果位置生成较短路线，并在每个停车点只处理所需的云台方向。

**Architecture:** `UpperCP.c` 严格解析并原子提交二维码位置；`app.c` 管理扫码超时、位置映射、两方向路线比较和非阻塞作业推进。沿用现有矩形环路、导航底层和抓取状态机，不新增生产模块。

**Tech Stack:** STM32F407、ARMCC 5、STM32 HAL、FreeRTOS/CMSIS-RTOS2、嵌入式 C、PowerShell 静态检查。

## Global Constraints

- C 区入口为 `WAYPOINT(-2600.0f, 10.0f, PI, false)`。
- 默认数组保持 `uint8_t fruits[8] = {4,3,1,10,8,9,2,11};`。
- 只接受恰好 8 个、范围 `1..12`、互不重复的整数位置。
- 扫码只发送一次 `scan\r\n`；有效帧立即结束等待，10 秒超时使用默认数组。
- 二维码数组顺序不参与访问顺序，选择两种合法巡航方向中的较短者。
- 左通道 `x=-1900`，右通道 `x=-2600`，纵坐标为 `400/900/1400/1900`。
- `1..4` 为左通道 `+90°`，`5..8` 为左通道 `-90°`，`9..12` 为右通道 `+90°`。
- 同一坐标只停车一次；双侧按 `+90°`、`-90°` 处理，单侧不扫描另一侧。
- 不修改导航底层、PCA9685 标定和抓取动作内部流程。

---

### Task 1: 二维码严格解析和原子提交

**Files:**
- Modify: `Core/Src/UpperCP.c:186-190,339-357`
- Modify: `Core/Inc/UpperCP.h:6-10`

**Interfaces:**
- Consumes: `QR:<位置1>,...,<位置8>;`，终止符由 `UpperCP_RX()` 剥离。
- Produces: 合法帧才同时更新 `fruits[8]`、`fruits_count`、`CameraFlag`；新增 `void UpperCP_ResetQrResult(void)`。

- [ ] **Step 1: 运行旧行为检查**

```powershell
rg -n "sscanf\(p_num, \"%f\"|fruits\[i\+\+\]|fruits_count = i" Core/Src/UpperCP.c
```

Expected before implementation: 命中逐项直接覆盖数组和浮点解析代码。

- [ ] **Step 2: 写最小严格解析实现**

在 `ErWeiMa_func()` 使用局部 `uint8_t parsed[8]`、`bool seen[13]` 和 `strtol()`。每个 token 必须完整解析为整数、范围为 `1..12`、未重复且总数不超过 8；最终恰好 8 项才执行：

```c
memcpy(fruits, parsed, sizeof(fruits));
fruits_count = 8U;
CameraFlag = 1U;
```

增加只清除接收状态、不改默认数组的接口：

```c
void UpperCP_ResetQrResult(void)
{
    fruits_count = 0U;
    CameraFlag = 0U;
}
```

- [ ] **Step 3: 验证解析契约**

```powershell
rg -n "strtol|parsed\[8\]|seen\[13\]|memcpy\(fruits|fruits_count = 8U|UpperCP_ResetQrResult" Core/Src/UpperCP.c Core/Inc/UpperCP.h
git diff --check -- Core/Src/UpperCP.c Core/Inc/UpperCP.h
```

Expected: 严格校验、原子复制和复位接口均命中，diff 检查无输出。

### Task 2: 10 秒非阻塞扫码阶段

**Files:**
- Modify: `Core/Inc/app.h:10-17`
- Modify: `Core/Src/app.c:19-43,104-112,136-160,228-245`

**Interfaces:**
- Consumes: `UpperCP_ResetQrResult()`、`CameraFlag`、`fruits_count`、`fruits`。
- Produces: `APP_MODE_SCAN_C`；首次 Tick 发送一次 `scan`，有效数据或 10000 ms 超时后进入 C 区规划。

- [ ] **Step 1: 运行旧行为检查**

```powershell
rg -n -A6 "case APP_MODE_ROUTE_C" Core/Src/app.c
```

Expected before implementation: 同一分支发送 `scan` 后立即使用硬编码数组规划。

- [ ] **Step 2: 写扫码状态实现**

定义 `APP_QR_SCAN_TIMEOUT_MS 10000U`、首次发送标志和截止 Tick。A 路线完成后转入 `APP_MODE_SCAN_C`。首次进入时调用 `UpperCP_ResetQrResult()`、`UpperCP_SendTask("scan")` 并记时；后续 Tick 仅在 `CameraFlag != 0U && fruits_count == 8U` 或超时时进入 `APP_MODE_ROUTE_C`。超时不修改 `fruits`。

`APP_MODE_ROUTE_C` 只调用新规划接口并以 `APP_MODE_BACK` 为下一模式，不再发送 `scan`。

同时把 A 路线最后一个 C 区入口航点改为 `WAYPOINT(-2600.0f, 10.0f, PI, false)`，确保扫码和 C 区规划开始前底盘已到新入口并保持确认的朝向。

- [ ] **Step 3: 验证扫码状态**

```powershell
rg -n "APP_QR_SCAN_TIMEOUT_MS|APP_MODE_SCAN_C|UpperCP_ResetQrResult|UpperCP_SendTask\(\"scan\"\)|fruits_count == 8U" Core/Src/app.c Core/Inc/app.h
rg -n "\{2,4,8,10\}" Core/Src/app.c
git diff --check -- Core/Src/app.c Core/Inc/app.h
```

Expected: 超时值为 10000，`scan` 只在扫码首次进入路径，旧硬编码无命中。

### Task 3: 位置映射和较短路线生成

**Files:**
- Modify: `Core/Inc/app.h:20-27,73-100`
- Modify: `Core/Src/app.c:19-40,104-128,430-564`

**Interfaces:**
- Consumes: `const uint8_t fruit_positions[8]`。
- Produces: `AppWaypoint_t.action_mask`；`APP_ACTION_POSITIVE` 表示 `+90°`，`APP_ACTION_NEGATIVE` 表示 `-90°`；`App_RouteC_PlanAndRun(const uint8_t *fruit_positions, AppMode_t next_mode)` 生成 `s_dynamic_route`。

- [ ] **Step 1: 运行旧规划接口检查**

```powershell
rg -n "start_node_idx|target_nodes|is_target\[12\]|has_action = is_target" Core/Src/app.c Core/Inc/app.h
```

Expected before implementation: 输入仍被直接当作 `0..11` 环路节点，不能表达同点不同云台侧。

- [ ] **Step 2: 扩展航点动作位图**

保留 `has_action` 并新增 `uint8_t action_mask`，定义：

```c
#define APP_ACTION_NONE      0x00U
#define APP_ACTION_POSITIVE  0x01U
#define APP_ACTION_NEGATIVE  0x02U
```

更新 `WAYPOINT` 宏，使原有 A 区 `act == true` 的点默认具有正负两侧动作位，无动作点为 `APP_ACTION_NONE`，保持 A 区行为不变。

- [ ] **Step 3: 写固定位置映射和路线选择**

映射为：`1/5 -> node4(+/-)`、`2/6 -> node3(+/-)`、`3/7 -> node2(+/-)`、`4/8 -> node1(+/-)`、`9 -> node7(+)`、`10 -> node8(+)`、`11 -> node9(+)`、`12 -> node10(+)`。起点固定为 node 11，即 `(-2600,10,PI)`。

分别沿顺、逆方向累加相邻节点的实际毫米距离，比较覆盖全部目标所需的总距离，平局保持顺时针优先；不得用节点数量代替距离。生成路线时合并同一 node 的动作位，保留目标点、最终点及 `0/5/6/11` 四个拐角，删除直线上的无动作中间点，并保证输出不超过 `s_dynamic_route[12]`。返回路线写入的两个无动作航点也必须显式设置 `action_mask = APP_ACTION_NONE`，避免复用动态数组时残留旧动作位。

- [ ] **Step 4: 验证映射和旧接口清理**

```powershell
rg -n "APP_ACTION_POSITIVE|APP_ACTION_NEGATIVE|fruit_positions|-2600\.0f, 10\.0f, PI" Core/Src/app.c Core/Inc/app.h
rg -n "num_targets|start_node_idx|target_nodes" Core/Src/app.c Core/Inc/app.h
git diff --check -- Core/Src/app.c Core/Inc/app.h
```

Expected: 新动作位、位置输入和入口命中；旧规划参数无命中；diff 检查无输出。

### Task 4: 按动作位处理目标侧

**Files:**
- Modify: `Core/Src/app.c:285-413`

**Interfaces:**
- Consumes: 当前航点的 `action_mask`。
- Produces: 正向单侧、反向单侧和双侧三种非阻塞作业路径。

- [ ] **Step 1: 运行旧固定双侧检查**

```powershell
rg -n "StartGimbalMove\(90\.0f|StartGimbalMove\(-90\.0f|APP_ROUTE_WAIT_GRAB_FIRST|APP_ROUTE_WAIT_GRAB_SECOND" Core/Src/app.c
```

Expected before implementation: 每个作业点固定先执行 `+90°` 再执行 `-90°`。

- [ ] **Step 2: 用动作位控制阶段跳转**

到达作业点时，包含 `APP_ACTION_POSITIVE` 才进入第一视野，否则在完成同样的升降安全等待后直接进入第二视野。第一视野完成后，仅在包含 `APP_ACTION_NEGATIVE` 时进入第二视野，否则完成航点。双侧保持先正后负，不改 `ActionScheduler` 内部流程。

- [ ] **Step 3: 验证三种动作路径**

```powershell
rg -n "action_mask.*APP_ACTION_POSITIVE|action_mask.*APP_ACTION_NEGATIVE|APP_ROUTE_FIRST_WAIT|APP_ROUTE_SECOND_WAIT" Core/Src/app.c
git diff --check -- Core/Src/app.c
```

Expected: 两个动作位分别控制对应阶段，双侧仍按正后负推进。

### Task 5: 整体验证和交付

**Files:**
- Verify: `Core/Src/UpperCP.c`
- Verify: `Core/Inc/UpperCP.h`
- Verify: `Core/Src/app.c`
- Verify: `Core/Inc/app.h`
- Verify: `MDK-ARM/vet6_mdk.uvprojx`

**Interfaces:**
- Consumes: Tasks 1-4 的最终源码。
- Produces: 静态契约结果、构建证据和板端验证清单。

- [ ] **Step 1: 运行完整静态契约检查**

```powershell
rg -n "uint8_t fruits\[8\] = \{4,3,1,10,8,9,2,11\};|APP_QR_SCAN_TIMEOUT_MS.*10000|APP_MODE_SCAN_C|APP_ACTION_POSITIVE|APP_ACTION_NEGATIVE|UpperCP_SendTask\(\"scan\"\)" Core/Src/UpperCP.c Core/Src/app.c Core/Inc/app.h
git diff --check
```

Expected: 所有核心契约命中，diff 检查无输出。

- [ ] **Step 2: 检查 Keil 工程成员**

```powershell
rg -n "<FileName>(app|UpperCP)\.c</FileName>" MDK-ARM/vet6_mdk.uvprojx
```

Expected: `app.c`、`UpperCP.c` 均为 Keil 工程成员。

- [ ] **Step 3: 执行 Keil 命令行构建**

```powershell
& 'D:\keil5\UV4\UV4.exe' -b '.\MDK-ARM\vet6_mdk.uvprojx' -t 'vet6_mdk' -o '.\MDK-ARM\c_zone_qr_build.log'
Get-Content '.\MDK-ARM\c_zone_qr_build.log'
```

Expected: 构建日志显示 0 errors；如许可证或 Keil 环境阻止命令行构建，保留完整错误文本，并明确要求用户在 Keil 中执行 Rebuild All 后回传日志。静态检查不得表述为板端验证。

- [ ] **Step 4: 检查最终范围**

```powershell
git status --short
git diff --stat
```

Expected: 生产源码仅涉及 `UpperCP.c/.h`、`app.c/.h`，不包含 `navigation.c`、`pca9685.c`、`action_scheduler.c`。

- [ ] **Step 5: 执行板端检查清单**

1. 进入扫码阶段确认 UART5 只发送一次 `scan\r\n`。
2. 10 秒内发送合法帧，确认立即起步且数组一致。
3. 不发送二维码，确认约 10 秒后按默认数组起步。
4. 测试左通道同点双侧 `1/5`、正向单侧 `2`、反向单侧 `6`。
5. 测试右通道 `9..12` 只转 `+90°`。
6. 确认从 `(-2600,10,PI)` 出发，选择较短方向且拐角不斜切。

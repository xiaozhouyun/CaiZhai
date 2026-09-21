# B 区跑点与抓取路线 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 C 区完成后从底部进入 B 区，到 `(-1500,2350)` 经过空扫码状态，再沿同一通道按 `1→5→2→6→3→7→4→8` 单侧抓取并返回原点。

**Architecture:** 继续使用 `app.c` 现有非阻塞路线状态机。`APP_MODE_ROUTE_B` 动态生成三点入口路线，`APP_MODE_SCAN_B` 只作为未来扫码入口并立即启动固定 B 区作业路线；抓取和返回均复用现有调度逻辑。

**Tech Stack:** STM32F407 HAL、FreeRTOS、C11、PowerShell、MinGW GCC 主机测试。

## Global Constraints

- 当前 B 区扫码状态不得发送 `scan`、读取二维码结果或等待 8 秒。
- B 区入口必须从底部走，不能从 C 区斜穿顶部二维码区域。
- B 区底盘只走 `x=-1500 mm`，不左右横移。
- 1/2/3/4 只转 `+90°`，5/6/7/8 只转 `-90°`。
- 不修改 `navigation.c`、`action_scheduler.c`、`UpperCP.c`、A 区坐标或 C 区规划算法。
- 不自动启动 Keil；主机测试完成后由用户在 Keil 编译并返回日志。

---

### Task 1: 建立 B 区路线行为测试

**Files:**
- Create: `tests/app_route_b_test.c`
- Create: `tests/run_app_route_b_test.ps1`
- Create: `tests/app_route_b_stubs/main.h`
- Create: `tests/app_route_b_stubs/arms.h`
- Create: `tests/app_route_b_stubs/tiancan.h`
- Create: `tests/app_route_b_stubs/navigation.h`
- Create: `tests/app_route_b_stubs/usart.h`
- Create: `tests/app_route_b_stubs/bujin.h`
- Create: `tests/app_route_b_stubs/voice.h`
- Create: `tests/app_route_b_stubs/pca9685.h`
- Create: `tests/app_route_b_stubs/UpperCP.h`
- Create: `tests/app_route_b_stubs/action_scheduler.h`
- Create: `tests/app_route_b_stubs/cmsis_os.h`
- Create: `tests/app_route_b_stubs/vofa.h`
- Test: `Core/Src/app.c`

**Interfaces:**
- Consumes: `App_Init()`、`App_SetMode()`、`App_RunCurrentMode()`、`App_NotifyGrabDone()`。
- Produces: 一个链接真实 `Core/Src/app.c` 的主机测试，记录 `Navigation_Request()` 航点、云台角度和 UART5 任务字符串。

- [ ] **Step 1: 写失败的主机测试**

测试桩必须把最后一个导航目标保存为 `{x,y,yaw}`；`arrive()` 将 `g_robot_pos` 更新到该目标并令 `Navigation_IsIdle()` 返回真。测试从 `APP_MODE_ROUTE_B` 开始，逐阶段推进真实 `App_RunCurrentMode()`，并用字面量断言：

```c
static const float expected_y[8] = {
    2150.0f, 1950.0f, 1700.0f, 1500.0f,
    1200.0f, 1000.0f, 700.0f, 500.0f
};
static const float expected_gimbal[8] = {
    90.0f, -90.0f, 90.0f, -90.0f,
    90.0f, -90.0f, 90.0f, -90.0f
};

assert_waypoint(0, -2600.0f, 10.0f, PI / 2.0f);
assert_waypoint(1, -1500.0f, 10.0f, 0.0f);
assert_waypoint(2, -1500.0f, 2350.0f, PI);
assert(scan_send_count == 0U);
```

对八个作业点逐个执行“导航到达 → 升降/云台等待 → `send` → `App_NotifyGrabDone()`”，断言每次导航的 `x=-1500`、Y 顺序和首次云台角度。最后断言返回请求依次为 `(-1500,10,PI/2)`、`(0,0,PI/2)`，并结束在 `APP_MODE_IDLE`。

`tests/run_app_route_b_test.ps1` 使用真实 `app.c`：

```powershell
$ErrorActionPreference = 'Stop'
$testExe = Join-Path $env:TEMP 'app_route_b_test.exe'
& gcc -std=c11 -Wall -Wextra -Werror -Wno-unused-function `
    -I tests/app_route_b_stubs -I Core/Inc `
    tests/app_route_b_test.c Core/Src/app.c -lm -o $testExe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $testExe
exit $LASTEXITCODE
```

- [ ] **Step 2: 运行测试并确认因功能缺失而失败**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_app_route_b_test.ps1`

Expected: 编译失败，明确缺少 `APP_MODE_SCAN_B` 或 B 区路线行为不符合断言；不得因测试桩缺声明而失败。

---

### Task 2: 实现最小 B 区状态和路线

**Files:**
- Modify: `Core/Inc/app.h`
- Modify: `Core/Src/app.c`
- Test: `tests/app_route_b_test.c`

**Interfaces:**
- Consumes: 现有 `App_StartRoute()`、`App_RouteTick()`、`APP_ACTION_POSITIVE`、`APP_ACTION_NEGATIVE`。
- Produces: `APP_MODE_SCAN_B`、固定 `k_route_b[]`、动态三点 B 区入口路线。

- [ ] **Step 1: 增加扫码占位模式和单侧航点初始化**

在 `APP_MODE_ROUTE_B` 后增加：

```c
APP_MODE_SCAN_B,    /**< B 区扫码占位：当前直接进入 B 区抓取路线 */
```

在不改变现有 `WAYPOINT` 含义的前提下增加：

```c
#define WAYPOINT_SIDE(x, y, yaw, action) \
    {(x), (y), (yaw), true, (action)}
```

- [ ] **Step 2: 增加固定 B 区八点路线**

```c
static const AppWaypoint_t k_route_b[] = {
    WAYPOINT_SIDE(-1500.0f, 2150.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-1500.0f, 1950.0f, PI, APP_ACTION_NEGATIVE),
    WAYPOINT_SIDE(-1500.0f, 1700.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-1500.0f, 1500.0f, PI, APP_ACTION_NEGATIVE),
    WAYPOINT_SIDE(-1500.0f, 1200.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-1500.0f, 1000.0f, PI, APP_ACTION_NEGATIVE),
    WAYPOINT_SIDE(-1500.0f,  700.0f, PI, APP_ACTION_POSITIVE),
    WAYPOINT_SIDE(-1500.0f,  500.0f, PI, APP_ACTION_NEGATIVE),
};
```

- [ ] **Step 3: 接入 C→B入口→空扫码→B抓取→返回状态链**

将 C 区规划的下一模式从 `APP_MODE_BACK` 改为 `APP_MODE_ROUTE_B`。在 `APP_MODE_ROUTE_B` 中写入 `s_dynamic_route[0..2]`：

```c
s_dynamic_route[0] = (AppWaypoint_t){g_robot_pos.x, 10.0f, PI / 2.0f, false, APP_ACTION_NONE};
s_dynamic_route[1] = (AppWaypoint_t){-1500.0f, 10.0f, 0.0f, false, APP_ACTION_NONE};
s_dynamic_route[2] = (AppWaypoint_t){-1500.0f, 2350.0f, PI, false, APP_ACTION_NONE};
App_StartRoute(s_dynamic_route, 3U, APP_MODE_SCAN_B);
```

`APP_MODE_SCAN_B` 当前只执行：

```c
/* 预留：以后在此发送扫码请求并等待完成信号，最长 8 秒。 */
App_StartRoute(k_route_b, APP_ROUTE_LEN(k_route_b), APP_MODE_BACK);
```

- [ ] **Step 4: 运行主机测试并确认通过**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_app_route_b_test.ps1`

Expected: `app_route_b_test: PASS`。

---

### Task 3: 回归与交付检查

**Files:**
- Verify: `Core/Inc/app.h`
- Verify: `Core/Src/app.c`
- Verify: `tests/app_route_b_test.c`
- Verify: `tests/run_app_route_b_test.ps1`

**Interfaces:**
- Consumes: Task 1、2 的实现和测试。
- Produces: 可交给 Keil 编译和实车验证的源码状态。

- [ ] **Step 1: 运行 B 区主机行为测试**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_app_route_b_test.ps1`

Expected: `app_route_b_test: PASS`。

- [ ] **Step 2: 运行现有相关契约测试**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/action_scheduler_link_contract.ps1
powershell -ExecutionPolicy Bypass -File tests/gimbal_before_lift_contract.ps1
powershell -ExecutionPolicy Bypass -File tests/gimbal_settle_before_lower_contract.ps1
```

Expected: `gimbal_before_lift_contract.ps1` 通过。当前分支在本次改动前已有两项基线失败，执行后必须保持相同错误且不得出现新的失败：

- `action_scheduler_link_contract.ps1`：`ZhuaZi_close call is missing.`；
- `gimbal_settle_before_lower_contract.ps1`：`Missing 1000ms mechanical settle time after the final gimbal command.`。

这两项涉及本次明确不修改的 `action_scheduler.c` 既有状态，不在 B 区路线任务中修复。

- [ ] **Step 3: 检查补丁质量和修改范围**

Run:

```powershell
git diff --check
git diff -- Core/Inc/app.h Core/Src/app.c tests/app_route_b_test.c tests/run_app_route_b_test.ps1 tests/app_route_b_stubs
```

Expected: 无空白错误；生产代码只修改 `app.h/app.c`，二维码解析、导航和机械动作文件无改动。

- [ ] **Step 4: 提交实现**

```powershell
git add Core/Inc/app.h Core/Src/app.c tests/app_route_b_test.c tests/run_app_route_b_test.ps1 tests/app_route_b_stubs docs/superpowers/plans/2026-08-12-b-zone-route.md
git commit -m "feat: add B zone route"
```

- [ ] **Step 5: 交付 Keil 与实车检查清单**

要求用户编译 `MDK-ARM/vet6_mdk.uvprojx` 并返回完整日志。实车依次确认：C 区从底部退出、B 区上行三个入口点、扫码点无等待、八点停车顺序、云台正负方向、`(-1500,10)` 转弯和 `(0,0)` 返回。

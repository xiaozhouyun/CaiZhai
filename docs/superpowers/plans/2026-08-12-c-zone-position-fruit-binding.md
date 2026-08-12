# C 区位置与水果绑定 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 STM32 的 C 区视觉请求携带实际位置编号，并让 K230按二维码建立的“位置 → 指定水果”映射判断面积最大的检测框。

**Architecture:** STM32继续独立完成 C 区最短路线，在动态航点中分别保存正、反视野对应的位置并通过扩展后的 `UpperCP_SendTask()` 发送 `send:<位置>`。K230不复制路线算法，只原子解析二维码映射、解析带位置任务并在播报后按当前位置指定水果决定抓取或 `arm:5;`。

**Tech Stack:** STM32F407 HAL、FreeRTOS、C11、PowerShell/MinGW GCC 主机测试、K230 CanMV MicroPython、CPython `unittest`/AST方法提取测试。

## Global Constraints

- C 区使用 `send:1`～`send:12`；A/B 区继续使用普通 `send`。
- 直接扩展现有 `UpperCP_SendTask()` 白名单，不新增发送API。
- 二维码第 `i` 个水果严格对应第 `i` 个位置；K230不得复制 STM32路线算法。
- 只处理面积最大的有效检测框；错误水果仍先播报再 `arm:5;`。
- 当前不处理坏果，最大框为坏果时播报并 `arm:5;`，但保留 `enable_bad_fruit` 分支。
- 无有效二维码映射、位置不存在或畸形位置请求时立即发送一次 `arm:5;`。
- 不使用默认水果映射兜底。
- 不修改 STM32导航底层、机械动作时序、K230模型标签顺序和 `arm:0～6` 含义。
- 不自动启动 Keil；完成源码验证后由用户编译并返回日志。

---

### Task 1: STM32任务白名单行为

**Files:**
- Create: `tests/uppercp_send_task_test.c`
- Create: `tests/run_uppercp_send_task_test.ps1`
- Create: `tests/uppercp_task_stubs/*.h`
- Modify: `Core/Src/UpperCP.c:148-164`

**Interfaces:**
- Consumes: `void UpperCP_SendTask(const char *task)`。
- Produces: 白名单接受 `send`、`send:1..12`、`scan`、`pour`，其余输入不产生 UART5发送。

- [ ] **Step 1: 写真实 `UpperCP.c` 的失败测试**

测试桩记录 `HAL_UART_Transmit()`收到的字节，依次调用：

```c
UpperCP_SendTask("send");
UpperCP_SendTask("send:1");
UpperCP_SendTask("send:12");
UpperCP_SendTask("scan");
UpperCP_SendTask("pour");
```

断言完整输出为：

```text
send\r\nsend:1\r\nsend:12\r\nscan\r\npour\r\n
```

随后逐个输入 `NULL`、`send:0`、`send:13`、`send:abc`、`send:1xxx`、`other`，断言输出长度不再变化。

- [ ] **Step 2: 运行并确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_uppercp_send_task_test.ps1`

Expected: 测试因当前白名单拒绝 `send:1`/`send:12`而失败，不得因缺少 HAL类型或测试桩声明失败。

- [ ] **Step 3: 最小扩展白名单**

在 `UpperCP_SendTask()` 内保留精确的 `send/pour/scan` 判断，并对 `send:` 后缀使用 `strtol`严格验证：完整消费字符串且值在 `1..12`。畸形或越界时直接返回。

- [ ] **Step 4: 运行并确认 GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_uppercp_send_task_test.ps1`

Expected: `uppercp_send_task_test: PASS`。

---

### Task 2: STM32 C 区位置随路线传递

**Files:**
- Modify: `Core/Inc/app.h`
- Modify: `Core/Src/app.c`
- Create: `tests/app_route_c_position_test.c`
- Create: `tests/run_app_route_c_position_test.ps1`
- Reuse: `tests/app_route_b_stubs/*.h`

**Interfaces:**
- Consumes: `App_RouteC_PlanAndRun(const uint8_t *, AppMode_t)`、`UpperCP_SendTask(const char *)`。
- Produces: `AppWaypoint_t.positive_position`、`AppWaypoint_t.negative_position`，以及按实际处理侧发送的位置任务。

- [ ] **Step 1: 写 C 区失败行为测试**

链接真实 `Core/Src/app.c`，以 `{1,5,2,6,3,7,4,8}` 规划 C 区。测试推进导航、云台和抓取完成状态，断言收到的八个视觉任务按实际路线为：

```text
send:4, send:8, send:3, send:7,
send:2, send:6, send:1, send:5
```

再以 `{1,2,3,4,9,10,11,12}` 规划并断言所有发送位置的集合与输入八个位置完全相同、每个只出现一次。现有 B 区主机测试继续断言普通 `send`，防止 A/B 协议回归。

- [ ] **Step 2: 运行并确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/run_app_route_c_position_test.ps1`

Expected: 当前 `AppWaypoint_t`没有位置字段且视觉任务仅为 `send`，测试编译或行为断言按此原因失败。

- [ ] **Step 3: 扩展航点并保存两侧位置**

在 `action_mask` 后增加：

```c
uint8_t positive_position;
uint8_t negative_position;
```

更新所有航点宏和动态航点赋值，默认写 0。C 区规划增加 `target_positive_positions[12]`、`target_negative_positions[12]`；位置映射时同时写动作位和实际位置，复制到 `temp_route`及 `s_dynamic_route`。

- [ ] **Step 4: 统一视觉发送入口**

将 `App_SendVisionTask()`改为接收 `uint8_t position`。位置 0 发送 `send`；位置 1～12 使用 8 字节缓冲区生成 `send:<位置>`并调用 `UpperCP_SendTask()`。第一视野传 `positive_position`，第二视野传 `negative_position`。

- [ ] **Step 5: 运行并确认 GREEN**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_app_route_c_position_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_app_route_b_test.ps1
```

Expected: 两项均 `PASS`。

---

### Task 3: K230二维码位置映射与任务解析

**Files:**
- Modify: `D:\codexproject\zhihuinongye\dayun\model\main.py`
- Create: `D:\codexproject\zhihuinongye\dayun\model\tests\test_position_fruit_binding.py`

**Interfaces:**
- Consumes: 二维码 payload、UART文本 `send`/`send:<位置>`。
- Produces: `fruit_by_position`、`current_position`、`current_target_fruit`、严格任务接收行为。

- [ ] **Step 1: 写二维码与任务解析失败测试**

通过 AST提取 `detect`类中待测方法，绕过桌面不存在的 CanMV依赖。用字面量二维码验证：

```python
payload = "番茄\n洋葱\n南瓜\n辣椒\n番茄\n洋葱\n南瓜\n辣椒\n4,3,1,10,8,9,2,11"
expected = {4: 15, 3: 16, 1: 13, 10: 14, 8: 15, 9: 16, 2: 13, 11: 14}
```

断言数量不等、位置重复/越界、未知水果名不改变旧映射。断言普通 `send`清空位置限制并进入检测；`send:10`选择水果 14；`send:12`在映射缺失时发送一次 `arm:5;`并保持空闲；`send:0`、`send:abc`同样安全跳过。

- [ ] **Step 2: 运行并确认 RED**

Run: `python -m unittest tests.test_position_fruit_binding -v`

Expected: 因 `fruit_by_position`和位置任务解析尚不存在而失败。

- [ ] **Step 3: 原子建立二维码映射**

初始化空映射和当前请求字段。增加一个只负责验证并返回映射结果的方法；`qr_detect()`只有在八组数据全部合法时才覆盖映射、显示列表并发送 `QR:<位置>;`。删除每帧写默认水果/位置的兜底。

- [ ] **Step 4: 解析普通与带位置任务**

将 UART读取与任务判断分开：普通 `send`设置无位置限制；严格合法的 `send:<1..12>`查映射；无映射或畸形带位置请求立即 `arm:5;`并回空闲。`scan/pour`行为保持现状。

- [ ] **Step 5: 运行并确认 GREEN**

Run: `python -m unittest tests.test_position_fruit_binding -v`

Expected: 全部通过。

---

### Task 4: K230最大框、播报后匹配与坏果占位

**Files:**
- Modify: `D:\codexproject\zhihuinongye\dayun\model\main.py`
- Modify: `D:\codexproject\zhihuinongye\dayun\model\tests\test_position_fruit_binding.py`
- Verify: `D:\codexproject\zhihuinongye\dayun\model\tests\test_y_axis_toggle.py`

**Interfaces:**
- Consumes: YOLO检测框、`current_target_fruit`、`enable_bad_fruit`。
- Produces: 只选最大有效框；正常水果先播报类别/成熟度，再按成熟度与当前位置匹配决定抓取或跳过。

- [ ] **Step 1: 写最大框与决策失败测试**

覆盖以下真实行为：

- 大框为辣椒、小框为二维码目标番茄时，选择辣椒；
- 小框为坏果、大框为正常水果时，选择正常水果；
- 最大框为成熟辣椒、当前位置要求番茄时，发送 `voice:14;`、`voice:18;`、`arm:5;`；
- 最大框为成熟番茄且当前位置要求番茄时返回确认抓取，不发送 `arm:5;`；
- 普通 A/B `send`下成熟正常水果不做二维码匹配；
- 最大框为坏果且 `enable_bad_fruit=False`时发送 `voice:20;`、`arm:5;`；开关为真时进入保留的坏果处理返回值。

- [ ] **Step 2: 运行并确认 RED**

Run: `python -m unittest tests.test_position_fruit_binding -v`

Expected: 当前坏果优先和 `fruit_list[self.order]`逻辑导致断言失败。

- [ ] **Step 3: 最小修改检测与判断**

`detect_box()`只在有效框索引中选择面积最大者，删除坏果抢占逻辑。`yuyin()`保持先播报后判断：坏果关闭、未成熟、带位置类别不匹配均统一发送 `arm:5;`；正确成熟目标进入现有对准。移除 `order`对抓取判断和递增的影响，任务完成/跳过时清理当前位置上下文。

- [ ] **Step 4: 运行 K230回归测试**

Run:

```powershell
python -m unittest tests.test_position_fruit_binding -v
python -m unittest tests.test_y_axis_toggle -v
python -m py_compile main.py
```

Expected: 单元测试和语法检查全部通过；不声称 CanMV摄像头或KModel已验证。

---

### Task 5: 两端最终验证与提交

**Files:**
- Verify: STM32本次修改和测试
- Verify: K230 `main.py`与测试

**Interfaces:**
- Consumes: Task 1～4的全部结果。
- Produces: 两个仓库中各自可审查的提交和硬件验证清单。

- [ ] **Step 1: STM32最终验证**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_uppercp_send_task_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_app_route_c_position_test.ps1
powershell -ExecutionPolicy Bypass -File tests/run_app_route_b_test.ps1
powershell -ExecutionPolicy Bypass -File tests/gimbal_before_lift_contract.ps1
git diff --check
```

Expected: 四项测试及差异检查通过。不得暂存用户现有 `MDK-ARM/vet6_mdk/app.__i`删除状态。

- [ ] **Step 2: K230最终验证**

在 `D:\codexproject\zhihuinongye\dayun\model`运行：

```powershell
python -m unittest tests.test_position_fruit_binding -v
python -m unittest tests.test_y_axis_toggle -v
python -m py_compile main.py
git diff --check
```

Expected: 全部通过，且只修改指定 `main.py`和本次测试。

- [ ] **Step 3: 分仓库提交**

STM32仓库提交 `app.h/app.c/UpperCP.c`、测试和本计划，提交信息：

```text
feat: send C zone fruit positions
```

K230仓库在 `codex/c-zone-fruit-binding`分支提交 `main.py`和测试，提交信息：

```text
feat: bind QR fruits to positions
```

- [ ] **Step 4: 交付硬件检查**

要求用户用 Keil编译 `MDK-ARM/vet6_mdk.uvprojx`并返回完整日志；CanMV板端依次验证二维码映射、`send:<位置>`收发、最大框选择、错误水果播报后跳过、坏果跳过和正确成熟水果抓取。

# HUD 曲率置信度指示条 复刻文档

## 功能目标
在 onroad HUD 底部显示一条弧形指示条：
- angleState 车辆（如 Ford）显示 **曲率置信度 + 曲率强度**。
- torque 车辆保留原 **扭矩条** 行为。
- 默认显示，无开关。

## 涉及文件与路径
- `selfdrive/ui/mici/onroad/torque_bar.py`
  - 曲率置信度/扭矩逻辑与绘制
- `selfdrive/ui/onroad/hud_renderer.py`
  - C3 HUD 渲染入口
- `selfdrive/ui/mici/onroad/hud_renderer.py`
  - 小屏 HUD 渲染入口
- `selfdrive/controls/lib/drive_helpers.py`
  - `MAX_CURVATURE = 0.2`（置信度版/曲率上限参考）

## 数据来源（UI SubMaster）
来自 `ui_state.sm`：
- `controlsState.curvature`
- `controlsState.desiredCurvature`
- `controlsState.lateralControlState.which()`
- `carState.vEgo`
- `liveParameters.roll`
- `carOutput.actuatorsOutput.torque`（非 angleState）
- （可选，置信度版）`modelV2.laneLineProbs`、`modelV2.roadEdgeStds`

## 当前实现（torque_bar.py）
angleState 已接入 **曲率置信度 + 曲率强度**：
- `_torque_filter` = `sign(desiredCurvature) * confidence`
- `bar_mag` = `abs(desiredCurvature) / curv_limit`（夹取 0..1）
- 颜色分级（绿/黄/红）与强度联动透明度

## 渲染入口
两个 HUD 都直接渲染，不再做 angleState 前置判断：
- `selfdrive/ui/onroad/hud_renderer.py`
- `selfdrive/ui/mici/onroad/hud_renderer.py`

核心调用：
```
self._torque_bar.render(rect)
```

## 模式选择逻辑
文件：`selfdrive/ui/mici/onroad/torque_bar.py`
```
controls_state = ui_state.sm["controlsState"]
if controls_state.lateralControlState.which() == "angleState":
  # angleState 模式（当前基线 / 置信度版）
else:
  # 原扭矩模式
```

## 曲率置信度模式（angleState）
### 1) 方向
```
desired_curv = controlsState.desiredCurvature
curv_sign = +1 if desired_curv >= 0 else -1
```
`_torque_filter` 用 `curv_sign * confidence` 控制左右方向。

### 2) 置信度计算
```
lane_conf = min(laneLineProbs[1], laneLineProbs[2])
edge_conf = clip(1 - max(roadEdgeStds), 0..1)
confidence = clip(0.7 * lane_conf + 0.3 * edge_conf, 0..1)
```
数组缺失时回退为 `0.0`。

### 3) 曲率强度
```
v_ego = max(carState.vEgo, 1.0)
max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + liveParameters.roll * g
curv_limit = min(MAX_CURVATURE, max_lat_accel / v_ego^2)
curv_ratio = abs(desired_curv) / max(curv_limit, 1e-6)
bar_mag = clip(curv_ratio, 0..1)
```

### 4) 滤波
```
_torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
_curve_intensity_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
```
`_curve_intensity_filter` 为增强平滑可选项，当前代码基线未包含。

### 5) 颜色分级
```
conf >= 0.7 -> 绿色 (0,255,120)
conf >= 0.4 -> 黄色 (255,200,0)
否则       -> 红色 (255,80,80)
```
透明度与强度关联：
```
alpha_scale = (0.35 + 0.65 * bar_mag) * alpha_filter
```

## 原扭矩模式（非 angleState）
保持原逻辑：
```
torque = -carOutput.actuatorsOutput.torque
```
颜色保持白色到橙色的渐变。

## 几何与绘制
文件：`selfdrive/ui/mici/onroad/torque_bar.py`

核心常量：
```
TORQUE_ANGLE_SPAN = 12.7
torque_line_radius = 1200 * scale
top_angle = -90
```

位置：
```
cx = rect.x + rect.width / 2 + (8 * scale)
cy = rect.y + rect.height + torque_line_radius - torque_line_offset
```

弧形绘制：
```
arc_bar_pts(...) -> draw_polygon(...)
```

强度映射（高度/偏移）：
```
bar_mag = abs(_torque_filter.x)  # 当前基线；置信度版可用曲率强度 bar_mag
torque_line_offset = interp(bar_mag, [0.5, 1], [22*scale, 26*scale])
torque_line_height = interp(bar_mag, [0.5, 1], [14*scale, 56*scale])
```

## 尺寸缩放修复（适配 C3 大屏）
之前尺寸偏小是因为 TorqueBar 采用固定像素，未随大屏缩放。

已加入逻辑：
```
BASE_UI_WIDTH = 536
BASE_UI_HEIGHT = 240
scale = self._scale * min(rect.width / BASE_UI_WIDTH, rect.height / BASE_UI_HEIGHT)
```
在 C3 2160x1080 上，`scale` 约为 4.0，尺寸与大屏匹配。

## 默认显示与可见性
- 无额外 UI 开关。
- 透明度由 `_torque_line_alpha_filter` 控制：
```
alpha_filter = (ui_state.status != DISENGAGED) or ui_state.dp_alka_active
```

## 复刻步骤（给其他开发者/AI）
1) 在 `selfdrive/ui/mici/onroad/torque_bar.py` 实现 angleState 曲率置信度逻辑：
   - `laneLineProbs` + `roadEdgeStds` 计算置信度
   - 曲率强度 `abs(desiredCurvature) / curv_limit`
   - 颜色分级（绿/黄/红）
2) 在 `selfdrive/ui/onroad/hud_renderer.py` 和 `selfdrive/ui/mici/onroad/hud_renderer.py`
   - 始终渲染 `TorqueBar`。
3) 加入尺寸自适应缩放（按 `rect` 计算 scale）。
4) 确保 `ui_state.sm` 包含 `modelV2`, `controlsState`, `carState`, `liveParameters`。

## 快速验证清单
- Ford（angleState）曲率置信度条可见，颜色随置信度变化。
- 大屏 C3 尺寸正常（不再偏小）。
- torque 车型保持原扭矩条行为。

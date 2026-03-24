# rmdev_ins

`rmdev` 姿态解算模块。

## 依赖

- `emdevif`（含 `emdevif_timeline`）
- `rmdev_device_model`
- `rmdev_math`
- `CMSISDSP`

## 启用方式

该模块默认不编译。需在上层 `rmdev` 配置中开启：

- `RMDEV_ENABLE_INS_MODULE=ON`

## 已集成算法

- 基于四元数扩展卡尔曼滤波的姿态更新算法( Wang
  Hongxi, https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example )

## 使用建议

- 在固定周期任务中调用更新逻辑
- 确保 `timeline` 时间基准稳定
- 将传感器原始数据先写入 `rmdev_device_model`，再进入姿态解算流程

## 注意事项

- 若未链接 `CMSISDSP`，该模块无法正常构建
- 建议与板级传感器驱动一起联调验证时间戳与坐标约定

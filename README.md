# rmdev_ins

属于 rmdev 的一个子模块，用于进行姿态解算。

## 依赖

* `emdevif`（包括 `emdevif_timeline`）
* `rmdev_device_model`
* `CMSISDSP`

## 支持的算法

该姿态解算包含一个命名规范（参考 `Ins` 类的注释），在该命名规范下，通过偏特化 `InsAlgorithm` 枚举以实现指定使用哪种算法。

1. 基于四元数扩展卡尔曼滤波的姿态更新算法( Wang
   Hongxi, https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example )

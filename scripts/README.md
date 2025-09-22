# Python相机标定程序使用说明

这是一个简单的Python相机标定程序，可以直接读取图片文件进行相机内参标定。

## 安装依赖

```bash
pip install opencv-python opencv-contrib-python numpy pyyaml
```

## 使用方法

### 1. 生成ChArUco标定板

```bash
cd FAST-Calib_ROS2/scripts
python camera_calibration.py --generate_board
```

这会生成一个名为 `charuco_board.png` 的标定板图像，打印出来使用。

### 2. 收集标定图像

- 将生成的标定板打印在A4纸上（或更大）
- 用相机拍摄标定板的不同角度和位置的照片
- 建议拍摄20-50张照片，包括：
  - 标定板在图像中心
  - 标定板在图像四个角落
  - 标定板倾斜不同角度
  - 标定板距离相机不同距离
- 将所有照片放在一个文件夹中

### 3. 执行标定

```bash
# 基本用法
python camera_calibration.py --images /path/to/your/images

# 指定输出文件
python camera_calibration.py --images /path/to/your/images --output my_calibration.yaml

# 指定图像格式
python camera_calibration.py --images /path/to/your/images --pattern "*.png"

# 自定义标定板参数
python camera_calibration.py --images /path/to/your/images \
    --board_width 7 --board_height 5 \
    --square_size 0.04 --marker_size 0.02
```

### 4. 查看结果

程序会在控制台输出标定结果，并生成两个文件：
- `camera_calibration.yaml`: 完整的标定结果
- `camera_calibration_simple.yaml`: 简化版本，可直接复制到qr_params.yaml

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| --images | 必需 | 包含标定图像的文件夹路径 |
| --output | camera_calibration.yaml | 输出文件名 |
| --board_width | 7 | 标定板宽度方向的方格数量 |
| --board_height | 5 | 标定板高度方向的方格数量 |
| --square_size | 0.04 | 方格边长 (米) |
| --marker_size | 0.02 | ArUco标记边长 (米) |
| --pattern | *.jpg | 图像文件模式 |
| --generate_board | - | 生成标定板图像 |

## 示例

假设你的标定图像在 `calibration_images` 文件夹中：

```bash
cd FAST-Calib_ROS2/scripts

# 1. 生成标定板
python camera_calibration.py --generate_board

# 2. 执行标定
python camera_calibration.py --images calibration_images

# 3. 查看结果
cat camera_calibration_simple.yaml
```

然后将输出的参数复制到 `qr_params.yaml` 文件中。

## 注意事项

1. **图像质量**: 确保图像清晰，标定板完全可见
2. **光照条件**: 避免过强或过弱的光照，减少反光
3. **标定板平整**: 确保标定板平整，没有弯曲
4. **角度多样性**: 拍摄不同角度和位置的照片
5. **数量充足**: 至少需要10张有效图像，建议20-50张

## 故障排除

### 问题1: 检测不到角点
- 检查标定板是否完全在图像中
- 改善光照条件
- 确保标定板平整清晰

### 问题2: 有效图像数量不足
- 增加拍摄图像数量
- 确保标定板在每张图像中都清晰可见
- 尝试不同的拍摄角度

### 问题3: RMS误差过大
- 检查标定板的实际尺寸是否与参数一致
- 增加高质量的标定图像
- 确保相机焦距在拍摄过程中保持不变
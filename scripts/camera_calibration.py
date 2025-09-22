#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import glob
import os
import yaml
import argparse
from pathlib import Path

class CameraCalibration:
    def __init__(self, board_size=(5, 4), square_size=0.05, marker_size=0.037, show_detection=False):
        """
        初始化相机标定类
        
        Args:
            board_size: ChArUco板的方格数量 (width, height)
            square_size: 方格边长 (米)
            marker_size: ArUco标记边长 (米)
            show_detection: 是否显示检测结果窗口
        """
        self.board_size = board_size
        self.square_size = square_size
        self.marker_size = marker_size
        self.show_detection = show_detection
        
        # 创建ChArUco标定板 - OpenCV 4.7+
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.CharucoBoard(
            board_size, square_size, marker_size, self.dictionary
        )

        # 配置检测参数
        self.detector_params = cv2.aruco.DetectorParameters()
        # 提高检测精度的参数设置
        self.detector_params.adaptiveThreshWinSizeMin = 3
        self.detector_params.adaptiveThreshWinSizeMax = 23
        self.detector_params.adaptiveThreshWinSizeStep = 10
        self.detector_params.adaptiveThreshConstant = 7
        self.detector_params.minMarkerPerimeterRate = 0.03
        self.detector_params.maxMarkerPerimeterRate = 4.0
        self.detector_params.polygonalApproxAccuracyRate = 0.03
        self.detector_params.minCornerDistanceRate = 0.05
        self.detector_params.minDistanceToBorder = 3
        self.detector_params.minMarkerDistanceRate = 0.05
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_params.cornerRefinementWinSize = 5
        self.detector_params.cornerRefinementMaxIterations = 30
        self.detector_params.cornerRefinementMinAccuracy = 0.1
        
        # 配置ChArUco参数
        self.charuco_params = cv2.aruco.CharucoParameters()
        self.charuco_params.cameraMatrix = None  # 如果有相机内参可以设置
        self.charuco_params.distCoeffs = None    # 如果有畸变系数可以设置
        self.charuco_params.minMarkers = 2       # 最少需要的标记数量
        self.charuco_params.tryRefineMarkers = True  # 尝试细化标记
        
        # 配置细化参数
        self.refine_params = cv2.aruco.RefineParameters()
        self.refine_params.minRepDistance = 10.0
        self.refine_params.errorCorrectionRate = 3.0
        self.refine_params.checkAllOrders = True

        # 存储所有图像的角点
        self.all_charuco_corners = []
        self.all_charuco_ids = []
        self.image_size = None

    def detect_corners(self, image, image_name=""):
        """
        检测单张图像中的ChArUco角点
        
        Args:
            image: 输入图像
            image_name: 图像名称（用于显示）
            
        Returns:
            corners: 检测到的角点
            ids: 角点ID
            success: 是否成功检测
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 检测ArUco标记
        detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
        
        # 创建可视化图像
        vis_image = image.copy()
        
        # 详细的调试信息
        debug_info = []
        debug_info.append(f"Image: {image_name}")
        debug_info.append(f"Image size: {image.shape[:2]}")
        debug_info.append(f"Board config: {self.board_size[0]}x{self.board_size[1]}")
        debug_info.append(f"Square size: {self.square_size}m")
        debug_info.append(f"Marker size: {self.marker_size}m")
        debug_info.append(f"Dictionary: DICT_4X4_50")
        
        # 绘制检测到的ArUco标记
        if marker_ids is not None and len(marker_corners) > 0:
            cv2.aruco.drawDetectedMarkers(vis_image, marker_corners, marker_ids)
            debug_info.append(f"ArUco markers detected: {len(marker_corners)}")
            debug_info.append(f"Marker IDs: {marker_ids.flatten().tolist()}")
            
            # 分析标记ID范围
            expected_max_id = self.board_size[0] * self.board_size[1] - 1
            actual_max_id = marker_ids.max() if len(marker_ids) > 0 else -1
            debug_info.append(f"Expected max ID: {expected_max_id}, Actual max ID: {actual_max_id}")
            
            # 检查标记ID是否超出预期范围
            if actual_max_id > expected_max_id:
                debug_info.append("WARNING: Marker IDs exceed expected range!")
                debug_info.append("This suggests board size mismatch!")
            
            # 检测ChArUco角点
            charuco_detector = cv2.aruco.CharucoDetector(
                self.board, 
                self.charuco_params, 
                self.detector_params, 
                self.refine_params
            )
            charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)
            ret = len(charuco_corners) if charuco_corners is not None else 0
            
            debug_info.append(f"ChArUco corners detected: {ret}")
            
            # 绘制ChArUco角点
            if ret > 0:
                cv2.aruco.drawDetectedCornersCharuco(vis_image, charuco_corners, charuco_ids)
                debug_info.append(f"ChArUco corner IDs: {charuco_ids.flatten().tolist() if charuco_ids is not None else 'None'}")
            else:
                debug_info.append("ChArUco detection FAILED - Possible reasons:")
                debug_info.append("1. Board size mismatch")
                debug_info.append("2. Square/marker size mismatch")
                debug_info.append("3. Dictionary type mismatch")
                debug_info.append("4. Insufficient marker visibility")
                debug_info.append("5. Image quality issues")
                
                # 额外的诊断信息
                unique_ids = len(set(marker_ids.flatten())) if marker_ids is not None else 0
                debug_info.append(f"Unique marker count: {unique_ids}")
                
                # 检查标记分布
                if len(marker_corners) >= 4:
                    # 计算标记间距离，用于验证尺寸设置
                    corners_array = np.array([corner[0] for corner in marker_corners])
                    if len(corners_array) >= 2:
                        # 计算相邻标记的平均距离
                        distances = []
                        for i in range(len(corners_array)):
                            for j in range(i+1, len(corners_array)):
                                dist = np.linalg.norm(corners_array[i].mean(axis=0) - corners_array[j].mean(axis=0))
                                distances.append(dist)
                        if distances:
                            avg_distance = np.mean(distances)
                            debug_info.append(f"Average marker distance (pixels): {avg_distance:.1f}")
                            
                            # 估算实际的方格大小（像素）
                            estimated_square_pixels = avg_distance * 0.7  # 粗略估算
                            debug_info.append(f"Estimated square size (pixels): {estimated_square_pixels:.1f}")
        else:
            marker_corners, marker_ids = [], None
            charuco_corners, charuco_ids = None, None
            ret = 0
            debug_info.append("No ArUco markers detected!")
        
        # 添加状态信息到图像上
        status_text = []
        status_text.append(f"Image: {image_name}")
        status_text.append(f"ArUco Markers: {len(marker_corners) if marker_corners else 0}")
        status_text.append(f"ChArUco Corners: {ret}")
        status_text.append(f"Status: {'SUCCESS' if ret >= 4 else 'FAILED'}")
        
        # 在图像上绘制状态信息
        y_offset = 30
        for i, text in enumerate(status_text):
            color = (0, 255, 0) if ret >= 4 else (0, 0, 255)  # 绿色表示成功，红色表示失败
            cv2.putText(vis_image, text, (10, y_offset + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # 打印详细调试信息
        if ret < 4:
            print("  === 详细调试信息 ===")
            for info in debug_info:
                print(f"  {info}")
            print("  ==================")
        
        # 如果启用了可视化或检测失败，显示结果
        if self.show_detection or ret < 4:
            self._show_detection_result(vis_image, image_name, ret >= 4)
        
        if ret < 4:  # 至少需要4个角点
            return None, None, False
            
        return charuco_corners, charuco_ids, True
    
    def _show_detection_result(self, vis_image, image_name, success):
        """
        显示检测结果窗口
        
        Args:
            vis_image: 可视化图像
            image_name: 图像名称
            success: 检测是否成功
        """
        # 调整图像大小以适应屏幕
        height, width = vis_image.shape[:2]
        max_height = 800
        if height > max_height:
            scale = max_height / height
            new_width = int(width * scale)
            vis_image = cv2.resize(vis_image, (new_width, max_height))
        
        window_name = f"Detection Result - {image_name}"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, vis_image)
        
        if not success:
            print(f"  检测失败 - 显示结果窗口: {window_name}")
            print("  按任意键继续处理下一张图像，按 'q' 退出，按 's' 跳过显示后续失败图像")
            
            key = cv2.waitKey(0) & 0xFF
            cv2.destroyWindow(window_name)
            
            if key == ord('q'):
                print("  用户选择退出")
                cv2.destroyAllWindows()
                exit(0)
            elif key == ord('s'):
                print("  跳过显示后续失败图像")
                self.show_detection = False
        else:
            print(f"  检测成功 - 显示结果窗口: {window_name}")
            cv2.waitKey(1000)  # 显示1秒
            cv2.destroyWindow(window_name)

    def process_images(self, image_folder, image_pattern="*.jpg"):
        """
        处理文件夹中的所有图像
        
        Args:
            image_folder: 图像文件夹路径
            image_pattern: 图像文件模式
        """
        image_files = glob.glob(os.path.join(image_folder, image_pattern))
        
        if not image_files:
            # 尝试其他常见格式
            for pattern in ["*.png", "*.jpeg", "*.bmp"]:
                image_files.extend(glob.glob(os.path.join(image_folder, pattern)))
        
        if not image_files:
            raise ValueError(f"在文件夹 {image_folder} 中没有找到图像文件")
        
        print(f"找到 {len(image_files)} 张图像")
        
        valid_images = 0
        
        for i, image_file in enumerate(image_files):
            image_name = os.path.basename(image_file)
            print(f"处理图像 {i+1}/{len(image_files)}: {image_name}")
            
            # 读取图像
            image = cv2.imread(image_file)
            if image is None:
                print(f"  警告: 无法读取图像 {image_file}")
                continue
            
            # 设置图像尺寸
            if self.image_size is None:
                self.image_size = (image.shape[1], image.shape[0])
            
            # 检测角点
            corners, ids, success = self.detect_corners(image, image_name)
            
            if success:
                self.all_charuco_corners.append(corners)
                self.all_charuco_ids.append(ids)
                valid_images += 1
                print(f"  成功检测到 {len(corners)} 个角点")
            else:
                corner_count = 0 if corners is None else len(corners)
                print(f"  警告: 未能检测到足够的角点, 检测到 {corner_count} 个角点")
        
        print(f"\n成功处理 {valid_images} 张图像")
        
        if valid_images < 10:
            print("警告: 有效图像数量较少，可能影响标定精度")
        
        # 关闭所有窗口
        cv2.destroyAllWindows()
        
        return valid_images

    def calibrate(self):
        """
        执行相机标定
        
        Returns:
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数
            rms_error: RMS重投影误差
        """
        if len(self.all_charuco_corners) < 10:
            raise ValueError("有效图像数量不足，至少需要10张图像进行标定")
        
        print("开始相机标定...")
        
        # 准备标定数据
        all_object_points = []
        all_image_points = []
        
        for i in range(len(self.all_charuco_corners)):
            charuco_corners = self.all_charuco_corners[i]
            charuco_ids = self.all_charuco_ids[i]
            
            # 使用board.matchImagePoints获取对应的3D-2D点对
            # 注意：matchImagePoints返回的是Mat对象，需要正确处理
            object_points, image_points = self.board.matchImagePoints(charuco_corners, charuco_ids)
            
            if object_points is not None and image_points is not None and len(object_points) > 0:
                # 转换为numpy数组
                obj_pts = np.array(object_points, dtype=np.float32).reshape(-1, 3)
                img_pts = np.array(image_points, dtype=np.float32).reshape(-1, 2)
                
                all_object_points.append(obj_pts)
                all_image_points.append(img_pts)
        
        if len(all_object_points) < 10:
            raise ValueError("有效的3D-2D点对数量不足，至少需要10组")
        
        # 使用cv2.calibrateCamera进行标定
        rms_error, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            all_object_points,
            all_image_points,
            self.image_size,
            None,
            None
        )
        
        print(f"标定完成! RMS误差: {rms_error:.3f} 像素")
        
        return camera_matrix, dist_coeffs, rms_error
    
    def print_results(self, camera_matrix, dist_coeffs, rms_error):
        """
        打印标定结果
        """
        print("\n" + "="*50)
        print("相机标定结果")
        print("="*50)
        print(f"RMS重投影误差: {rms_error:.3f} 像素")
        print(f"图像尺寸: {self.image_size[0]} x {self.image_size[1]}")
        print(f"有效图像数量: {len(self.all_charuco_corners)}")
        
        print("\n相机内参矩阵:")
        print(camera_matrix)
        
        print("\n畸变系数:")
        print(dist_coeffs.flatten())
        
        # 提取参数
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        k1, k2, p1, p2 = dist_coeffs.flatten()[:4]
        
        print("\n" + "="*50)
        print("用于 qr_params.yaml 的参数:")
        print("="*50)
        print(f"fx: {fx:.11f}")
        print(f"fy: {fy:.11f}")
        print(f"cx: {cx:.11f}")
        print(f"cy: {cy:.11f}")
        print(f"k1: {k1:.11f}")
        print(f"k2: {k2:.11f}")
        print(f"p1: {p1:.11f}")
        print(f"p2: {p2:.11f}")
        print("="*50)
        
        return {
            'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
            'k1': k1, 'k2': k2, 'p1': p1, 'p2': p2
        }
    
    def save_results(self, camera_matrix, dist_coeffs, rms_error, output_file):
        """
        保存标定结果到YAML文件
        """
        # 提取参数
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        k1, k2, p1, p2 = dist_coeffs.flatten()[:4]
        
        # 创建完整的输出数据
        output_data = {
            'calibration_info': {
                'date': str(np.datetime64('now')),
                'opencv_version': cv2.__version__,
                'rms_error': float(rms_error),
                'image_count': len(self.all_charuco_corners),
                'image_size': [int(self.image_size[0]), int(self.image_size[1])],
                'board_size': [int(self.board_size[0]), int(self.board_size[1])],
                'square_size': float(self.square_size),
                'marker_size': float(self.marker_size)
            },
            'camera_matrix': camera_matrix.tolist(),
            'distortion_coefficients': dist_coeffs.flatten().tolist(),
            'camera_parameters': {
                'fx': float(fx),
                'fy': float(fy),
                'cx': float(cx),
                'cy': float(cy),
                'k1': float(k1),
                'k2': float(k2),
                'p1': float(p1),
                'p2': float(p2)
            }
        }
        
        with open(output_file, 'w', encoding='utf-8') as f:
            yaml.dump(output_data, f, default_flow_style=False, allow_unicode=True)
        
        print(f"\n标定结果已保存到: {output_file}")
        
        # 同时保存一个简化版本，方便直接复制到qr_params.yaml
        simple_output = output_file.replace('.yaml', '_simple.yaml')
        simple_data = {
            'fast_calib': {
                'ros__parameters': {
                    'fx': float(fx),
                    'fy': float(fy),
                    'cx': float(cx),
                    'cy': float(cy),
                    'k1': float(k1),
                    'k2': float(k2),
                    'p1': float(p1),
                    'p2': float(p2)
                }
            }
        }
        
        with open(simple_output, 'w', encoding='utf-8') as f:
            yaml.dump(simple_data, f, default_flow_style=False)
        
        print(f"简化参数已保存到: {simple_output}")

    def verify_board_parameters(self, image_folder):
        """
        验证标定板参数是否正确
        
        Args:
            image_folder: 图像文件夹路径
        """
        print("\n=== 标定板参数验证 ===")
        print(f"当前配置:")
        print(f"  标定板尺寸: {self.board_size[0]}x{self.board_size[1]}")
        print(f"  方格边长: {self.square_size}m ({self.square_size*1000}mm)")
        print(f"  标记边长: {self.marker_size}m ({self.marker_size*1000}mm)")
        print(f"  字典类型: DICT_4X4_50")
        print(f"  预期标记ID范围: 0-{self.board_size[0] * self.board_size[1] - 1}")
        
        # 分析第一张图像
        image_files = glob.glob(os.path.join(image_folder, "*.jpg"))
        if not image_files:
            image_files = glob.glob(os.path.join(image_folder, "*.png"))
        
        if image_files:
            print(f"\n分析第一张图像: {os.path.basename(image_files[0])}")
            image = cv2.imread(image_files[0])
            if image is not None:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                # 检测ArUco标记
                detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
                marker_corners, marker_ids, _ = detector.detectMarkers(gray)
                
                if marker_ids is not None and len(marker_corners) > 0:
                    print(f"  检测到 {len(marker_corners)} 个ArUco标记")
                    print(f"  标记ID: {sorted(marker_ids.flatten().tolist())}")
                    
                    max_id = marker_ids.max()
                    expected_max = self.board_size[0] * self.board_size[1] - 1
                    
                    if max_id > expected_max:
                        print(f"  ❌ 警告: 最大标记ID ({max_id}) 超出预期范围 (0-{expected_max})")
                        print(f"     建议的标定板尺寸: 至少 {int(np.sqrt(max_id + 1)) + 1}x{int(np.sqrt(max_id + 1)) + 1}")
                    else:
                        print(f"  ✅ 标记ID范围正常 (0-{max_id})")
                    
                    # 分析标记间距
                    if len(marker_corners) >= 2:
                        corners_array = np.array([corner[0] for corner in marker_corners])
                        distances = []
                        for i in range(len(corners_array)):
                            for j in range(i+1, len(corners_array)):
                                center_i = corners_array[i].mean(axis=0)
                                center_j = corners_array[j].mean(axis=0)
                                dist = np.linalg.norm(center_i - center_j)
                                distances.append(dist)
                        
                        if distances:
                            min_dist = min(distances)
                            avg_dist = np.mean(distances)
                            print(f"  标记间距离 (像素): 最小={min_dist:.1f}, 平均={avg_dist:.1f}")
                else:
                    print("  ❌ 未检测到任何ArUco标记")
        
        print("=====================\n")

def generate_charuco_board(output_file="charuco_board.png", board_size=(5, 4), 
                          square_size=0.05, marker_size=0.037, image_size=(2000, 1400)):
    """
    生成ChArUco标定板图像
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard(board_size, square_size, marker_size, dictionary)
    img = board.generateImage(image_size)
    
    cv2.imwrite(output_file, img)
    print(f"ChArUco标定板已生成: {output_file}")
    print(f"标定板参数: {board_size[0]}x{board_size[1]} 方格")
    print(f"方格边长: {square_size}m, 标记边长: {marker_size}m")

def main():
    parser = argparse.ArgumentParser(description='相机标定程序')
    parser.add_argument('--images', '-i', 
                       help='包含标定图像的文件夹路径')
    parser.add_argument('--output', '-o', default='camera_calibration.yaml',
                       help='输出文件名')
    parser.add_argument('--board_width', default=4, type=int,
                       help='标定板宽度方向的方格数量')
    parser.add_argument('--board_height', default=5, type=int,
                       help='标定板高度方向的方格数量')
    parser.add_argument('--square_size', default=0.05, type=float,
                       help='方格边长 (米)')
    parser.add_argument('--marker_size', default=0.037, type=float,
                       help='ArUco标记边长 (米)')
    parser.add_argument('--pattern', default='*.jpg',
                       help='图像文件模式')
    parser.add_argument('--generate_board', action='store_true',
                       help='生成ChArUco标定板图像')
    parser.add_argument('--show_detection', action='store_true',
                       help='显示所有检测结果窗口（包括成功的）')
    parser.add_argument('--verify_params', action='store_true',
                       help='验证标定板参数是否与图像匹配')
    
    args = parser.parse_args()
    
    # 打印OpenCV版本信息
    print(f"OpenCV版本: {cv2.__version__}")
    
    # 如果需要生成标定板
    if args.generate_board:
        generate_charuco_board(
            board_size=(args.board_width, args.board_height),
            square_size=args.square_size,
            marker_size=args.marker_size
        )
        return
    
    # 检查是否提供了图像文件夹
    if not args.images:
        print("错误: 请提供图像文件夹路径 (使用 --images 参数)")
        print("或者使用 --generate_board 生成标定板")
        return
    
    # 检查图像文件夹
    if not os.path.exists(args.images):
        print(f"错误: 图像文件夹不存在: {args.images}")
        return
    
    try:
        # 创建标定对象
        calibrator = CameraCalibration(
            board_size=(args.board_width, args.board_height),
            square_size=args.square_size,
            marker_size=args.marker_size,
            show_detection=args.show_detection
        )
        
        # 如果需要验证参数
        if args.verify_params:
            calibrator.verify_board_parameters(args.images)
            return
        
        # 处理图像
        valid_count = calibrator.process_images(args.images, args.pattern)
        
        if valid_count < 10:
            print("错误: 有效图像数量不足，无法进行标定")
            return
        
        # 执行标定
        camera_matrix, dist_coeffs, rms_error = calibrator.calibrate()
        
        # 保存结果
        calibrator.save_results(camera_matrix, dist_coeffs, rms_error, args.output)
        
        print("\n标定完成!")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
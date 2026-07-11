import cv2
import numpy as np

def detect_red_blocks():
    # 打开默认摄像头 (索引为 0)
    # 如果你有外接摄像头，可能需要更改为 1 或 2
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("错误：无法打开摄像头。")
        return

    print("摄像头已打开。按 'q' 键退出程序。")

    while True:
        # 读取一帧图像
        ret, frame = cap.read()
        
        if not ret:
            print("错误：无法读取画面帧。")
            break

        # 为了避免镜像操作带来的不适，将图像水平翻转
        frame = cv2.flip(frame, 1)

        # 将图像从 BGR (蓝绿红) 颜色空间转换到 HSV (色相、饱和度、明度) 颜色空间
        # HSV 空间更适合进行颜色提取
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定义更严格的红色 HSV 阈值范围
        # 提高饱和度(S)和明度(V)的下限，收窄色相(H)范围，过滤不纯正的红
        
        # 红色范围 1 (0-8)
        lower_red_1 = np.array([0, 150, 100])
        upper_red_1 = np.array([8, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)

        # 红色范围 2 (172-180)
        lower_red_2 = np.array([172, 150, 100])
        upper_red_2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

        # 将两个掩膜(mask)相加，得到完整的红色区域掩膜
        mask = mask1 + mask2

        # 形态学操作：开运算 (先腐蚀后膨胀)
        # 使用更大的 7x7 卷积核，更彻底地去除画面中的细小噪点
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 形态学操作：膨胀
        # 让识别到的红色区域更加饱满连贯
        mask = cv2.dilate(mask, kernel, iterations=1)

        # 寻找轮廓
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 遍历所有找到的轮廓
        for cnt in contours:
            # 计算轮廓的面积
            area = cv2.contourArea(cnt)
            
            # 提高面积阈值到 1000，过滤掉更小的红色干扰
            if area > 150:
                # 获取轮廓的边界框坐标
                x, y, w, h = cv2.boundingRect(cnt)
                
                # 在原图上绘制绿色边界框，线宽为 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # 计算中心点坐标并画一个小圆点
                center_x = x + w // 2
                center_y = y + h // 2
                cv2.circle(frame, (center_x, center_y), 3, (255, 0, 0), -1)
                
                # 在边界框上方添加文字标签和中心点坐标
                text = f"Red Block (X:{center_x}, Y:{center_y})"
                cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 打印中心点坐标到终端
                print(f"检测到红色物块，中心坐标: X={center_x}, Y={center_y}")

        # 显示处理后的画面
        cv2.imshow("Red Block Detection", frame)

        # 检测按键，如果按下 'q' 键则跳出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头资源并关闭所有窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_red_blocks()
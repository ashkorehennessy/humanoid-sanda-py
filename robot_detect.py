

def robot_detect(image):
    # 转换颜色空间
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 设定颜色范围（在HSV颜色空间中）
    lower_orange = np.array([14, 65, 190])  # 橙色的最低HSV值
    upper_orange = np.array([35, 255, 255])  # 橙色的最高HSV值

    orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

    # 查找橙色连接件的轮廓
    contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 计算每个轮廓的外接矩形和中心点
    bounding_boxes = [cv2.boundingRect(contour) for contour in contours]
    center_points = [(box[0] + box[2] // 2, box[1] + box[3] // 2) for box in bounding_boxes]

    # 检查是否有轮廓
    if len(center_points) == 0:
        return None

    # 第一次筛选
    # 计算所有中心点的平均坐标
    avg_x = sum(point[0] for point in center_points) // len(center_points)
    avg_y = sum(point[1] for point in center_points) // len(center_points)

    # 筛选掉距离平均坐标较远的
    filtered_bounding_boxes = []
    filtered_center_points = []
    distance_threshold = 100

    for box, center in zip(bounding_boxes, center_points):
        distance = np.sqrt((center[0] - avg_x) ** 2 + (center[1] - avg_y) ** 2)
        if distance <= distance_threshold:
            filtered_bounding_boxes.append(box)
            filtered_center_points.append(center)

    if len(filtered_center_points) == 0:
        return None

    # 第二次筛选
    # 计算第一次筛选的中心点的平均坐标
    avg_x = sum(point[0] for point in filtered_center_points) // len(filtered_center_points)
    avg_y = sum(point[1] for point in filtered_center_points) // len(filtered_center_points)

    # 筛选掉距离平均坐标较远的小球
    filtered_bounding_boxes = []
    filtered_center_points = []

    distance_threshold = 150
    for box, center in zip(bounding_boxes, center_points):
        distance = np.sqrt((center[0] - avg_x) ** 2 + (center[1] - avg_y) ** 2)
        if distance <= distance_threshold:
            filtered_bounding_boxes.append(box)
            filtered_center_points.append(center)

    # 计算大矩形的位置
    if len(filtered_bounding_boxes) > 0:
        x_min = min(box[0] for box in filtered_bounding_boxes)
        y_min = min(box[1] for box in filtered_bounding_boxes)
        x_max = max(box[0] + box[2] for box in filtered_bounding_boxes)
        y_max = max(box[1] + box[3] for box in filtered_bounding_boxes)

        area = (x_max - x_min) * (y_max - y_min)

        print("area:", area, end="")

        if 3000 < area < 100000:
            return (x_min + x_max) // 2
        else:
            return None
    else:
        return None



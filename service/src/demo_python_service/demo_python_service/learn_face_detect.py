import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory #获取功能包 share 的绝对路径
import os

def main():
    # 获取资源文件路径
    image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource', 'default.jpg')
    # 加载图片
    image = cv2.imread(image_path)
    # 检测人脸
    face_locations = face_recognition.face_locations(image, number_of_times_to_upsample=1, model="hog")
    # 绘制人脸
    for face_location in face_locations:
        top, right, bottom, left = face_location
        cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 4)
    # 显示图片
    cv2.imshow('image', image)
    # 等待按键
    cv2.waitKey(0)
    # 关闭窗口
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


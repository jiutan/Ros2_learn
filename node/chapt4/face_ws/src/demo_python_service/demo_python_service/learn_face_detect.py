# 安装的 face_recognitione 库
import face_recognition
# 导入 Opencv 库（常用的 视觉 库）
import cv2
# 获取 功能包share目录 的 绝对路径： /share/功能包名字
from ament_index_python.packages import get_package_share_directory
# 使用 OS库 连接 路径（可以避免 路径错误）
import os 


def  main():
    # 获取 图片的 真实路径： 功能包 路径 / resource / default.jgp
    # 连接路径 的 函数：os.path.join()
    # 参数： 多个 待拼接 的 路径
    default_image_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/default.jpg')
    # get_package_share_directory('demo_python_service')+ 'resource/default.jpg'

    print(f'图片的真实路径：{default_image_path}') 

    # 使用 CV2 来 加载 图片（imread） 存放至 image 对象
    image = cv2.imread(default_image_path)

    # 在 图片中 查找 人脸
    # 函数：face_recognition.face_locations
    # 参数： 1. 图片对象    2.上采样次数：默认为1       3.识别人脸的模型：默认，model='hog'
    # 返回值：上下左右 坐标值
    face_location = face_recognition.face_locations(image,number_of_times_to_upsample=1,model='hog')

    # 来绘制 人脸识别框
    for top,right,bottom,left in face_location:
        # 使用 CV2 绘制 矩形框
        # 函数：cv2.rectangle()
        # 参数：1. 图像     2. left-top location       3. right-bottom location     4. color:(r,g,b)    5. line-width   
        cv2.rectangle(image,(left,top),(right,bottom),(255,0,0),4)

    # 结果显示
    # 函数：cv2.imshow()
    # 参数：1. 框的名字     2. 图形
    cv2.imshow('Face Detect Result',image)
    
    # 执行等待，按按键退出
    cv2.waitkey(0)
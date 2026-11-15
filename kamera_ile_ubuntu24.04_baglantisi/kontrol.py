#ilk 30 kare atlanacak kamera karanlık gelebilir diye.

import numpy as np
import rclpy 
from rclpy.node import node
from sensor_msgs.msg import Image
import cv2


class deneme(Node):
    def __init__(self):
        super.__init__("kontrol")
        self.n = 0
        self.create_subscription(Image,"/camera/image_raw",self.cb,10)

    def cb(self,m):
        self.n +=1
        if self.n < 30: #pozlama oturması için bekleyelim.
            return

        a = np.frombuffer(m.data,dtype=np.uint8)
        print('kodlama',m.encoding,'boyut',m.width,'x',m.height,
              'ortalama',a.mean(),'en parlak',a.max())
        ch = m.step // m.width
        img= a.reshape(m.height,m.width,ch)
        cv2.imwrite('ros_test.jpg',img[:,:,3])
        raise SystemExit


rclpy.init()
n=T()
try:
    rclpy.spin(n)
except SystemExit:
    print("hata gozden gecirin.")
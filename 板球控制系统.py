# Untitled - By: WY - 周一 7月 19 2021
import pyb, sensor, image, math, time
sensor.reset()#初始化摄像头，reset()是sensor模块里面的函数
sensor.set_framesize(sensor.QVGA)#设置图像像素大小
sensor.set_pixformat(sensor.GRAYSCALE)#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种
sensor.skip_frames(time = 2000)
sensor.set_contrast(1)
sensor.set_gainceiling(16)
huidu = (31,66,-13,16,-14,19)#LAB色彩图像阈值
QvYv_x=(-200,0,200,-200,0,200,-200,0,200)#各区域x坐标
QvYv_y=(-200,-200,-200,0,0,0,200,200,200)#各区域y坐标
CiShu=(2,6,8,4,2,6,8,4,2,6,8,4,2,6)
clock = time.clock()#创建一个时钟对象来跟踪FPS帧率
moshi=x=y=PD_x=PI_x=PD_y=PI_y=0#定义各全局变量并赋初值
B=2
C=5
D=7
def TiQvZuoBiao(img):#提取小球相对平衡板的坐标
    global x,y#在函数内部声明变量x,y的作用域是全局
    img.binary([(31,66,-13,16,-14,19)])#LAB阈值二值化
    img.erode(1)#对图像白色边缘执行腐蚀，滤除白色噪点
    img.draw_rectangle((50,10,220,220), color=(255,0,0))#标注出平衡板区域
    for c in img.find_blobs([(254,255)],roi=(50,10,220,220),x_stride=6, y_stride=6,pixel_threshold=75):
        img.draw_cross(c.cx(),c.cy(),size=15,color=(255,125,125))#在色块中心绘制十字线
        x=(int(c.cx())-50)/220*650-325#计算小球在板上X坐标
        y=(int(c.cy())-10)/220*650-325#计算小球在板上Y坐标
def PID_SuanFa(x0,y0):#计算舵机偏移量并控制平衡板运动
    global x,y,PD_x,PI_x,PD_y,PI_y
    Px=0.96
    Ix=0.08
    Dx=0.12
    Py=0.95
    Iy=0.08
    Dy=0.12
    x0-=x
    y0-=y
    PWM_x=Px*(x0-PD_x)+Ix*x0+Dx*(x0-2*PD_x+PI_x)
    PWM_y=Py*(y0-PD_y)+Iy*y0+Dy*(y0-2*PD_y+PI_y)
    PD_x=x0
    PD_y=y0
    PI_x=PD_x
    PI_y=PD_y

    print(PWM_x,PWM_y)#代替舵机输出
def WenDing(P):
    while(QvYv_x[P]*0.95>x or x>QvYv_x[P]*1.05)and(QvYv_y[P]*0.95>y or y>QvYv_y[P]*1.05):
        while(QvYv_x[P]*0.95>x or x>QvYv_x[P]*1.05)and(QvYv_y[P]*0.95>y or y>QvYv_y[P]*1.05):
            img = sensor.snapshot()#拍摄图像
            TiQvZuoBiao(img)#提取小球相对平衡板的坐标[x,y]
            PID_SuanFa(QvYv_x[P],QvYv_y[P])#稳定在第P+1点
        time.sleep_ms(2300)#停留2.3s
while(True):#主函数部分
    clock.tick()#更新FPS帧率时钟。
    moshi=1#模式选择
    if moshi==1:#第一问
        while(True):
            img = sensor.snapshot()#拍摄图像
            TiQvZuoBiao(img)#提取小球相对平衡板的坐标[x,y]
            PID_SuanFa(QvYv_x[1],QvYv_y[1])#稳定在第2点
    elif moshi==2:#第二问
        while(True):
            img = sensor.snapshot()#拍摄图像
            TiQvZuoBiao(img)#提取小球相对平衡板的坐标[x,y]
            PID_SuanFa(QvYv_x[4],QvYv_y[4])#稳定在第5点
    elif moshi==3:#第三问
        while(True):
            img = sensor.snapshot()#拍摄图像
            TiQvZuoBiao(img)#提取小球相对平衡板的坐标[x,y]
            PID_SuanFa(QvYv_x[3],QvYv_y[3])#稳定在第4点
            if QvYv_x[3]*0.95<x<QvYv_x[3]*1.05 and QvYv_y[3]*0.95<y<QvYv_y[3]*1.05:
                time.sleep_ms(2300)#停留2.3s
                if QvYv_x[4]*0.95<x<QvYv_x[4]*1.05 and QvYv_y[4]*0.95<y<QvYv_y[4]*1.05:
                    while(True):
                        img = sensor.snapshot()#拍摄图像
                        TiQvZuoBiao(img)#提取小球相对平衡板的坐标[x,y]
                        PID_SuanFa(QvYv_x[4],QvYv_y[4])#稳定在第5点
    elif moshi==4:#第四问
        while(True):
            img = sensor.snapshot()#拍摄图像
            TiQvZuoBiao(img)#提取小球相对平衡板的坐标[x,y]
            PID_SuanFa(QvYv_x[8],QvYv_y[8])#稳定在第9点
    elif moshi==5:#第五问
        WenDing(1)#稳定在第2点并保持2.3s
        WenDing(5)#稳定在第6点并保持2.3s
        while(True):
            img = sensor.snapshot()#拍摄图像
            TiQvZuoBiao(img)#提取小球相对平衡板的坐标[x,y]
            PID_SuanFa(QvYv_x[8],QvYv_y[8])#稳定在第9点
    elif moshi==6:#第六问
        WenDing(B-1)#稳定在第B点并保持2.3s
        WenDing(C-1)#稳定在第C点并保持2.3s
        while(True):
            img = sensor.snapshot()#拍摄图像
            TiQvZuoBiao(img)#提取小球相对平衡板的坐标[x,y]
            PID_SuanFa(QvYv_x[D-1],QvYv_y[D-1])#稳定在第D点
    elif moshi==6:#第七问
        for i in range(14):
            WenDing(CiShu[i]-1)#稳定在第B点并保持2.3s
        while(True):
            img = sensor.snapshot()#拍摄图像
            TiQvZuoBiao(img)#提取小球相对平衡板的坐标[x,y]
            PID_SuanFa(QvYv_x[8],QvYv_y[8])#稳定在第9点

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import csv

def Get_datas(filePath, datas, x):
    # with ... as 的用法  1、 首先调用open()返回对象的__enter__()方法， __enter__()的返回值赋给f  2、结束时自动调用 __exit__()方法   3、异常处理 
    with open(filePath) as f:
        reader = csv.reader(f)
        i = 0 
        for row in reader:
            try:
                value = row[1]
            except ValueError:
                print('Error data')
            else:
                if(i==0):  
                    i = i + 1 
                    continue  
                datas.append(float(value))
                x.append(i)
                i = i + 1 
                if(i==100):
                    break

y, x = [],[]

filePath = '/home/gogo/lwh/lwh_ws/src/liv_slam-master/slam_data/time/times_scan_scan.csv'

# 读取数据 
Get_datas(filePath, y, x)   

sum = 0
# 求均值
for i in y:
    sum += i

average = sum / len(y)
averages = []
for i in range(len(y)):
    averages.append(average)
print('average is: ', average)

plt.plot(x, y, c='red', linewidth=0.5)
plt.plot(x, averages, c='blue', linewidth=1)    # 绘制平均曲线  

title = 'data curve'
plt.title(title, fontsize=24)
plt.xlabel('index')
plt.ylabel('matcher score')

plt.show()














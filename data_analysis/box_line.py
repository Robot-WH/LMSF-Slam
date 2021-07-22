#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pandas as pd
import matplotlib.pyplot as plt

# 读取Titanic数据集     
# /home/gogo/lwh/lwh_ws/src/liv_slam-master/slam_data/time/times.csv
# '/home/gogo/lwh/实验/融合/IMU/IMU预测/addImu.csv'
# '/home/gogo/lwh/实验/融合/IMU/IMU预测/noImu.csv'
datas_1 = pd.read_csv('/home/gogo/lwh/实验/融合/IMU/IMU预测/addImu.csv')
datas_2 = pd.read_csv('/home/gogo/lwh/实验/融合/IMU/IMU预测/noImu.csv')


# print(datas.time)

# 设置图片大小
plt.figure(figsize=(10,8))

# 设置图形的显示风格
plt.style.use('ggplot')

# 设置中文和负号正常显示
plt.rcParams['font.sans-serif'] = [u'SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 绘图
# plt.boxplot(x=datas.time, # 指定绘图数据
#             labels=datas.columns,
#             patch_artist  = True, # 要求用自定义颜色填充盒形图，默认白色填充
#             showmeans = True, # 以点的形式显示均值
#             whis=8,   
#             widths=0.2,      #箱体宽度
#             boxprops = {'color':'black','facecolor':'#9999ff'}, # 设置箱体属性，填充色和边框色
#             flierprops = {'marker':'o','markerfacecolor':'red','color':'black'}, # 设置异常值属性，点的形状、填充色和边框色
#             meanprops = {'marker':'D','markerfacecolor':'indianred'}, # 设置均值点的属性，点的形状、填充色
#             medianprops = {'linestyle':'--','color':'orange'}         # 设置中位数线的属性，线的类型和颜色
#             ) 

plt.boxplot( #x = datas.time, # 指定绘图数据
            (datas_1.score, datas_2.score),

            labels=('add imu','no imu'),

            patch_artist=True, # 要求用自定义颜色填充盒形图，默认白色填充
 
            showmeans=True, # 以点的形式显示均值
 
            boxprops = {'color':'black','facecolor':'#9999ff'}, # 设置箱体属性，填充色和边框色
 
            flierprops = {'marker':'o','markerfacecolor':'red','color':'black'}, # 设置异常值属性，点的形状、填充色和边框色
 
            meanprops = {'marker':'D','markerfacecolor':'indianred'}, # 设置均值点的属性，点的形状、填充色
 
            medianprops = {'linestyle':'--','color':'orange'}) # 设置中位数线的属性，线的类型和颜色


plt.ylabel('match score')

plt.title('imu predict')    #子标题 

# # 去除箱线图的上边框与右边框的刻度标签
# plt.tick_params(top='off', right='off')

# 显示图形
plt.show()

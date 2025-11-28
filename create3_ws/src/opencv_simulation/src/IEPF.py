# Author Eko Rudiawan
from scipy import linspace, polyval, polyfit, sqrt, stats, randn
import timeit
import math
import matplotlib.pyplot as plt
import numpy as np
import random
import pandas as pd

# 计算两点之间的欧几里得距离
def measPointToPoint(varP1, varP2):
    return math.sqrt(((varP2[0]-varP1[0])*(varP2[0]-varP1[0])) + ((varP2[1]-varP1[1])*(varP2[1]-varP1[1])))

# 计算点到直线的距离
# 直线由两个点 varPk 和 varPl 定义，varP0 是待测点
def measPointToLine(varPk, varPl, varP0):
    return abs((varPl[1]-varPk[1])*varP0[0] - (varPl[0]-varPk[0])*varP0[1] + varPl[0]*varPk[1] - varPl[1]*varPk[0]) / math.sqrt( math.pow((varPl[0] - varPk[0]), 2) + math.pow((varPl[1] - varPk[1]), 2))

# IEPF函数
# 输入为点坐标列表和端点列表 
# 输出为直线方程列表，格式为 Ax + By + C = 0
def iepfFunction(dThreshold, ptList, ePtList):
    maxDPtToLine = 0  # 最大距离初始化为0
    breakPointIndex = -1  # 断点索引初始化为-1
    _, jumlahEndpoint = ePtList.shape  # 获取端点的数量
    # 遍历每一对端点
    for i in range(0, jumlahEndpoint - 1):
        # 计算斜率 A
        varA = float(ePtList[1, i + 1] - ePtList[1, i]) / float(ePtList[0, i + 1] - ePtList[0, i])
        varB = -1.00  # B 为 -1
        varC = float(ePtList[1, i] - varA * ePtList[0, i])  # 计算 C
        # 遍历端点之间的点
        for j in range(ePtList[2, i], ePtList[2, i + 1]):
            if j == 0 or j == ePtList[2, i]:
                continue  # 跳过首尾端点
            # 计算点到直线的距离
            dPtToLine = float(abs((varA * ptList[0, j] + varB * ptList[1, j] + varC) / (math.sqrt(varA * varA + varB * varB))))
            # 如果距离超过阈值，检查是否为最大值
            if dPtToLine > dThreshold:
                if (dPtToLine > maxDPtToLine):
                    maxDPtToLine = dPtToLine
                    breakPointIndex = j  # 更新断点索引
    # 如果找到断点，则递归调用
    if breakPointIndex != -1:        
        y = np.array([[ptList[0, breakPointIndex]], [ptList[1, breakPointIndex]], [breakPointIndex]])
        ePtList = np.insert(ePtList, [jumlahEndpoint - 1], y, axis=1)  # 插入新的断点
        ePtList = iepfFunction(dThreshold, ptList, ePtList)  # 递归调用

    return ePtList  # 返回更新后的端点列表
    

# 合并线段
def mergeLine(mneThreshold, ptList, ePtList):
    jumlahEndpoint = len(ePtList[0])  # 获取端点数量
    for i in range(0, jumlahEndpoint - 2):
        varPk = [ePtList[0][i], ePtList[1][i]]  # 当前线段的起点
        varPl = [ePtList[0][i + 2], ePtList[1][i + 2]]  # 当前线段的终点
        varP0 = [ePtList[0][i + 1], ePtList[1][i + 1]]  # 待合并的点
        varMaxDistance = measPointToLine(varPk, varPl, varP0)  # 计算当前线段与待合并点的距离
        varPk = [ePtList[0][i], ePtList[1][i]]  # 当前线段的起点
        varPl = [ePtList[0][i + 1], ePtList[1][i + 1]]  # 当前线段的中点
        prevIndex = ePtList[2][i + 1] - 1  # 前一个点的索引
        nextIndex = ePtList[2][i + 1] + 1  # 后一个点的索引
        varP0 = [ptList[0][prevIndex], ptList[1][nextIndex]]  # 获取待合并点的坐标
        xx = measPointToLine(varPk, varPl, varP0)  # 计算当前线段与待合并点的距离
        varMNEprev = varMaxDistance / xx  # 计算相对距离
        # 如果相对距离大于阈值，则合并线段
        if varMNEprev > 2:
            ePtList[0].pop(i + 1)  # 移除当前线段的中点
            ePtList[1].pop(i + 1)
            ePtList[2].pop(i + 1)
    return ePtList  # 返回合并后的端点列表

# 测试速度的函数
def testSpeed(loop):
    startTime = time.perf_counter()  # 记录开始时间
    for i in range(0, loop):
        P0 = [0, 0]  # 起始点
        P1 = [10, 10]  # 终止点
        measPointToPoint(P0, P1)  # 计算距离
    endTime = time.perf_counter()  # 记录结束时间
    totalTime = (endTime - startTime) / loop  # 计算平均时间
    totalTime *= 1000000  # 转换为微秒
    print('Total time {} microseconds'.format(totalTime))  # 输出结果

def main():
    # 从CSV文件读取数据集
    df = pd.read_csv('dataset.csv')
    # 将数据转换为NumPy数组
    npDataset = df.to_numpy()  # 更改为to_numpy()
    # 删除第0列并转置
    npPoint = np.transpose(np.delete(npDataset, 0, axis=1))
    # 设置初始端点
    endPoint0 = 0
    endPointN = npPoint[0].size - 1

    npEndpoint = np.zeros((3, 2), dtype=int)  # 初始化端点数组

    npEndpoint[0, 0] = npPoint[0, endPoint0]  # 端点的x坐标
    npEndpoint[1, 0] = npPoint[1, endPoint0]  # 端点的y坐标
    npEndpoint[2, 0] = endPoint0  # 端点的索引

    npEndpoint[0, 1] = npPoint[0, endPointN]  # 端点的x坐标
    npEndpoint[1, 1] = npPoint[1, endPointN]  # 端点的y坐标
    npEndpoint[2, 1] = endPointN  # 端点的索引

    # 调用IEPF函数提取直线，阈值设为50
    predictedLine = iepfFunction(30, npPoint, npEndpoint)

    _, jumlahEndpoint = predictedLine.shape  # 获取预测线的端点数量
    jumlahGaris = jumlahEndpoint - 1  # 计算直线段数量
    
    clusterPoint = np.zeros((jumlahGaris, 2, 1))  # 初始化聚类点

    plt.plot(npPoint[0], npPoint[1], 'r')  # 绘制原始点
    plt.plot(predictedLine[0], predictedLine[1])  # 绘制提取的直线
    plt.axis([0, 640, 0, 640])  # 设置坐标轴范围
    plt.show()  # 显示图形
    
if __name__ == "__main__":
    main()  # 执行主函数


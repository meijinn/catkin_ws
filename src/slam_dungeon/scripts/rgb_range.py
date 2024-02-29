#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import os

# 対象画像読み込み
file_name='./img/1.jpg'
img = cv2.imread(file_name,cv2.IMREAD_COLOR)

height, width, channels = img.shape[:3]

x = width//2
y = height//2

w = width//4
h = height//4

# 対象範囲を切り出し
fromX1 = 0 #対象範囲開始位置 X座標
fromY1 = 0 #対象範囲開始位置 Y座標
ToX1 = x-w #対象範囲終了位置 X座標
ToY1 = height #対象範囲終了位置 Y座標
# y:y+h, x:x+w　の順で設定
LeftBox = img[fromY1: ToY1, fromX1: ToX1]

fromX2 = x+w
fromY2 = 0
ToX2 = width
ToY2 = height
RightBox = img[fromY2: ToY2, fromX2: ToX2]


# RGB平均値を出力
# flattenで一次元化しmeanで平均を取得 
Leftb = LeftBox.T[0].flatten().mean()
Leftg = LeftBox.T[1].flatten().mean()
Leftr = LeftBox.T[2].flatten().mean()

Rightb = RightBox.T[0].flatten().mean()
Rightg = RightBox.T[1].flatten().mean()
Rightr = RightBox.T[2].flatten().mean()


print("B: %.2f" % ((Rightb+Leftb)/2))
print("G: %.2f" % ((Rightg+Leftg)/2))
print("R: %.2f" % ((Rightr+Leftr)/2))
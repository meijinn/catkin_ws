#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import sys

cap = cv2.VideoCapture(0)
cnt = 1

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# 対象範囲を切り出し
fromX = 0 #対象範囲開始位置 X座標
fromY = 0 #対象範囲開始位置 Y座標
ToX = width #対象範囲終了位置 X座標
ToY = height #対象範囲終了位置 Y座標

while(True):
    # フレームをキャプチャする
    ret, frame = cap.read()
    # cv2.rectangle(frame, (width, height), (x+w, 0), color=(0,0,255),thickness= 4)
    # cv2.rectangle(frame, (0, height), (x-w, 0), color=(0,0,255),thickness= 4)
    # 画面に表示する
    # frame=cv2.resize(frame,(20,20))
    imgBox = frame[fromY: ToY, fromX: ToX]
    cv2.imshow('frame',frame)
    # print(frame)
    # print('\n')

    b = imgBox.T[0].flatten().mean()
    g = imgBox.T[1].flatten().mean()
    r = imgBox.T[2].flatten().mean()

    print("B: %.2f" % (b))
    print("G: %.2f" % (g))
    print("R: %.2f" % (r))
    # キーボード入力待ち
    key = cv2.waitKey(1) & 0xFF

    # qが押された場合は終了する esc に変更
    if key == 27:
        break
    # sが押された場合は保存する
    if key == ord('s'):
        path = "./img/{}.jpg".format(cnt)
        cv2.imwrite(path,frame)
        cnt += 1

# キャプチャの後始末と，ウィンドウをすべて消す
cap.release()
cv2.destroyAllWindows()
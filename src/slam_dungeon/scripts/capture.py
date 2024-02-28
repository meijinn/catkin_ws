#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2

cap = cv2.VideoCapture(1)

while(True):
    # フレームをキャプチャする
    ret, frame = cap.read()

    # 画面に表示する
    cv2.imshow('frame',frame)

    # キーボード入力待ち
    key = cv2.waitKey(1) & 0xFF

    # qが押された場合は終了する
    if key == ord('q'):
        break
    # sが押された場合は保存する
    if key == ord('s'):
        path = "./img/photo.jpg"
        cv2.imwrite(path,frame)

# キャプチャの後始末と，ウィンドウをすべて消す
cap.release()
cv2.destroyAllWindows()
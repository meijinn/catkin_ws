#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2

#画像を読み込む
image = cv2.imread('./img/1.jpg')
#image=cv2.resize(image,(4,4))
#imreadで返された配列を出力
print(image)

#読み込んだ画像を表示する
cv2.imshow('img',image)
cv2.waitKey(0)
cv2.destroyAllWindows()

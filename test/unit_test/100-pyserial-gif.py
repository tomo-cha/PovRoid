# -*- coding: utf-8 -*-

import serial
import time
import cv2
import math
import sys
from PIL import Image

args = sys.argv

# 配列設定
PIXELS = 50  # LED1本あたりのセル数
NUMTAPES = 2  # 繋げるLEDの本数
Div = 60  # 1周の分割数
l = [[0] * PIXELS*NUMTAPES for i in range(Div)]  # RGBを格納するためのリスト宣言・初期化

Bright = 15  # 輝度
Led0Bright = 50  # 中心LEDの輝度 [%]

# シリアルポートの設定
ser = serial.Serial('/dev/cu.usbserial-130', 230400,timeout=1)  # 適切なポート名に変更してください
time.sleep(2)  # 接続の安定のために少し待機


# 画像変換関数
def polarConv(imgOrgin):
    h, w, _ = imgOrgin.shape
    imgRedu = cv2.resize(imgOrgin, (math.floor((PIXELS * 2 - 1) / h * w), PIXELS * 2 - 1))
    h2, w2, _ = imgRedu.shape
    wC = math.floor(w2 / 2)
    hC = math.floor(h2 / 2)
    imgPolar = Image.new('RGB', (PIXELS, Div))

    for j in range(0, Div):
        for i in range(0, hC + 1):
            rP = int(imgRedu[hC + math.ceil(i * math.cos(2 * math.pi / Div * j)),
                             wC - math.ceil(i * math.sin(2 * math.pi / Div * j)), 2]
                     * ((100 - Led0Bright) / PIXELS * i + Led0Bright) / 100 * Bright / 100)
            gP = int(imgRedu[hC + math.ceil(i * math.cos(2 * math.pi / Div * j)),
                             wC - math.ceil(i * math.sin(2 * math.pi / Div * j)), 1]
                     * ((100 - Led0Bright) / PIXELS * i + Led0Bright) / 100 * Bright / 100)
            bP = int(imgRedu[hC + math.ceil(i * math.cos(2 * math.pi / Div * j)),
                             wC - math.ceil(i * math.sin(2 * math.pi / Div * j)), 0]
                     * ((100 - Led0Bright) / PIXELS * i + Led0Bright) / 100 * Bright / 100)

            l[j][hC - i] = '%02X%02X%02X' % (gP, rP, bP)
            # l[j][hC - i] = str(bP)
            # print(bP)


# Gifファイルを読み込む
while True:
    count = 0
    gif = cv2.VideoCapture("python/hands/1.png")  # ファイルパスを適切に設定してください

    while True:
        is_success, pic = gif.read()
        if not is_success:
            break

        if count % 15 == 0:  # 12枚ごとに表示する
            polarConv(pic)

            for k in range(3):  # パケットロスを防ぐために3回送信
                for j in range(0, Div):
                    data = 'b%02X' % j  # データの開始文字 'b' を付ける
                    for i in range(0, PIXELS):
                        data += l[j][i]
                    for i in range(0, PIXELS):
                        data += l[int((j + Div / 2 - 1) % Div)][PIXELS - 1 - i]
                        if i == PIXELS - 1:
                            data += '/'  # データの終了文字 '/' を付ける
                            ser.write(data.encode('utf-8'))
                            # time.sleep(0.001)  # 送信間隔を調整
                            print(data.encode('utf-8'))
            print(count)
        count += 1

ser.close()

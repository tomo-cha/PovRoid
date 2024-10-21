# -*- coding: utf-8 -*-

import socket
import time
import cv2
import os
import math
import sys
from PIL import Image

args = sys.argv

# 配列設定
PIXELS = 25  # LED1本あたりのセル数
NUMTAPES = 2  # 繋げるLEDの本数
Div = 60  # 1周の分割数
l = [[0] * PIXELS*NUMTAPES for i in range(Div)]  # RGBを格納するためのリスト宣言・初期化

Bright = 15 #輝度
Led0Bright = 50 #中心LEDの輝度 [%]



#画像変換関数
def polarConv(imgOrgin):
    #画像データ読み込み
    # imgOrgin = cv2.imread(pic) 

    #画像サイズ取得
    h, w, _ = imgOrgin.shape

    #画像縮小
    imgRedu = cv2.resize(imgOrgin,(math.floor((PIXELS * 2 -1)/h *w), PIXELS * 2 -1))

    #縮小画像中心座標
    h2, w2, _ = imgRedu.shape
    wC = math.floor(w2 / 2)
    hC = math.floor(h2 / 2)

    #極座標変換画像準備
    imgPolar = Image.new('RGB', (PIXELS, Div))


    #極座標変換
    for j in range(0, Div):
        # file.write('\t{')
        for i in range(0, hC+1):
            #座標色取得
            #参考：http://peaceandhilightandpython.hatenablog.com/entry/2016/01/03/151320
            rP = int(imgRedu[hC + math.ceil(i * math.cos(2*math.pi/Div*j)),
                         wC - math.ceil(i * math.sin(2*math.pi/Div*j)), 2]
                     * ((100 - Led0Bright) / PIXELS * i + Led0Bright) / 100 * Bright /100)
            gP = int(imgRedu[hC + math.ceil(i * math.cos(2*math.pi/Div*j)),
                         wC - math.ceil(i * math.sin(2*math.pi/Div*j)), 1]
                     * ((100 - Led0Bright) / PIXELS * i + Led0Bright) / 100 * Bright /100)
            bP = int(imgRedu[hC + math.ceil(i * math.cos(2*math.pi/Div*j)),
                         wC - math.ceil(i * math.sin(2*math.pi/Div*j)), 0]
                     * ((100 - Led0Bright) / PIXELS * i + Led0Bright) / 100 * Bright /100)
            
            # file.write('0x%02X%02X%02X' % (rP,gP,bP))

            l[j][hC-i] = '%02X%02X%02X' % (gP, rP, bP)
            
            # if i == hC:
            #     file.write('},\n')
            # else:
            #     file.write(', ')
                
            imgPolar.putpixel((i,j), (rP, gP, bP))
    # file.write('};')

# Gifファイルを読み込む
while True:
    count = 0
    gif = cv2.VideoCapture("/Users/takaharatomotakeshi/workspace/nakanishi/PovRoid/python/ufo_black_bg.gif")

    while True:
        is_success, pic = gif.read()

        # ファイルが読み込めなくなったら終了
        if not is_success:
            break

        # print(pic)
        #変換
        if(count%15==0): #12枚ごとに表示する
            # 変換
            polarConv(pic)
            # udp設定
            sendAddr = ('192.168.11.33', 1234)  # 送信先(esp32)のipアドレス, ポート番号は1234で統一する
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            for k in range(3): #パケットロスがあるので3回送る
                for j in range(0, Div):
                    data = '%02X' % j
                    for i in range(0, PIXELS):
                        data+=l[j][i]
                    for i in range(0, PIXELS):
                        data+=l[int((j+Div/2-1)%Div)][PIXELS-1-i]
                        if i == PIXELS-1:
                            udp.sendto(data.encode('utf-8'), sendAddr)
                            time.sleep(0.002) #sleepがないとパケットロスが激増する
                            # print(data.encode('utf-8'))
            print(count)
        count += 1

udp.close()

#ファイル作成
file = open('src/graphics.h', 'w')
file.write('#define NUMPIXELS ' + str(PIXELS*NUMTAPES) + '\n')
file.write('#define Div ' + str(Div) + '\n' + '\n')
#file.write('#define Frame ' + str(Frame) + '\n' + '\n')


file.write('uint32_t pic [Div][NUMPIXELS] = {' + '\n')
for j in range(0, Div):
    file.write('\t{')
    for i in range(0, PIXELS):
        file.write("0x")
        file.write(l[j][i])
        file.write(', ')
    for i in range(0, PIXELS):
        file.write("0x")
        file.write(l[int((j+Div/2-1)%Div)][PIXELS-1-i])
        if i == PIXELS-1:
            file.write('},\n')
        else:
            file.write(', ')
file.write('};')

file.close()

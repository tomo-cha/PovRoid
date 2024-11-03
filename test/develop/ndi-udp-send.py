import sys
import numpy as np
# import cv2
import NDIlib as ndi

import socket
import time
import cv2
import os
import math
import sys
from PIL import Image


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
            

            l[j][hC-i] = '%02X%02X%02X' % (gP, rP, bP)
                
            imgPolar.putpixel((i,j), (rP, gP, bP))


def main():

    if not ndi.initialize():
        return 0

    ndi_find = ndi.find_create_v2()

    if ndi_find is None:
        return 0

    sources = []
    while not len(sources) > 0:
        print('Looking for sources ...')
        ndi.find_wait_for_sources(ndi_find, 1000)
        sources = ndi.find_get_current_sources(ndi_find)

    ndi_recv_create = ndi.RecvCreateV3()
    ndi_recv_create.color_format = ndi.RECV_COLOR_FORMAT_BGRX_BGRA

    ndi_recv = ndi.recv_create_v3(ndi_recv_create)

    if ndi_recv is None:
        return 0

    target_ndi=""

    for s in sources:
        if(s.ndi_name == "MSI (D1)"):
            target_ndi = s.ndi_name
            ndi.recv_connect(ndi_recv, s)

    ndi.find_destroy(ndi_find)

    cv2.startWindowThread()

    while True:
        t, v, _, _ = ndi.recv_capture_v2(ndi_recv, 5000)

        if t == ndi.FRAME_TYPE_VIDEO:
            print(f"from:{target_ndi} ")
            print('Video data received (%dx%d).' % (v.xres, v.yres))
            frame = np.copy(v.data)
            polarConv(pic) #画像の極座標変換
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
            # cv2.imshow('ndi image', frame)
            ndi.recv_free_video_v2(ndi_recv, v)

        if cv2.waitKey(1) & 0xff == 27:
            break

    ndi.recv_destroy(ndi_recv)
    ndi.destroy()
    cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())

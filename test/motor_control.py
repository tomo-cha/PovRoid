import socket

# udp設定
sendAddr1 = ('192.168.10.31', 9090)  # 送信先(esp32)のipアドレス, ポート番号は1234で統一する
sendAddr = ('192.168.10.32', 9090)  # 送信先(esp32)のipアドレス, ポート番号は1234で統一する
sendAddr3 = ('192.168.10.33', 9090)  # 送信先(esp32)のipアドレス, ポート番号は1234で統一する
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while(1):
    # print("1, 2, 3")
    # to = input()
    # if(to == "1"):
    #     sendAddr = sendAddr1
    # if(to == "2"):
    #     sendAddr = sendAddr2
    # if(to == "3"):
    #     sendAddr = sendAddr3
    print("input r, a, and number")
    data = input()
    for _ in range(10):
        udp.sendto(data.encode('utf-8'), sendAddr) #失敗するので複数回
    print(data.encode('utf-8'))


udp.close()
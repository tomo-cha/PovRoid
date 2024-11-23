'''
速度制御

'''
import serial
import time

# シリアルポートの設定（ポート名とボーレートを適切に設定）
ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0)  # ポート名とボーレートを変更してください
time.sleep(2)  # 接続の安定化のために少し待機

while True:
    # 好きな値を入力
    limit_value = input("Enter voltage limit value: ")

    # コマンドを作成して送信
    command = f"L{limit_value}\n"  # コマンドに入力値を追加し改行も付加
    ser.write(command.encode())  # バイト形式に変換して送信

    # 必要に応じて返信を確認する
    response = ser.readline().decode().strip()  # 応答の読み込みとデコード
    print("Response:", response)

# シリアルポートを閉じる
ser.close()

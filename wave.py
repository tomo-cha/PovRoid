#動いた  https://github.com/google-ai-edge/mediapipe/blob/master/docs/solutions/hands.md



import cv2
import mediapipe as mp
import math
import serial
import time

# シリアルポートの設定
port = '/dev/cu.usbserial-A5XK3RJT' # Macの場合、ポート名は'/dev/tty.usbserial-xxxxx'など
baudrate = 115200  # 通信速度

# シリアルポートのオープン
ser = serial.Serial(port, baudrate, timeout=1)
time.sleep(2)  # 接続安定のため少し待つ
start_time = time.time()

# landmarkの繋がり表示用
landmark_line_ids = [ 
    (0, 1), (1, 5), (5, 9), (9, 13), (13, 17), (17, 0),  # 掌
    (1, 2), (2, 3), (3, 4),         # 親指
    (5, 6), (6, 7), (7, 8),         # 人差し指
    (9, 10), (10, 11), (11, 12),    # 中指
    (13, 14), (14, 15), (15, 16),   # 薬指
    (17, 18), (18, 19), (19, 20),   # 小指
]

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=2,                # 最大検出数
    min_detection_confidence=0.7,   # 検出信頼度
    min_tracking_confidence=0.7     # 追跡信頼度
)

cap = cv2.VideoCapture(0)   # カメラのID指定
if cap.isOpened():
    while True:
        # カメラから画像取得
        success, img = cap.read()
        if not success:
            continue
        img = cv2.flip(img, 1)          # 画像を左右反転
        img_h, img_w, _ = img.shape     # サイズ取得

        # 検出処理の実行
        results = hands.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        if results.multi_hand_landmarks:
            # 検出した手の数分繰り返し
            for h_id, hand_landmarks in enumerate(results.multi_hand_landmarks):

                # landmarkの繋がりをlineで表示
                for line_id in landmark_line_ids:
                    # 1点目座標取得
                    lm = hand_landmarks.landmark[line_id[0]]
                    lm_pos1 = (int(lm.x * img_w), int(lm.y * img_h))
                    # 2点目座標取得
                    lm = hand_landmarks.landmark[line_id[1]]
                    lm_pos2 = (int(lm.x * img_w), int(lm.y * img_h))
                    # line描画
                    cv2.line(img, lm_pos1, lm_pos2, (128, 0, 0), 1)

                # landmarkをcircleで表示
                z_list = [lm.z for lm in hand_landmarks.landmark]
                z_min = min(z_list)
                z_max = max(z_list)
                for lm in hand_landmarks.landmark:
                    lm_pos = (int(lm.x * img_w), int(lm.y * img_h))
                    lm_z = int((lm.z - z_min) / (z_max - z_min) * 255)
                    cv2.circle(img, lm_pos, 3, (255, lm_z, lm_z), -1)

                # 検出情報をテキスト出力
                # - テキスト情報を作成
                hand_texts = []
                for c_id, hand_class in enumerate(results.multi_handedness[h_id].classification):
                    hand_texts.append("#%d-%d" % (h_id, c_id)) 
                    hand_texts.append("- Index:%d" % (hand_class.index))
                    hand_texts.append("- Label:%s" % (hand_class.label))
                    hand_texts.append("- Score:%3.2f" % (hand_class.score * 100))
                # - テキスト表示に必要な座標など準備
                lm0 = hand_landmarks.landmark[0]
                lm0_x = int(lm0.x * img_w)
                lm0_y = int(lm0.y * img_h)
                lm12 = hand_landmarks.landmark[12]
                lm12_x = int(lm12.x * img_w)
                lm12_y = int(lm12.y * img_h)
                lm_c = (64, 0, 0)
                font = cv2.FONT_HERSHEY_SIMPLEX
                # - テキスト出力
                for cnt, text in enumerate(hand_texts):
                    cv2.putText(img, text, (lm0_x-50, lm0_y-10 + 10 * cnt), font, 0.3, lm_c, 1)

                # 点0と点12をlineで結び、lineの傾きをtextで表示する
                lm0_pos = (lm0_x, lm0_y)
                lm12_pos = (lm12_x, lm12_y)
                cv2.line(img, lm0_pos, lm12_pos, (0, 128, 0), 5)
                rotate = math.atan2(lm12_y-lm0_y, lm12_x-lm0_x)
                ajust_rotate = round((rotate+math.pi*1/2) % (2 * math.pi), 2) #小数第二位まで
                text = f"rotation: {ajust_rotate}"
                cv2.putText(img, text, (lm12_x-50, lm12_y-10), font, 0.3, lm_c, 1)
                current_time = time.time()

                # 経過時間が2秒以上経っていたら処理を実行
                # if current_time - start_time >= 1:
                #     start_time = current_time
                #     c = f"{ajust_rotate}"
                #     ser.write(c.encode())
                #     print(f"送信: {c}")
                c = f"{rotate}"
                ser.write(c.encode())
                print(f"送信: {c}")

        # 画像の表示
        cv2.namedWindow("window", cv2.WINDOW_NORMAL)
        cv2.imshow("MediaPipe Hands", img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q') or key == 0x1b:
            print("quit")
            break

cap.release()
# シリアルポートのクローズ
ser.close()
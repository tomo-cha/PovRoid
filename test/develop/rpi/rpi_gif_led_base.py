import time
import board
import adafruit_dotstar as dotstar
import serial

# シリアルポートの設定
port = '/dev/ttyUSB1' # Macの場合、ポート名は'/dev/tty.usbserial-xxxxx'など
baudrate = 115200  # 通信速度
# シリアルポートのオープン
ser = serial.Serial(port, baudrate, timeout=0)

# dotstarの設定
numpixels = 50           # Number of LEDs in strip
order     = dotstar.BGR  # Might need GRB instead for older DotStar LEDs
strip     = dotstar.DotStar(board.SCK, board.MOSI, numpixels,
              brightness=0.25, auto_write=False, pixel_order=order)

#gifの変換データ
# pic = [  gif2graphics.pyで変換したpicをここにコピペする



rot_time = 0.082881
time_old = 0.000000
frame = 119
num_frame = 0
div = 60
num_div = 0
state_div = 0
update_cycle = 10
count = 0
tmp = 0.000000

'''
動的なrot_timeをesp32から取得
rot_time/div秒ごとにLEDのパターンを変える
→

'''
while True:                  # Loop forever
	start = time.process_time()

	recv = ser.readline().decode('utf-8').strip()
	if recv:
		# print(recv)
		parts = recv.split()  # スペースで分割
		if len(parts) == 2:
			flag, value = parts[0], parts[1]
			num_div = int(flag) * int(div / update_cycle)
			rot_time = float(value) * 0.000001 * update_cycle #4半周ごとならx4で1周  # 後半の値を rot_time に格納
			# print(rot_time)
	

	if(time.process_time() - time_old > rot_time / div):
		time_old = time.process_time()
		for i in range(numpixels):
			strip[i] = pic[num_frame][(num_div+int(div/2))%div][i]
		
		strip.show()
		# print(f"num_frame:{num_frame}")
		num_div += 1
		if(num_div >= div):
			num_div = 0
			num_frame += 1
		if(num_frame >= frame):
			num_frame = 0
	end = time.process_time()
	if(time.process_time() - tmp > 1):
		print(f"loop time: {end - start}")
		tmp = time.process_time()

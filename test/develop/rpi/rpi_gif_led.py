import time
import board
import adafruit_dotstar as dotstar
import serial
import numpy as np
import NDIlib as ndi
import cv2
import math
from PIL import Image
import concurrent.futures
import os

# LEDと極座標変換設定
PIXELS = 25  # LEDs per strip
NUMTAPES = 2
Div = 60  # Circular division for polar mapping
Bright = 15  # Overall brightness
Led0Bright = 50  # Brightness at the center [%]
NUMPIXELS = PIXELS * NUMTAPES  # Length of DotStar strip
ORDER = dotstar.BGR  # Change to GBR for older DotStar strips

# First two arguments in strip declaration identify the clock and data pins
# (here we're using the hardware SPI pins).
# ガンマ補正
BRIGHTNESS = 0.5
GAMMA = np.array([int(pow(float(i) / 255.0, 2.7) * BRIGHTNESS * 255.0 + 0.5) for i in range(256)], dtype=np.uint8)
DOTS = dotstar.DotStar(
	board.SCK,
	board.MOSI,
	NUMPIXELS,
	auto_write=False,
	brightness=BRIGHTNESS,
	pixel_order=ORDER,
)


# シリアル通信設定
port = '/dev/ttyUSB1'  # Adjust based on your platform
baudrate = 115200
ser = serial.Serial(port, baudrate, timeout=0)

# グローバル変数
COLUMN = [0 for x in range(Div)]
for x in range(Div):
	COLUMN[x] = [[0, 0, 0, 0] for _ in range(NUMPIXELS)]
rot_time = 0.082881  # Default rotation time
time_old = 0.000000
tmp = 0.000000
update_cycle = 10
num_div = 0
prev_frame_hash = None


def initialize_ndi():
	"""Initialize NDI and return receiver and finder objects."""
	if not ndi.initialize():
		raise RuntimeError("NDI Initialization failed.")

	ndi_find = ndi.find_create_v2()
	if ndi_find is None:
		raise RuntimeError("Failed to create NDI finder.")
	sources = []
	while not len(sources) > 0:
		print('Looking for sources ...')
		ndi.find_wait_for_sources(ndi_find, 1000)
		sources = ndi.find_get_current_sources(ndi_find)
		print('Network sources (%s found).' % len(sources))
		for i, s in enumerate(sources):
			print('%s. %s' % (i + 1, s.ndi_name))
	ndi_recv_create = ndi.RecvCreateV3()
	ndi_recv_create.color_format = ndi.RECV_COLOR_FORMAT_BGRX_BGRA
	ndi_recv = ndi.recv_create_v3(ndi_recv_create)
	if ndi_recv is None:
		raise RuntimeError("Failed to create NDI receiver.")

	target_ndi=""
	for s in sources:
		if(s.ndi_name == "MSI (POV1)"):
			target_ndi = s.ndi_name
			print(f"target_ndi:{target_ndi} ")
			ndi.recv_connect(ndi_recv, s)

	ndi.find_destroy(ndi_find)
	cv2.startWindowThread()

	return target_ndi, ndi_recv


def receive_ndi_data(target_ndi,ndi_recv):
	"""Receive video frame from NDI and update `pic` data."""
	global prev_frame_hash

	t, v, _, _ = ndi.recv_capture_v2(ndi_recv, 5000)
	if t == ndi.FRAME_TYPE_VIDEO:
		# print(f"from:{target_ndi} ")
		# print(f"Video data received ({v.xres}x{v.yres}).")
		frame = np.copy(v.data)
		frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
		# frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

		# 前回のフレームと比較
		current_frame_hash = hash(frame_rgb.tobytes())
		if prev_frame_hash != current_frame_hash:
			# print("新しいデータを検出しました。`polarConv`を実行します。")
			polarConv(frame_rgb)
			prev_frame_hash = current_frame_hash  # ハッシュ値を更新
		
		ndi.recv_free_video_v2(ndi_recv, v)


def polarConv(imgOrgin):
	"""Convert an image to polar coordinates."""
	global BRIGHTNESS
	# Load image in RGB format and get dimensions:
	# IMG = Image.fromarray(imgOrgin)
	# PIXELS_FULL = np.array(IMG)

	# リサイズ処理
	# WIDTH_FULL, HEIGHT_FULL = imgOrgin.size
	# if WIDTH_FULL > HEIGHT_FULL:
	# 	RESIZE = imgOrgin.resize((math.floor((NUMPIXELS) / HEIGHT_FULL * WIDTH_FULL), NUMPIXELS+1))
	# else:
	# 	RESIZE = imgOrgin.resize((math.floor(NUMPIXELS+1), math.floor((NUMPIXELS) / WIDTH_FULL * HEIGHT_FULL)))

	# リサイズされた画像をNumPy配列に変換
	# PIXELS = np.array(RESIZE)
	# WIDTH, HEIGHT = RESIZE.size
	# print(f"WIDTH:{WIDTH}, HEIGHT:{HEIGHT}")

	#画像データ読み込み
	# imgOrgin = cv2.imread(pic) 

	#画像サイズ取得
	WIDTH_FULL, HEIGHT_FULL, _ = imgOrgin.shape

	#画像縮小
	if WIDTH_FULL > HEIGHT_FULL:
		# RESIZE = imgOrgin.resize((math.floor((NUMPIXELS) / HEIGHT_FULL * WIDTH_FULL), NUMPIXELS+1))
		imgRedu = cv2.resize(imgOrgin,(math.floor((NUMPIXELS) / HEIGHT_FULL * WIDTH_FULL), NUMPIXELS+1))
	else:
		# RESIZE = imgOrgin.resize((math.floor(NUMPIXELS+1), math.floor((NUMPIXELS) / WIDTH_FULL * HEIGHT_FULL)))
		imgRedu = cv2.resize(imgOrgin,(math.floor(NUMPIXELS+1), math.floor((NUMPIXELS) / WIDTH_FULL * HEIGHT_FULL)))

	#縮小画像中心座標
	PIXELS = np.array(imgRedu)
	WIDTH, HEIGHT, _ = imgRedu.shape


	# Allocate list of lists, one for each column of image.
	# print("Allocating...")

	# Convert entire RGB image into columnxrow 2D list.
	# print("Converting...")
	for x in range(Div):  # For each column of image
		start = time.time()
		#0.4ms
		# y=0~24
		theta = 2*math.pi*x/Div
		for y in range(int(NUMPIXELS/2)):  # For each pixel in column
			r = int(NUMPIXELS/2) - y

			img_x = WIDTH // 2 + int(r * math.sin(theta))
			img_y = HEIGHT // 2 - int(r * math.cos(theta))
			# print(f"x:{x},y:{y},img_x:{img_x},img_y:{img_y}")
			value = PIXELS[img_x % WIDTH, img_y % HEIGHT]  # Read RGB pixel in image
			COLUMN[x][y][0] = GAMMA[value[0]]  # Gamma-corrected R
			COLUMN[x][y][1] = GAMMA[value[1]]  # Gamma-corrected G
			COLUMN[x][y][2] = GAMMA[value[2]]  # Gamma-corrected B
			COLUMN[x][y][3] = BRIGHTNESS  # Brightness
			# if(x==30):
			# 	print(f"x:{x},y:{y},img_x:{WIDTH/2+int(r*math.sin(theta))},img_y:{HEIGHT/2-int(r*math.cos(theta))}")
		# y=25~49
		theta = 2*math.pi*int(x+Div/2)/Div
		for y in range(int(NUMPIXELS/2), NUMPIXELS):  # For each pixel in column
			r = y - int(NUMPIXELS/2) + 1
			
			img_x = WIDTH // 2 + int(r * math.sin(theta))
			img_y = HEIGHT // 2 - int(r * math.cos(theta))
			value = PIXELS[img_x % WIDTH, img_y % HEIGHT]  # Read RGB pixel in image
			COLUMN[x][y][0] = GAMMA[value[0]]  # Gamma-corrected R
			COLUMN[x][y][1] = GAMMA[value[1]]  # Gamma-corrected G
			COLUMN[x][y][2] = GAMMA[value[2]]  # Gamma-corrected B
			COLUMN[x][y][3] = BRIGHTNESS  # Brightness
			# if(x==30):
			# 	print(f"x:{x},y:{y},img_x:{WIDTH/2+int(r*math.sin(theta))},img_y:{HEIGHT/2-int(r*math.cos(theta))}")
		end = time.time()
		# print('loop: TIME {:.6f}\n'.format(end - start))



def handle_serial_data():
	"""Read and parse data from serial input."""
	global rot_time, num_div, update_cycle

	recv = ser.readline().decode('utf-8').strip()
	if recv:
		parts = recv.split()
		if len(parts) == 2:
			try:
				flag, value = parts
				num_div = int(flag) * int(Div / update_cycle)
				rot_time = float(value) * 0.000001 * update_cycle
				# print(f"num_div:{num_div},rot_time:{rot_time}")
			except ValueError:
				pass


def update_led_display():
	"""Update LED strip based on `pic` data."""
	global rot_time, num_div, time_old
	

	if(time.process_time() - time_old > rot_time / Div):

		time_old = time.process_time()
		# for i in range(numpixels):
		# 	strip[i] = pic[num_div][i]
		DOTS[0 : DOTS.n] = COLUMN[num_div]  # Copy column to DotStar buffer
		DOTS[0] = (20,20,20)
		DOTS.show()  # Send data to strip
		# print(f"num_div:{num_div}")
		num_div += 1
		if(num_div >= Div):
			num_div = 0



def main():
	global tmp
	# target_ndi, ndi_recv = initialize_ndi()
	count = 0
	gif = cv2.VideoCapture("/home/ubuntu/control_gpio/ufo_black_bg.gif") #5ms
	
	try:
		while True:
			handle_serial_data()  # Read serial data
			
			is_success, frame = gif.read() #1ms
			# print(is_success)
			

			# ファイルが読み込めなくなったら終了
			if not is_success:
				gif = cv2.VideoCapture("/home/ubuntu/control_gpio/ufo_black_bg.gif")
				count = 0

			if(is_success):
				if(count%15==0):
					polarConv(frame)
				count += 1
			# if(time.process_time() - tmp > 1):
			# 	receive_ndi_data(target_ndi,ndi_recv)  # Update `pic` with NDI data
			# 	tmp = time.process_time()
			update_led_display()  # Refresh LEDs

	except KeyboardInterrupt:
		print("Exiting program...")
	finally:
		# ndi.find_destroy(ndi_find)
		# ndi.recv_destroy(ndi_recv)
		# ndi.destroy()
		# cv2.destroyAllWindows()
		ser.close()


if __name__ == "__main__":
	main()

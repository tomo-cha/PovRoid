import socket

# udp設定: 送信先アドレスを配列にまとめる
sendAddrs = [
    ('192.168.11.31', 9090),  # 送信先(esp32)のipアドレス1
    ('192.168.11.32', 9090),  # 送信先(esp32)のipアドレス2
    ('192.168.11.33', 9090)   # 送信先(esp32)のipアドレス3
]

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    while True:
        # Input both the address choice and the data in a single line
        print("Enter the address (1 for 192.168.11.31, 2 for 192.168.11.32, 3 for 192.168.11.33) and the data to send (separated by a space):")
        user_input = input().strip().split() #https://qiita.com/YuDachi/items/c9e7305b299a560e4473

        if len(user_input) != 2 or user_input[0] not in ['1', '2', '3']:
            print("Invalid input. Please enter the address number (1, 2, or 3) and the data, separated by a space.")
            continue

        # Extract the address choice and the data
        addr_choice = int(user_input[0]) - 1  # Convert to 0-based index
        data = user_input[1]

        # Get the corresponding address from the array
        sendAddr = sendAddrs[addr_choice]

        # Send the data multiple times
        for _ in range(10):
            udp.sendto(data.encode('utf-8'), sendAddr)

        print(f"Sent '{data}' to {sendAddr}")

finally:
    udp.close()

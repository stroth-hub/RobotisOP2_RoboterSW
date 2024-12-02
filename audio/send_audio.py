import socket
import pyaudio


# Socket
HOST = "192.168.137.1"
PORT = 5000
# Audio
CHUNK = 320*2
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

p = pyaudio.PyAudio()
devinfo = p.get_device_info_by_index(3)
devinfo2 = p.is_format_supported(RATE, 3, CHANNELS, FORMAT)
print(devinfo2)
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK,
		input_device_index=3)

print("Recording")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect((HOST, PORT))
    while True:
        data = stream.read(CHUNK)
        client_socket.send(data)

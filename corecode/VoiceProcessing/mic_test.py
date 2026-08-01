"""
[진단 도구] 마이크 실시간 파형 확인

실행: python3 mic_test.py   → 파형 창이 뜬다. 창을 닫으면 종료.
용도: 음성 파이프라인이 안 될 때 "마이크가 실제로 소리를 받고 있는가"를 먼저 확인한다.
      말했는데 직선이면 코드 문제가 아니라 장치/권한/볼륨 문제다.

설정은 MicController.MicConfig와 맞춰 둔 값이다(48kHz, mono, chunk 12000).

주의: 기본 입력 장치를 연다. 장치를 지정하려면 p.open()에 input_device_index를 넘긴다.
      번호 목록은 p.get_device_info_by_index(i)로 확인한다.
"""

import pyaudio
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def update_plot(frame):
    data = stream.read(CHUNK)
    audio_data = np.frombuffer(data, dtype=np.int16)
    line.set_ydata(audio_data)
    
    return line,


# Parameters for the microphone test
FORMAT = pyaudio.paInt16  # Format for the audio
CHANNELS = 1  # Mono audio
RATE = 48000  # Sample rate (44.1kHz is standard for audio)
CHUNK = 12000  # Size of each audio chunk

p = pyaudio.PyAudio()

# Open the stream for recording
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)


fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot(np.arange(0, CHUNK), np.arange(0, CHUNK))  # Initial plot
ax.set_title("Real-Time Audio Waveform")
ax.set_xlabel("Samples")
ax.set_ylabel("Amplitude")
ax.set_ylim(-2**15, 2**15)  # Range for 16-bit PCM audio data


# Create an animation to update the plot in real time
ani = animation.FuncAnimation(fig, update_plot, blit=True, interval=50)
plt.show()

# Clean up when done
stream.stop_stream()
stream.close()
p.terminate()
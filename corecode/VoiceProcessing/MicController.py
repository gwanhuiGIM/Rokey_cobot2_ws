"""
[라이브러리] 마이크 입력 래퍼 — 직접 실행하지 않는다

사용: mic = MicController(); mic.open_stream(); ... ; mic.close_stream()
      wakeup_word.py가 mic.stream을 그대로 넘겨받아 쓴다.

MicConfig 기본값: 48kHz / mono / int16 / chunk 12000(=0.25초) / buffer_size 24000(=0.5초)
openWakeWord는 16kHz를 요구하므로 wakeup_word.py 쪽에서 resample한다.

주의:
- device_index=10이 선언돼 있지만 audio.open()에 넘기지 않는다. 실제로는 시스템 기본 입력 장치가 열린다.
  특정 마이크를 쓰려면 open()에 input_device_index=self.config.device_index를 추가해야 한다.
  장치 번호는 mic_test.py나 `python3 -c "import pyaudio;..."`로 먼저 확인한다.
- record_audio()는 self를 쓰지 않고 내부에서 MicController를 새로 만든다.
  이미 open_stream()한 인스턴스에서 호출하면 장치를 두 번 여는 셈이니 주의.
"""

import pyaudio
import wave
import io

class MicConfig:
    chunk: int = 12000
    rate: int = 48000
    channels: int = 1
    record_seconds: int = 5
    fmt: int = pyaudio.paInt16
    device_index: int = 10
    buffer_size: int = 24000


class MicController:
    def __init__(self, config: MicConfig = MicConfig()):
        self.config = config
        self.frames = []
        self.audio = None     # open_stream()에서 생성
        self.stream = None
        self.sample_width = None  # 스트림 열 때 샘플 폭을 저장

    def open_stream(self):
        """새로운 PyAudio 인스턴스를 생성하고 스트림을 엽니다."""
        self.audio = pyaudio.PyAudio()
        self.sample_width = self.audio.get_sample_size(self.config.fmt)
        self.stream = self.audio.open(
            format=self.config.fmt,
            channels=self.config.channels,
            rate=self.config.rate,
            input=True,
            frames_per_buffer=self.config.chunk,
        )

    def close_stream(self):
        """스트림과 PyAudio 인스턴스를 종료합니다."""
        print("stop recording")
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.audio:
            self.audio.terminate()
            self.audio = None

    # def record_audio(self):
    #     print("start recording for 5 seconds")
    #     self.frames = []  # 이전 프레임 초기화
    #     num_chunks = int(self.config.rate / self.config.chunk * self.config.record_seconds)

    #     for _ in range(num_chunks):
    #         data = self.stream.read(self.config.chunk, exception_on_overflow=False)
    #         self.frames.append(data)

    # def save_wav(self, filename):
    #     """녹음된 데이터를 WAV 파일로 저장합니다."""
    #     with wave.open(filename, 'wb') as wf:
    #         wf.setnchannels(self.config.channels)
    #         wf.setsampwidth(self.sample_width)
    #         wf.setframerate(self.config.rate)
    #         wf.writeframes(b''.join(self.frames))
    #     print("✅ 파일 저장 완료!")

    # def get_wav_data(self):
    #     wav_buffer = io.BytesIO()
    #     with wave.open(wav_buffer, 'wb') as wf:
    #         wf.setnchannels(self.config.channels)
    #         wf.setsampwidth(self.audio.get_sample_size(self.config.fmt))
    #         wf.setframerate(self.config.rate)
    #         wf.writeframes(b''.join(self.frames))
    #     return wav_buffer.getvalue()

    def record_audio(self) -> bytes:
        mic = MicController()
        mic.open_stream()

        print("start recording...")
        frames = []

        for _ in range(0, int(mic.config.rate / mic.config.chunk * mic.config.record_seconds)):
            data = mic.stream.read(mic.config.chunk)
            frames.append(data)

        mic.close_stream()

        # BytesIO를 사용해 메모리 내에서 WAV 파일을 저장
        wav_io = io.BytesIO()
        wf = wave.open(wav_io, 'wb')
        wf.setnchannels(mic.config.channels)
        wf.setsampwidth(mic.sample_width)
        wf.setframerate(mic.config.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return wav_io.getvalue()

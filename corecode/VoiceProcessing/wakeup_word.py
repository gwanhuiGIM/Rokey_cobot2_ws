"""
[음성 파이프라인 1단계] 웨이크업 워드 감지 — "hello rokey"

실행: python3 wakeup_word.py   → 감지될 때까지 블로킹, 감지되면 종료
사용: WakeupWord(buffer_size) → set_stream(mic.stream) → is_wakeup()이 True 될 때까지 폴링

모델: hello_rokey_8332_32.tflite (같은 디렉토리). openWakeWord 커스텀 학습 모델.
처리: 48kHz 마이크 입력 → scipy.resample로 16kHz 변환 → model.predict

임계값 두 개가 따로 논다:
  predict(threshold=0.1) — 모델 내부 컷
  confidence > 0.3       — 실제 판정 기준. 오탐이 잦으면 이 값을 올린다.

주의:
- 첫 실행 시 openwakeword.utils.download_models()가 네트워크로 기본 모델을 받는다(오프라인이면 실패).
- set_stream()을 부르기 전에는 self.model이 None이라 is_wakeup()이 터진다.
- 파이프라인 순서: wakeup_word → STT → keyword_extraction
"""

import numpy as np
import openwakeword
from openwakeword.model import Model
from scipy.signal import resample
from ament_index_python.packages import get_package_share_directory
import MicController

MODEL_NAME = "hello_rokey_8332_32.tflite"


class WakeupWord:
    def __init__(self, buffer_size):
        openwakeword.utils.download_models()
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None
        self.buffer_size = buffer_size

    def is_wakeup(self):
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
        outputs = self.model.predict(audio_chunk, threshold=0.1)
        confidence = outputs[self.model_name]
        print("confidence: ", confidence)
        # Wakeword 탐지
        if confidence > 0.3:
            print("Wakeword detected!")
            return True
        return False

    def set_stream(self, stream):
        self.model = Model(wakeword_models=[MODEL_NAME])
        self.stream = stream


if __name__ == "__main__":
    Mic = MicController.MicController()
    Mic.open_stream()

    wakeup = WakeupWord(Mic.config.buffer_size)
    wakeup.set_stream(Mic.stream)
    while wakeup.is_wakeup() is False:
        pass

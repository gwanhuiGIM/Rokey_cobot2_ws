"""
[음성 파이프라인 2단계] 음성 → 텍스트 (OpenAI Whisper API)

실행: python3 STT.py   → 5초 녹음 후 인식 결과 출력
사용: STT(api_key).speech2text() → str

필요: 같은 디렉토리에 .env, 안에 OPENAI_API_KEY=sk-...
      .env는 절대 커밋하지 않는다(.gitignore 확인).

녹음: sounddevice로 16kHz mono int16, 5초 고정(self.duration).
      임시 wav로 떨어뜨린 뒤 whisper-1에 업로드한다.

주의:
- 네트워크 API다. 오프라인/키 없음/과금 한도 초과면 여기서 멈춘다.
- MicController를 쓰지 않고 sounddevice로 따로 녹음한다. 즉 wakeup_word가 쓰던 스트림과 별개다.
  wakeup 직후 바로 부르면 장치 점유가 겹칠 수 있으니 close_stream() 후 호출한다.
- 5초 고정이라 말이 길면 잘린다. duration을 늘리거나 무음 감지를 붙여야 한다.
- 임시 wav는 delete=False라 /tmp에 남는다.
"""

from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import tempfile
import os

# from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv


load_dotenv(dotenv_path=os.path.join(".env"))
openai_api_key = os.getenv("OPENAI_API_KEY")


class STT:
    def __init__(self, openai_api_key):
        self.client = OpenAI(api_key=openai_api_key)
        # self.openai_api_key = openai_api_key
        self.duration = 5  # seconds
        self.samplerate = 16000  # Whisper는 16kHz를 선호

    def speech2text(self):
        # 녹음 설정
        print("음성 녹음을 시작합니다. \n 5초 동안 말해주세요...")
        audio = sd.rec(
            int(self.duration * self.samplerate),
            samplerate=self.samplerate,
            channels=1,
            dtype="int16",
        )
        sd.wait()
        print("녹음 완료. Whisper에 전송 중...")

        # 임시 WAV 파일 저장
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio)

            # Whisper API 호출
            with open(temp_wav.name, "rb") as f:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1", file=f)

        print("STT 결과: ", transcript.text)
        return transcript.text


if __name__ == "__main__":
    stt = STT(openai_api_key)
    output_message = stt.speech2text()

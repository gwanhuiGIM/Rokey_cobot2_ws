"""
[음성 파이프라인 3단계] 문장 → (도구, 목적지) 추출 (gpt-4o + LangChain)

실행: python3 keyword_extraction.py   → 파일 하단 예시 문장으로 동작 확인
사용: ExtractKeyword().extract_keyword("hammer를 pos1으로 가져와") → (["hammer"], ["pos1"])

필요: 같은 디렉토리에 .env, 안에 OPENAI_API_KEY=sk-...

인식 어휘(프롬프트에 하드코딩): hammer, screwdriver, wrench, pos1, pos2, pos3
  → 새 물체를 추가하려면 prompt_content의 <도구 리스트>와 YOLO 클래스 이름을 함께 고쳐야 한다.
출력 규약: LLM이 "도구1 도구2 / pos1 pos2" 형식으로 답하고, 이 코드가 '/'로 잘라 파싱한다.

주의:
- LLM이 형식을 어기면(‘/’가 0개 또는 2개 이상) 경고만 내고 None을 반환한다.
  호출부에서 None 처리를 반드시 해야 로봇이 빈 값으로 움직이지 않는다.
- temperature=0.5다. 같은 문장에 다른 답이 나올 수 있으니 재현성이 필요하면 0으로 내린다.
- 여기 나온 pos1~pos3는 이름일 뿐이고, 실제 좌표는 로봇 제어 쪽에서 매핑해야 한다.
"""

import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
import warnings
from langchain.prompts import PromptTemplate



class ExtractKeyword:
    def __init__(self):
        load_dotenv(dotenv_path=".env")
        openai_api_key = os.getenv("OPENAI_API_KEY")
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )
        prompt_content = """
            당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에 등장하는 도구의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

            <도구 리스트>
            - hammer, screwdriver, wrench, pos1, pos2, pos3

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / pos1 pos2 ...]
            - 도구와 위치는 각각 공백으로 구분
            - 도구가 없으면 앞쪽은 공백 없이 비우고, 목적지가 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "못 박는 것" → hammer)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.

            <예시>
            - 입력: "hammer를 pos1에 가져다 놔"  
            출력: hammer / pos1

            - 입력: "왼쪽에 있는 해머와 wrench를 pos1에 넣어줘"  
            출력: hammer wrench / pos1

            - 입력: "왼쪽에 있는 hammer를 줘"  
            출력: hammer /

            - 입력: "왼쪽에 있는 못박을 수 있는 것을 줘"  
            출력: hammer /

            - 입력: "hammer는 pos2에 두고 screwdriver는 pos1에 둬"  
            출력: hammer screwdriver / pos2 pos1

            <사용자 입력>
            "{user_input}"                
        """
        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content.strip().split("/")
        if len(result) != 2:
            warnings.warn("The object list is more than one.")
            return None

        object, destination = result[0], result[1]
        object = object.split()

        destination = destination.split()

        print(f"llm's response(object): {object}")
        print(f"llm's response(destination): {destination}")

        return object, destination


if __name__ == "__main__":
    # stt = STT()
    # output_message = stt.speech2text()
    # output_message = "못 박는데 사용되는 도구를 1번 위치로 가져와"
    # output_message = "hammer를 1번 위치로 가져와"
    output_message = "hammer를 pos1으로 가져와"
    extract_keyword = ExtractKeyword()
    keyword = extract_keyword.extract_keyword(output_message)

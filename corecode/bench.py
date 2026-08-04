#!/usr/bin/env python3
"""T4 VRAM·추론시간 측정. 결과를 그대로 파이프라인 파라미터로 쓴다.

측정하는 값 3개, 각각 다른 질문에 답한다:
    latency_ms        "A안(매 프레임 추론)이 실시간이 되는가"   → detect_graspx.md §4 A/B안 판단
    peak_reserved_mb  "T4 15GB에 몇 개까지 동시에 올라가는가"   → 배치 상한
    max_batch         위를 직접 이분탐색 대신 2배씩 늘려 실측    → cdist 충돌필터 K 상한

왜 이렇게 재는가 (이거 틀리면 숫자가 전부 거짓말이 된다):
  - CUDA 커널은 비동기다. synchronize() 없이 time.perf_counter()로 재면
    큐에 넣은 시간만 재고 "0.3ms"라고 나온다. CUDA Event로 잰다.
  - 첫 호출은 CUDA 컨텍스트 생성 + cuDNN autotune + allocator 워밍업이 섞여
    3~10배 느리다. warmup 회를 버린다.
  - allocated < reserved < nvidia-smi.
    allocated = 텐서 실사용, reserved = 캐싱 allocator가 잡은 양,
    nvidia-smi = reserved + CUDA 컨텍스트(T4에서 대략 300~600MB).
    "T4에 올라가는가"를 판단할 땐 reserved + 컨텍스트로 봐야 한다. allocated로 보면 낙관적으로 틀린다.
"""

from __future__ import annotations

import json
import statistics
import time
from pathlib import Path

import torch

# T4 물리 VRAM 15360MiB. 컨텍스트·단편화·다른 프로세스 몫으로 남길 비율.
# UNVERIFIED: 0.85는 관례값이다. 실기에서 OOM 나면 낮춘다.
VRAM_SAFETY_FRAC = 0.85

def _bench_json() -> Path:
    """GPU마다 다른 파일. 커밋하지 않는다(.gitignore) — 측정치는 머신 종속이고,
    git으로 옮기면 GPU가 바뀐 뒤에도 옛 숫자를 조용히 읽는다.
    T4에서 재고 실물 GPU PC로 옮기면 파일이 없으니 load()가 default로 떨어진다 — 그게 맞는 동작이다."""
    name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "nogpu"
    slug = "".join(c if c.isalnum() else "_" for c in name)
    return Path(__file__).with_name(f"bench_{slug}.json")


def measure(fn, *args, warmup: int = 3, iters: int = 20, **kwargs) -> dict:
    """fn을 실측. 반환 단위: ms, MB. 중앙값을 쓴다(평균은 첫 outlier에 끌려간다)."""
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA 없음 — 원격 T4 인스턴스에서 실행할 것")

    for _ in range(warmup):
        fn(*args, **kwargs)
    torch.cuda.synchronize()

    torch.cuda.reset_peak_memory_stats()
    times = []
    start, end = torch.cuda.Event(True), torch.cuda.Event(True)
    for _ in range(iters):
        start.record()
        fn(*args, **kwargs)
        end.record()
        end.synchronize()
        times.append(start.elapsed_time(end))  # ms, GPU 클럭 기준

    return {
        "latency_ms": round(statistics.median(times), 2),
        "latency_p95_ms": round(sorted(times)[int(iters * 0.95) - 1], 2),
        "peak_alloc_mb": round(torch.cuda.max_memory_allocated() / 2**20, 1),
        "peak_reserved_mb": round(torch.cuda.max_memory_reserved() / 2**20, 1),
        "total_vram_mb": round(torch.cuda.get_device_properties(0).total_memory / 2**20),
        "device": torch.cuda.get_device_name(0),
    }


def max_batch(fn_of_n, start: int = 1, ceiling: int = 4096) -> int:
    """OOM 직전 배치를 실측. fn_of_n(n)이 배치 n짜리 1회 실행.

    ponytail: 2배씩 늘리고 첫 OOM에서 멈춘다(정확한 경계 대신 2배 안쪽 하한).
    경계가 성능에 실제로 걸리면 그때 이분탐색으로 좁힌다.
    """
    n, ok = start, 0
    while n <= ceiling:
        try:
            fn_of_n(n)
            torch.cuda.synchronize()
            ok = n
        except torch.cuda.OutOfMemoryError:
            break
        finally:
            torch.cuda.empty_cache()  # 다음 시도가 앞 시도의 캐시로 OOM 나는 걸 막는다
        n *= 2
    return int(ok * VRAM_SAFETY_FRAC)


def save(name: str, stats: dict) -> dict:
    """측정치를 bench_<GPU>.json에 누적. 코드가 하드코딩 대신 여기서 읽는다."""
    path = _bench_json()
    data = json.loads(path.read_text()) if path.exists() else {}
    data[name] = stats | {"measured_at": time.strftime("%Y-%m-%d %H:%M")}
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    return data[name]


def load(name: str, key: str, default):
    """파라미터 읽기. 이 GPU에서 안 쟀으면 default. 하드코딩 대신 이걸 쓴다."""
    path = _bench_json()
    if not path.exists():
        return default
    return json.loads(path.read_text()).get(name, {}).get(key, default)


if __name__ == "__main__":
    # 자체검증 겸 사용 예시: cdist 충돌필터(detect_graspx.md 4단계)를 실제로 재본다.
    # 워크로드를 같이 기록한다 — 이게 없으면 다른 GPU의 숫자와 비교가 성립하지 않는다.
    SCENE_PTS, CTRL_PTS = 20000, 64
    workload = {"scene_pts": SCENE_PTS, "ctrl_pts": CTRL_PTS}
    scene = torch.randn(SCENE_PTS, 3, device="cuda")

    def collide(k):
        g = torch.randn(k, CTRL_PTS, 3, device="cuda")
        return torch.cdist(g.reshape(-1, 3), scene).min(1).values

    s = measure(collide, 64) | {"workload": workload | {"k": 64}}
    print(json.dumps(s, indent=2))
    assert s["latency_ms"] > 0, "CUDA Event가 0을 반환 — synchronize 누락 의심"
    assert s["peak_reserved_mb"] >= s["peak_alloc_mb"], "reserved < allocated는 불가능"
    save("collision_cdist_k64", s)

    k = max_batch(collide)
    print(f"max grasp K (safety {VRAM_SAFETY_FRAC}): {k}")
    assert k >= 1
    save("collision_cdist", {"max_k": k, "workload": workload})

"""best_labels() 순수 함수 테스트 (카메라·GPU 불필요)."""

import numpy as np

from graspgenx_perception.capture_graspgenx_scene import LABEL_OBJ_BASE, best_labels


def test_empty_list_gives_none():
    assert best_labels([]) is None


def test_picks_frame_with_most_object_pixels():
    small = np.zeros((4, 4), dtype=np.uint8)
    small[0, 0] = LABEL_OBJ_BASE + 1
    big = np.zeros((4, 4), dtype=np.uint8)
    big[:2, :2] = LABEL_OBJ_BASE + 1
    assert best_labels([small, big, small]) is big


def test_ignores_non_object_labels_like_table():
    # 라벨값 <= 100(ground=0, table=2)은 "탐지"가 아니다 — 테이블만 가득 찍힌 프레임이
    # 물체를 실제로 잡은 프레임보다 이기면 안 된다.
    table_only = np.full((4, 4), 2, dtype=np.uint8)
    one_object = np.zeros((4, 4), dtype=np.uint8)
    one_object[0, 0] = LABEL_OBJ_BASE + 1
    assert best_labels([table_only, one_object]) is one_object

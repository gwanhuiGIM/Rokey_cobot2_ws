"""best_labels() / filter_labels_by_class() 순수 함수 테스트 (카메라·GPU 불필요)."""

import numpy as np

from graspgenx_perception.capture_graspgenx_scene import (
    LABEL_OBJ_BASE, best_labels, filter_labels_by_class,
)


def test_empty_list_gives_none():
    assert best_labels([]) == (None, None)


def test_picks_frame_with_most_object_pixels():
    small = np.zeros((4, 4), dtype=np.uint8)
    small[0, 0] = LABEL_OBJ_BASE + 1
    big = np.zeros((4, 4), dtype=np.uint8)
    big[:2, :2] = LABEL_OBJ_BASE + 1
    assert best_labels([(1, small), (2, big), (3, small)]) == (2, big)


def test_returns_stamp_of_the_chosen_frame():
    """stamp 가 어긋나면 다른 프레임의 클래스맵으로 필터링해 엉뚱한 물체를 잡는다."""
    a = np.zeros((4, 4), dtype=np.uint8)
    a[0, 0] = LABEL_OBJ_BASE + 1
    b = np.zeros((4, 4), dtype=np.uint8)
    b[:3, :3] = LABEL_OBJ_BASE + 2
    stamp, img = best_labels([(111, a), (222, b)])
    assert stamp == 222 and img is b


def test_ignores_non_object_labels_like_table():
    # 라벨값 <= 100(ground=0, table=2)은 "탐지"가 아니다 — 테이블만 가득 찍힌 프레임이
    # 물체를 실제로 잡은 프레임보다 이기면 안 된다.
    table_only = np.full((4, 4), 2, dtype=np.uint8)
    one_object = np.zeros((4, 4), dtype=np.uint8)
    one_object[0, 0] = LABEL_OBJ_BASE + 1
    assert best_labels([(1, table_only), (2, one_object)])[1] is one_object


def _two_objects():
    labels = np.zeros((4, 4), dtype=np.uint8)
    labels[0, :] = LABEL_OBJ_BASE + 1        # apple
    labels[1, :] = LABEL_OBJ_BASE + 2        # cup
    return labels, {LABEL_OBJ_BASE + 1: 'apple', LABEL_OBJ_BASE + 2: 'cup'}


def test_filter_keeps_only_wanted_class():
    labels, cmap = _two_objects()
    out, _ = filter_labels_by_class(labels, cmap, {'apple'})
    assert (out == LABEL_OBJ_BASE + 1).sum() == 4
    assert not (out == LABEL_OBJ_BASE + 2).any()


def test_filter_does_not_mutate_input():
    """라벨맵은 히스토리에 남아 다음 호출에서 다시 쓰인다 — 제자리 수정하면 오염된다."""
    labels, cmap = _two_objects()
    filter_labels_by_class(labels, cmap, {'apple'})
    assert (labels == LABEL_OBJ_BASE + 2).sum() == 4


def test_filter_drops_labels_missing_from_class_map():
    """클래스맵에 없는 라벨은 정체 불명이다. 남기면 '지정한 물체만'이 거짓말이 된다."""
    labels, _ = _two_objects()
    out, _ = filter_labels_by_class(labels, {LABEL_OBJ_BASE + 1: 'apple'}, {'apple'})
    assert not (out == LABEL_OBJ_BASE + 2).any()


def test_filter_with_no_match_clears_everything():
    labels, cmap = _two_objects()
    out, diag = filter_labels_by_class(labels, cmap, {'banana'})
    assert not (out > LABEL_OBJ_BASE).any()
    assert '없음' in diag

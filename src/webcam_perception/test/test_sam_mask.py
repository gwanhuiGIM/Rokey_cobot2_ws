import numpy as np

from webcam_perception.sam_mask_node import auto_box_from_color, compute_mask


def test_auto_box_from_color_finds_centered_object():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[120:360, 160:480] = (200, 200, 200)
    box = auto_box_from_color(img)
    assert box is not None
    x, y, w, h = box
    assert abs(x - 160) <= 5
    assert abs(y - 120) <= 5
    assert abs(w - 320) <= 10
    assert abs(h - 240) <= 10


def test_auto_box_from_color_returns_none_for_blank_image():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    assert auto_box_from_color(img) is None


def test_compute_mask_shape_and_binary():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[120:360, 160:480] = (200, 200, 200)
    mask = compute_mask(img, (160, 120, 320, 240))
    assert mask.shape == (480, 640)
    assert mask.dtype == np.uint8
    assert set(np.unique(mask)).issubset({0, 255})
    assert mask[240, 320] == 255

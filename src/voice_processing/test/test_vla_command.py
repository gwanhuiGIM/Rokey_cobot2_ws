"""`parse_command` 스키마 검증 테스트. 로봇도 카메라도 GPU 도 필요 없다.

    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 -m pytest src/voice_processing/test/test_vla_command.py -q

⚠️ **source 는 필요하다.** `parse_command` 자체는 rclpy 를 안 쓰지만 모듈 최상단이
`import rclpy` 라 ROS 환경 없이는 collection 단계에서 죽는다.

여기서 지키는 계약은 하나다: **필드가 없거나 타입이 다르면 조용히 기본값으로 진행하지
않는다.** `dry_run` 이 제거된 뒤로 잘못 받은 지시는 곧바로 실제 모션이 된다
(`md/plans/2026-08-08-vla-integration.md` §0-B).
"""

import json

from voice_processing.vla_command_node import classify_cmd, parse_command

DETECT = {'apple', 'orange', 'banana', 'cup'}


def cmd(**fields) -> str:
    payload = {'cmd': 'pick', 'class': 'apple'}
    payload.update(fields)
    return json.dumps(payload)


# ── 받아들이는 것 ───────────────────────────────────────────
def test_minimal_pick():
    out, warn = parse_command(cmd())
    assert out['class'] == 'apple'
    assert out['request_id'] == ''
    assert warn == ''


def test_cmd_defaults_to_pick():
    out, _ = parse_command('{"class": "apple"}')
    assert out is not None and out['cmd'] == 'pick'


def test_pick_and_place_is_the_same_thing():
    # 이 FSM 의 pick 사이클은 어차피 place_joints_deg 에 놓는 것으로 끝난다.
    out, _ = parse_command(cmd(cmd='pick_and_place'))
    assert out is not None


def test_vla_side_field_name_class_name():
    # VLA 의 SceneObject 는 `class_name` 이다. 그쪽 이름 그대로 보내도 받는다.
    out, _ = parse_command('{"cmd": "pick", "class_name": "orange"}')
    assert out['class'] == 'orange'


def test_comma_list_survives():
    # target_classes 가 원래 콤마 목록이고 FSM 이 그대로 브리지에 넘긴다.
    out, _ = parse_command(cmd(**{'class': 'apple,orange'}), allowed_classes=DETECT)
    assert out['class'] == 'apple,orange'


def test_request_id_is_echoed():
    out, _ = parse_command(cmd(request_id='a17-3'))
    assert out['request_id'] == 'a17-3'


# ── 거부하는 것 ─────────────────────────────────────────────
def test_not_json():
    out, why = parse_command('그냥 사과 집어')
    assert out is None and 'JSON' in why


def test_json_but_not_object():
    out, why = parse_command('["apple"]')
    assert out is None and why


def test_unknown_cmd():
    out, why = parse_command(cmd(cmd='drop'))
    assert out is None and 'cmd' in why


def test_empty_class():
    out, why = parse_command('{"cmd": "pick", "class": "  "}')
    assert out is None and 'class' in why


def test_class_with_space_is_rejected():
    # FSM 은 /get_keyword 응답을 공백으로 쪼개 첫 단어만 쓴다 — 뒷부분이 조용히 사라진다.
    out, why = parse_command(cmd(**{'class': 'apple orange'}))
    assert out is None and '공백' in why


def test_class_outside_detect_list():
    out, why = parse_command(cmd(**{'class': 'hammer'}), allowed_classes=DETECT)
    assert out is None and 'hammer' in why


def test_allowed_classes_empty_means_no_check():
    out, _ = parse_command(cmd(**{'class': 'hammer'}))
    assert out is not None


def test_place_is_rejected():
    out, why = parse_command(cmd(place={'kind': 'named', 'value': 'basket'}))
    assert out is None and 'place' in why


def test_falsy_place_is_also_rejected():
    # truthiness 로 보면 `{}` · `""` · `0` 이 통과한다 — 키의 존재로 판정해야 한다.
    for falsy in ({}, '', 0, []):
        out, why = parse_command(cmd(place=falsy))
        assert out is None and 'place' in why, f'{falsy!r} 가 통과했다'


def test_class_is_normalised():
    # 원문을 그대로 넘기면 `'apple,'` 이 target_classes 에 실려 빈 클래스를 찾게 된다.
    out, _ = parse_command(cmd(**{'class': 'apple,,orange,'}))
    assert out['class'] == 'apple,orange'


def test_class_of_only_commas():
    out, why = parse_command(cmd(**{'class': ',,,'}))
    assert out is None and 'class' in why


def test_pixel_without_pixel_wh():
    # 리사이즈된 프레임 위에서 찍은 좌표라면 기준 해상도 없이는 조용히 어긋난다.
    out, why = parse_command(cmd(pixel=[312, 188]))
    assert out is None and 'pixel_wh' in why


def test_pixel_wrong_shape():
    out, why = parse_command(cmd(pixel=[312], pixel_wh=[424, 240]))
    assert out is None and 'pixel' in why


def test_pixel_non_numeric():
    out, why = parse_command(cmd(pixel=['312', 188], pixel_wh=[424, 240]))
    assert out is None and 'pixel' in why


def test_pixel_wh_must_be_positive():
    out, why = parse_command(cmd(pixel=[312, 188], pixel_wh=[0, 240]))
    assert out is None and 'pixel_wh' in why


# ── 미구현 필드의 처리 ──────────────────────────────────────
def test_pixel_warn_passes_with_warning():
    out, warn = parse_command(cmd(pixel=[312, 188], pixel_wh=[424, 240]))
    assert out is not None
    assert out['ignored'] == ['pixel']
    assert out['pixel'] == (312.0, 188.0)
    assert warn


def test_pixel_reject_policy():
    out, why = parse_command(cmd(pixel=[312, 188], pixel_wh=[424, 240]),
                             pixel_policy='reject')
    assert out is None and 'select_by_point' in why


def test_base_xy_is_ignored_not_used():
    out, warn = parse_command(cmd(base_xy=[0.42, -0.18]))
    assert out is not None
    assert out['ignored'] == ['base_xy']
    assert warn


# ── cmd 라우팅 (rqt 패널 버튼 대응) ──────────────────────────
def test_classify_control_cmds():
    for word in ('start', 'abort', 'reset'):
        assert classify_cmd(word) == 'control'


def test_classify_approve_is_blocked():
    # rqt 패널의 '승인' 버튼에 해당한다 — 절대 열지 않는다(계획 §0-B).
    assert classify_cmd('approve') == 'blocked'


def test_classify_pick_and_unknown_both_fall_through_to_pick():
    # 'pick'/'pick_and_place'는 물론, 모르는 값도 parse_command 가 최종 거부하도록
    # 'pick' 갈래로 흘려보낸다.
    assert classify_cmd('pick') == 'pick'
    assert classify_cmd('pick_and_place') == 'pick'
    assert classify_cmd('drop') == 'pick'

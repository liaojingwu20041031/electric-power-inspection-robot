"""Regression tests for the DG-202611 forbidden-claim scanner.

The bug these lock down: the scanner's patterns used to be anchored with `\\b`.
Chinese characters are Unicode word characters, so `\\b` never fires between two
of them, and every CJK-phrased performance claim walked straight through the one
automated guard that exists to stop it.  Chinese terms are therefore matched as
plain substrings now, and `\\b` is banned outright (see
``test_no_pattern_uses_ascii_word_boundary``).

The second half is the mirror image: a disclaimer has to be able to name the
thing it denies, so the mandatory markers must still pass.
"""

from pathlib import Path

from dg_synthetic_validation.evidence_manifest import (
    DISCLAIMER_MARKERS,
    FORBIDDEN_CLAIM_PATTERNS,
    RELOCALIZATION_STATES,
    EvidenceEntry,
    EvidenceManifest,
    check_forbidden_claims,
    verify_manifest,
)


# Every one of these MUST be caught.  The first seven are the CJK phrasings the
# `\b`-anchored scanner silently passed; the rest are their ASCII equivalents.
POSITIVE_CASES = (
    "定位精度20厘米",
    "使用Gazebo仿真采集",
    "定位误差小于20cm",
    "特征重复率95%",
    "重定位成功率超过95%",
    "实车验证通过",
    "真实机器人测试",
    "positioning accuracy 20 cm",
    "relocalization success rate 95%",
    "validated on the real robot",
    "collected in Gazebo",
)

# Every one of these MUST pass clean: the six mandatory markers, a caption that
# only describes what is on screen, a caption that states a limitation, and
# plain elapsed-time / phase bookkeeping.
NEGATIVE_CASES = (
    "SYNTHETIC SOFTWARE VALIDATION",
    "NOT REAL ROBOT DATA",
    "NOT COMPETITION PERFORMANCE EVIDENCE",
    "合成软件验证",
    "非真实机器人数据",
    "非竞赛性能证据",
    "RViz 显示地图与激光点云，重定位状态为 ACTIVE_SCAN",
    "本运行为合成软件验证，未验证定位精度",
    "elapsed 42.0 s, phase ACTIVE_SCAN, event STATE_CHANGE",
    "运行时长 42.0 秒，阶段 VERIFYING，事件 STATE_CHANGE",
)


def test_every_positive_case_is_caught():
    """Reported as a group so one fix does not hide the next missed phrase."""
    missed = [text for text in POSITIVE_CASES if not check_forbidden_claims(text)]
    passed = len(POSITIVE_CASES) - len(missed)
    print(f"POSITIVE {passed}/{len(POSITIVE_CASES)}")
    assert not missed, (
        f"POSITIVE {passed}/{len(POSITIVE_CASES)}; scanner passed forbidden "
        f"claims: {missed}"
    )


def test_no_negative_case_is_flagged():
    flagged = {
        text: check_forbidden_claims(text)
        for text in NEGATIVE_CASES
        if check_forbidden_claims(text)
    }
    passed = len(NEGATIVE_CASES) - len(flagged)
    print(f"NEGATIVE {passed}/{len(NEGATIVE_CASES)}")
    assert not flagged, (
        f"NEGATIVE {passed}/{len(NEGATIVE_CASES)}; legitimate text flagged: "
        f"{flagged}"
    )


def test_no_pattern_uses_ascii_word_boundary():
    """`\\b` cannot survive a CJK junction, so no pattern may carry one.

    This is the guard against the original bug being reintroduced by someone
    "tidying up" a lookaround back into a `\\b`.
    """
    offenders = [
        pattern for pattern, _ in FORBIDDEN_CLAIM_PATTERNS
        if "\\b" in pattern or "\b" in pattern
    ]
    assert not offenders, f"patterns still anchored with \\b: {offenders}"


def test_relocalization_states_are_never_claims():
    """All ten real states, bare and in a caption, must scan clean."""
    for state in RELOCALIZATION_STATES:
        assert check_forbidden_claims(state) == []
        assert check_forbidden_claims(f"重定位状态为 {state}") == []
        assert check_forbidden_claims(f"relocalization state {state}") == []
    assert len(RELOCALIZATION_STATES) == 10


def test_claims_are_caught_mid_sentence_between_cjk_characters():
    """The exact shape `\\b` could not see: a claim with CJK on both sides."""
    for text in (
        "本图为定位精度20厘米的证据",
        "该图使用Gazebo仿真采集得到",
        "图中显示特征重复率95%的结果",
        "结论是重定位成功率超过95%达标",
        "已在真实机器人上完成验证",
        "在real robot上测试通过",
    ):
        assert check_forbidden_claims(text), f"missed mid-sentence claim: {text}"


def test_further_cjk_variants_are_caught():
    """The same claim rephrased the way Chinese actually varies."""
    for text in (
        "特征重复性98%",
        "特征重复度达到97%",
        "复现率95%以上",
        "定位精确度0.2米",
        "位姿偏差小于5cm",
        "定位准确率99%",
        "定位精度99%",
        "定位精度达到95%",
        "准确度98%",
        "重定位成功比例96%",
        "重定位成功 95%",
        "relocalization success 95%",
        "厘米级定位",
        "亚米级精度",
        "真机测试记录",
        "实物机器人现场运行",
        "实际机器人上部署",
        "物理仿真环境采集",
        "使用物理引擎生成",
        "竞赛成绩排名第一",
        "竞赛性能达标",
        "比赛排名前三",
        "20cm精度已达成",
    ):
        assert check_forbidden_claims(text), f"missed CJK variant: {text}"


def test_listed_denials_pass():
    """Denials on the closed allowlist pass; everything else fails closed."""
    for text in (
        "未使用真实机器人，也未使用Gazebo",
        "本图不构成竞赛性能证据",
        "no physical robot was involved. no gazebo was involved.",
    ):
        assert check_forbidden_claims(text) == [], text


def test_disclaimer_does_not_license_a_claim_elsewhere():
    """Exemption is per-occurrence, so a marker cannot launder a claim."""
    assert check_forbidden_claims("定位精度20厘米，NOT REAL ROBOT DATA")
    assert check_forbidden_claims("NOT REAL ROBOT DATA. 真实机器人测试通过。")
    assert check_forbidden_claims("非真实机器人数据；实车验证通过")


def test_ascii_edges_still_suppress_unrelated_numbers():
    """`\\b`'s useful half is kept for ASCII: "120 cm" is not a "20 cm" claim."""
    assert check_forbidden_claims("视野宽度120 cm") == []
    assert check_forbidden_claims("图例宽度 220 cm") == []


def test_entry_validate_reports_forbidden_claims_in_caption_and_notes():
    clean = EvidenceEntry(
        filename="shot.png",
        source="RVIZ",
        real_state="ACTIVE_SCAN",
        caption="RViz 显示地图与激光点云，重定位状态为 ACTIVE_SCAN",
    )
    assert clean.validate() == []

    in_caption = EvidenceEntry(
        filename="shot.png", source="RVIZ", real_state="ACTIVE_SCAN",
        caption="定位精度20厘米",
    )
    assert any("forbidden" in p for p in in_caption.validate())

    in_notes = EvidenceEntry(
        filename="shot.png", source="RVIZ", real_state="ACTIVE_SCAN",
        caption="RViz 显示地图与激光点云", notes="使用Gazebo仿真采集",
    )
    assert any("forbidden" in p for p in in_notes.validate())


def test_written_manifest_keeps_markers_and_surfaces_claims(tmp_path: Path):
    """A clean manifest verifies; a claim in a row surfaces through verify."""
    (tmp_path / "shot.png").write_bytes(b"\x89PNG\r\n\x1a\n")
    manifest = EvidenceManifest(tmp_path)
    manifest.add_entry(
        filename="shot.png",
        source="RVIZ",
        real_state="ACTIVE_SCAN",
        caption="RViz 显示地图与激光点云，重定位状态为 ACTIVE_SCAN",
        notes="本运行为合成软件验证，未验证定位精度",
    )
    result = manifest.write_manifest()
    assert result["problems"] == []
    assert result["verification"]["ok"], result["verification"]["problems"]

    markdown = (tmp_path / "evidence_manifest.md").read_text(encoding="utf-8")
    for marker in DISCLAIMER_MARKERS:
        assert marker in markdown
    # The disclaimer names what it denies, and still scans clean.
    assert check_forbidden_claims(markdown) == []

    dirty = tmp_path / "dirty"
    dirty.mkdir()
    (dirty / "shot.png").write_bytes(b"\x89PNG\r\n\x1a\n")
    bad = EvidenceManifest(dirty)
    bad.add_entry(
        filename="shot.png", source="RVIZ", real_state="ACTIVE_SCAN",
        caption="定位精度20厘米，实车验证通过",
    )
    bad_result = bad.write_manifest()
    assert not bad_result["verification"]["ok"]
    assert any("forbidden" in p for p in verify_manifest(dirty)["problems"])

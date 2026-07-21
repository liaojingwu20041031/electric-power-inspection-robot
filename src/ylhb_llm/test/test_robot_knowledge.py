from pathlib import Path

from ylhb_llm.robot_knowledge import RobotKnowledgeIndex


def test_search_matches_chinese_heading_and_returns_relative_source(tmp_path: Path):
    docs = tmp_path / 'docs'
    docs.mkdir()
    (docs / 'mobile.md').write_text(
        '# Mobile Bridge\n\n## APP 连接\n\n手机与机器人处于可达网络后，使用实时地址连接。',
        encoding='utf-8',
    )
    index = RobotKnowledgeIndex(tmp_path, ['docs/**/*.md'], max_section_chars=30)

    results = index.search('APP 怎么连接')

    assert results[0]['path'] == 'docs/mobile.md'
    assert results[0]['title'] == 'APP 连接'
    assert len(results[0]['content']) <= 30


def test_rebuild_skips_missing_and_outside_workspace_files(tmp_path: Path):
    outside = tmp_path.parent / 'outside.md'
    outside.write_text('# Secret\n\n不应读取', encoding='utf-8')
    index = RobotKnowledgeIndex(tmp_path, ['missing.md', '../outside.md'])

    index.rebuild()

    assert index.search('Secret') == []

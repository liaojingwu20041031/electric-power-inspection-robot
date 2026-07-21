from __future__ import annotations

import re
from pathlib import Path
from typing import Any


_HEADING = re.compile(r'^(#{1,6})\s+(.+?)\s*$', re.MULTILINE)
_WORDS = re.compile(r'[a-z0-9_./-]+', re.IGNORECASE)
_CJK = re.compile(r'[\u3400-\u9fff]')


class RobotKnowledgeIndex:
    def __init__(
        self,
        workspace_dir: Path,
        include_globs: list[str],
        max_section_chars: int = 1800,
    ) -> None:
        self.workspace_dir = Path(workspace_dir).expanduser().resolve()
        self.include_globs = list(include_globs)
        self.max_section_chars = max(1, int(max_section_chars))
        self._sections: list[dict[str, Any]] = []
        self.rebuild()

    def rebuild(self) -> None:
        sections: list[dict[str, Any]] = []
        seen: set[Path] = set()
        for pattern in self.include_globs:
            for path in self.workspace_dir.glob(str(pattern)):
                try:
                    resolved = path.resolve(strict=True)
                    resolved.relative_to(self.workspace_dir)
                except (OSError, ValueError):
                    continue
                if resolved in seen or not resolved.is_file() or resolved.suffix.lower() != '.md':
                    continue
                seen.add(resolved)
                try:
                    text = resolved.read_text(encoding='utf-8')
                except OSError:
                    continue
                relative = resolved.relative_to(self.workspace_dir).as_posix()
                sections.extend(self._split(relative, text))
        self._sections = sections

    def search(self, query: str, limit: int = 4) -> list[dict]:
        query = str(query or '').strip()
        if not query:
            return []
        ranked = []
        for index, section in enumerate(self._sections):
            score = self._score(query, section)
            if score:
                ranked.append((score, -index, section))
        ranked.sort(reverse=True, key=lambda item: (item[0], item[1]))
        return [
            {
                'path': item['path'],
                'title': item['title'],
                'content': item['content'][:self.max_section_chars],
            }
            for _score, _index, item in ranked[:max(1, min(int(limit), 5))]
        ]

    def _split(self, path: str, text: str) -> list[dict[str, str]]:
        matches = list(_HEADING.finditer(text))
        if not matches:
            content = text.strip()
            return [{'path': path, 'title': Path(path).stem, 'content': content}] if content else []
        sections = []
        for index, match in enumerate(matches):
            end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
            content = text[match.end():end].strip()
            sections.append({'path': path, 'title': match.group(2).strip(), 'content': content})
        return sections

    @staticmethod
    def _bigrams(text: str) -> set[str]:
        chars = [char for char in text if _CJK.match(char)]
        return {''.join(chars[index:index + 2]) for index in range(len(chars) - 1)}

    @classmethod
    def _score(cls, query: str, section: dict[str, str]) -> int:
        lowered = query.casefold()
        title = section['title'].casefold()
        content = section['content'].casefold()
        path = section['path'].casefold()
        score = 0
        if lowered in title or lowered in content or lowered in path:
            score += 20
        words = set(_WORDS.findall(lowered))
        score += 6 * len(words & set(_WORDS.findall(title)))
        score += 2 * len(words & set(_WORDS.findall(content + ' ' + path)))
        query_bigrams = cls._bigrams(query)
        score += 4 * len(query_bigrams & cls._bigrams(section['title']))
        score += len(query_bigrams & cls._bigrams(section['content']))
        return score

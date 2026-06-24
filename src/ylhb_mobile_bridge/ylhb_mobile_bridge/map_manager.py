import re
from pathlib import Path
from typing import Any

import yaml


MAP_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]+$')


class MapManagerError(Exception):
    def __init__(self, error: str, message: str, status_code: int) -> None:
        super().__init__(message)
        self.error = error
        self.status_code = status_code


class MapManager:
    def __init__(self, default_map_path: str | Path) -> None:
        default_path = Path(default_map_path).expanduser()
        self.maps_dir = default_path.parent
        self.default_name = default_path.stem

    def _validate_name(self, name: str) -> str:
        if not MAP_NAME_PATTERN.fullmatch(name or ''):
            raise MapManagerError(
                'invalid_map_name',
                'map name must contain only letters, numbers, '
                'underscore or hyphen',
                422,
            )
        return name

    def _paths(self, name: str) -> tuple[Path, Path]:
        safe_name = self._validate_name(name)
        return (
            self.maps_dir / f'{safe_name}.yaml',
            self.maps_dir / f'{safe_name}.pgm',
        )

    @staticmethod
    def _present(path: Path) -> bool:
        return path.exists() or path.is_symlink()

    def _inspect(self, name: str) -> dict[str, Any]:
        yaml_path, pgm_path = self._paths(name)
        yaml_present = self._present(yaml_path)
        pgm_present = self._present(pgm_path)
        issues: list[str] = []
        resolution = None
        origin = None

        if not yaml_present:
            issues.append('yaml_missing')
        if not pgm_present:
            issues.append('pgm_missing')
        if (
            (yaml_present and yaml_path.is_symlink())
            or (pgm_present and pgm_path.is_symlink())
        ):
            issues.append('symlink_not_allowed')

        if yaml_present and not yaml_path.is_symlink():
            try:
                document = yaml.safe_load(
                    yaml_path.read_text(encoding='utf-8')
                )
                if not isinstance(document, dict):
                    raise ValueError('map YAML must contain an object')
                image = document.get('image')
                expected_image = f'{name}.pgm'
                if (
                    not isinstance(image, str)
                    or Path(image).is_absolute()
                    or Path(image).name != image
                    or image != expected_image
                ):
                    issues.append('image_reference_invalid')
                resolution = document.get('resolution')
                origin = document.get('origin')
            except (OSError, UnicodeError, yaml.YAMLError, ValueError):
                issues.append('yaml_invalid')

        paths = [
            path
            for path in (yaml_path, pgm_path)
            if self._present(path) and not path.is_symlink()
        ]
        size_bytes = sum(path.stat().st_size for path in paths)
        modified_at = max(
            (path.stat().st_mtime for path in paths),
            default=None,
        )
        return {
            'name': name,
            'yaml_file': yaml_path.name if yaml_present else None,
            'pgm_file': pgm_path.name if pgm_present else None,
            'valid': not issues,
            'issues': issues,
            'is_default': name == self.default_name,
            'size_bytes': size_bytes,
            'modified_at': modified_at,
            'resolution': resolution,
            'origin': origin,
        }

    def list_maps(self) -> dict[str, Any]:
        if not self.maps_dir.exists():
            return {'maps': [], 'count': 0}
        names = {
            path.stem
            for path in self.maps_dir.iterdir()
            if path.suffix.lower() in {'.yaml', '.pgm'}
            and MAP_NAME_PATTERN.fullmatch(path.stem)
        }
        maps = [self._inspect(name) for name in sorted(names)]
        return {'maps': maps, 'count': len(maps)}

    def _require_mutable_name(self, name: str) -> tuple[Path, Path]:
        paths = self._paths(name)
        if name == self.default_name:
            raise MapManagerError(
                'default_map_protected',
                'the default map cannot be renamed or deleted',
                409,
            )
        if not any(self._present(path) for path in paths):
            raise MapManagerError('map_not_found', 'map not found', 404)
        if any(path.is_symlink() for path in paths):
            raise MapManagerError(
                'invalid_map_pair',
                'symbolic links are not allowed',
                409,
            )
        return paths

    def rename_map(self, name: str, new_name: str) -> dict[str, Any]:
        source_yaml, source_pgm = self._require_mutable_name(name)
        target_yaml, target_pgm = self._paths(new_name)
        if any(
            self._present(path)
            for path in (target_yaml, target_pgm)
        ):
            raise MapManagerError(
                'map_exists',
                'target map already exists',
                409,
            )

        source = self._inspect(name)
        if not source['valid']:
            raise MapManagerError(
                'invalid_map_pair',
                'only complete valid map pairs can be renamed',
                409,
            )

        original_yaml = source_yaml.read_bytes()
        yaml_moved = False
        pgm_moved = False
        try:
            source_yaml.rename(target_yaml)
            yaml_moved = True
            source_pgm.rename(target_pgm)
            pgm_moved = True
            document = yaml.safe_load(original_yaml)
            document['image'] = target_pgm.name
            target_yaml.write_text(
                yaml.safe_dump(document, sort_keys=False),
                encoding='utf-8',
            )
        except Exception as exc:
            try:
                if yaml_moved and target_yaml.exists():
                    target_yaml.write_bytes(original_yaml)
                if pgm_moved and target_pgm.exists():
                    target_pgm.rename(source_pgm)
                if yaml_moved and target_yaml.exists():
                    target_yaml.rename(source_yaml)
            except OSError:
                pass
            raise MapManagerError(
                'map_operation_failed',
                f'failed to rename map: {exc}',
                500,
            ) from exc

        return {
            'name': new_name,
            'yaml_file': target_yaml.name,
            'pgm_file': target_pgm.name,
            'renamed': [source_yaml.name, source_pgm.name],
        }

    def delete_map(self, name: str) -> dict[str, Any]:
        yaml_path, pgm_path = self._require_mutable_name(name)
        deleted = []
        try:
            for path in (yaml_path, pgm_path):
                if path.exists():
                    path.unlink()
                    deleted.append(path.name)
        except OSError as exc:
            raise MapManagerError(
                'map_operation_failed',
                f'failed to delete map: {exc}',
                500,
            ) from exc
        return {'name': name, 'deleted': deleted}

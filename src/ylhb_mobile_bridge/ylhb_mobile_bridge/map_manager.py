import base64
import io
import re
import time
from pathlib import Path
from typing import Any

from PIL import Image
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

    @staticmethod
    def _pgm_to_png_base64(
        pgm_path: Path,
        max_size_px: int = 1024,
    ) -> str:
        max_size_px = max(1, int(max_size_px))
        with Image.open(pgm_path) as source_image:
            image = source_image.convert('RGB')
            longest_edge = max(image.size)
            if longest_edge > max_size_px:
                scale = max_size_px / longest_edge
                size = (
                    max(1, int(image.width * scale)),
                    max(1, int(image.height * scale)),
                )
                nearest = getattr(
                    getattr(Image, 'Resampling', Image),
                    'NEAREST',
                )
                image = image.resize(size, resample=nearest)
            buffer = io.BytesIO()
            image.save(buffer, format='PNG', optimize=True)
        return base64.b64encode(buffer.getvalue()).decode('ascii')

    def preview_map(
        self,
        name: str,
        max_size_px: int = 1024,
    ) -> dict[str, Any]:
        yaml_path, pgm_path = self._paths(name)
        info = self._inspect(name)
        if not any(self._present(path) for path in (yaml_path, pgm_path)):
            raise MapManagerError('map_not_found', 'map not found', 404)
        if not info['valid']:
            raise MapManagerError(
                'invalid_map_pair',
                'only complete valid map pairs can be previewed',
                409,
            )
        try:
            png_base64 = self._pgm_to_png_base64(pgm_path, max_size_px)
        except Exception as exc:
            raise MapManagerError(
                'map_operation_failed',
                f'failed to preview map: {exc}',
                500,
            ) from exc
        return {
            'map_meta': {
                'name': info['name'],
                'yaml_file': info['yaml_file'],
                'pgm_file': info['pgm_file'],
                'resolution': info['resolution'],
                'origin': info['origin'],
                'size_bytes': info['size_bytes'],
                'modified_at': info['modified_at'],
                'is_default': info['is_default'],
            },
            'png_base64': png_base64,
        }

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

    def _archive_target(self, stem: str, suffix: str) -> Path:
        base = self.maps_dir / f'{stem}_{suffix}'
        candidate = base
        counter = 1
        while (
            self._present(candidate.with_suffix('.yaml'))
            or self._present(candidate.with_suffix('.pgm'))
            or self._present(candidate.with_suffix('.json'))
        ):
            candidate = self.maps_dir / f'{stem}_{suffix}_{counter}'
            counter += 1
        return candidate

    def _archive_route_targets(self, suffix: str) -> list[tuple[Path, Path]]:
        routes = sorted(self.maps_dir.glob('route_patrol_*.json'))
        targets: list[tuple[Path, Path]] = []
        for route_path in routes:
            target = self.maps_dir / f'deprecated_{route_path.stem}_{suffix}.json'
            counter = 1
            while self._present(target):
                target = (
                    self.maps_dir
                    / f'deprecated_{route_path.stem}_{suffix}_{counter}.json'
                )
                counter += 1
            targets.append((route_path, target))
        return targets

    def confirm_default(self, name: str) -> dict[str, Any]:
        source_yaml, source_pgm = self._paths(name)
        if name == self.default_name:
            info = self._inspect(name)
            if not info['valid']:
                raise MapManagerError(
                    'invalid_map_pair',
                    'default map pair is invalid',
                    409,
                )
            return {
                'changed': False,
                'default': info,
                'archived_previous_map': None,
                'archived_routes': [],
            }

        if not any(self._present(path) for path in (source_yaml, source_pgm)):
            raise MapManagerError('map_not_found', 'map not found', 404)
        source = self._inspect(name)
        if not source['valid']:
            raise MapManagerError(
                'invalid_map_pair',
                'only complete valid map pairs can become default',
                409,
            )

        default_yaml, default_pgm = self._paths(self.default_name)
        current_default = self._inspect(self.default_name)
        if not current_default['valid']:
            raise MapManagerError(
                'invalid_map_pair',
                'current default map pair is invalid',
                409,
            )

        suffix = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        archive_stem = self._archive_target(
            f'{self.default_name}_deprecated',
            suffix,
        )
        archive_yaml = archive_stem.with_suffix('.yaml')
        archive_pgm = archive_stem.with_suffix('.pgm')
        route_targets = self._archive_route_targets(suffix)
        moved: list[tuple[Path, Path]] = []
        original_source_yaml = source_yaml.read_bytes()
        original_default_yaml = default_yaml.read_bytes()

        def move(src: Path, dst: Path) -> None:
            if self._present(dst):
                raise MapManagerError(
                    'map_exists',
                    f'target file already exists: {dst.name}',
                    409,
                )
            src.rename(dst)
            moved.append((src, dst))

        try:
            move(default_yaml, archive_yaml)
            move(default_pgm, archive_pgm)
            move(source_yaml, default_yaml)
            move(source_pgm, default_pgm)
            archived_document = yaml.safe_load(original_default_yaml)
            archived_document['image'] = archive_pgm.name
            archive_yaml.write_text(
                yaml.safe_dump(archived_document, sort_keys=False),
                encoding='utf-8',
            )
            document = yaml.safe_load(original_source_yaml)
            document['image'] = default_pgm.name
            default_yaml.write_text(
                yaml.safe_dump(document, sort_keys=False),
                encoding='utf-8',
            )
            archived_routes = []
            for route_path, target in route_targets:
                move(route_path, target)
                archived_routes.append(
                    {'from': route_path.name, 'to': target.name}
                )
        except Exception as exc:
            self._rollback_moves(moved)
            if source_yaml.exists():
                source_yaml.write_bytes(original_source_yaml)
            if default_yaml.exists():
                default_yaml.write_bytes(original_default_yaml)
            if isinstance(exc, MapManagerError):
                raise
            raise MapManagerError(
                'map_operation_failed',
                f'failed to confirm default map: {exc}',
                500,
            ) from exc

        return {
            'changed': True,
            'default': self._inspect(self.default_name),
            'archived_previous_map': {
                'yaml_file': archive_yaml.name,
                'pgm_file': archive_pgm.name,
            },
            'archived_routes': archived_routes,
        }

    @staticmethod
    def _rollback_moves(moved: list[tuple[Path, Path]]) -> None:
        for original, current in reversed(moved):
            try:
                if current.exists() or current.is_symlink():
                    current.rename(original)
            except OSError:
                pass

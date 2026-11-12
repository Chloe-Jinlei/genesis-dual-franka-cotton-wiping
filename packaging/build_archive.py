"""Package all simulation evidence and pinned engine source; verify every byte.

Run with the original full scene directory, not a lightweight checkout missing
historical output. Does not load checkpoints or modify original scene files.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import tarfile
from datetime import datetime, timezone


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--original-scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    scene = args.original_scene.resolve()
    output = args.output.resolve()
    if output.exists():
        raise SystemExit('Output archive already exists; existing evidence is not overwritten.')
    if not (scene / 'output/contact_first_full/completion_validation.json').is_file():
        raise SystemExit('Expected the original scene with full completed-run evidence.')
    output.parent.mkdir(parents=True, exist_ok=True)
    skipped_dirs = {'.git', '.venv', '.cache', '__pycache__', '.pytest_cache'}
    skipped_suffixes = {'.pyc', '.pyo', '.nbc', '.nbi'}
    skipped_names = {'.DS_Store'}

    def keep(relative):
        return not (set(relative.parts) & skipped_dirs or
                    relative.suffix in skipped_suffixes or relative.name in skipped_names)

    payload = {}
    excluded_scene = []
    for file in sorted(scene.rglob('*')):
        relative = file.relative_to(scene)
        if file.is_symlink():
            raise ValueError(f'Unexpected scene symlink: {relative}')
        if not file.is_file():
            continue
        if keep(relative):
            payload['soy_cloth_scene/' + relative.as_posix()] = file
        else:
            excluded_scene.append(relative.as_posix())
    engine = json.loads((root / 'packaging/genesis-source-manifest.json').read_text())
    for relative, expected in engine['files'].items():
        file = root / 'genesis-world' / relative
        assert file.is_file() and not file.is_symlink()
        assert digest(file) == expected['sha256']
        payload['genesis-world/' + relative] = file
    generated = {'packaging/archive-manifest.json', 'packaging/release-assets.json'}
    for file in sorted(root.rglob('*')):
        relative = file.relative_to(root)
        if relative.parts[0] in {'soy_cloth_scene', 'genesis-world'} or not keep(relative):
            continue
        if relative.as_posix() in generated or not file.is_file():
            continue
        if file.is_symlink():
            raise ValueError(f'Unexpected packaging symlink: {relative}')
        payload[relative.as_posix()] = file

    entries = {name: {'bytes': file.stat().st_size, 'sha256': digest(file)}
               for name, file in sorted(payload.items())}
    manifest = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'All non-cache original scene files, including every historical output, plus pinned Genesis source and portable installation files.',
        'root_directory': root.name,
        'engine_commit': engine['commit'],
        'file_count_excluding_manifest': len(entries),
        'uncompressed_file_bytes_excluding_manifest': sum(x['bytes'] for x in entries.values()),
        'excluded_original_scene_cache_files': excluded_scene,
        'not_bundled': ['virtual environment binaries', 'machine-level installations', 'GPU/Python/Numba caches', 'Git object databases'],
        'self_hash_note': 'This manifest is included in the archive but does not hash itself. Release SHA256SUMS provides its digest.',
        'files': entries,
    }
    manifest_file = root / 'packaging/archive-manifest.json'
    manifest_file.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    payload['packaging/archive-manifest.json'] = manifest_file
    print(f'Packaging {len(payload)} files, {manifest["uncompressed_file_bytes_excluding_manifest"]:,} source bytes.', flush=True)
    with output.open('wb') as raw:
        with gzip.GzipFile(filename='', fileobj=raw, mode='wb', compresslevel=3, mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode='w|') as archive:
                for index, (name, file) in enumerate(sorted(payload.items()), 1):
                    archive.add(file, arcname=root.name + '/' + name, recursive=False)
                    if index % 1000 == 0:
                        print(f'Archived {index}/{len(payload)} files.', flush=True)
    expected_entries = dict(entries)
    expected_entries['packaging/archive-manifest.json'] = {
        'bytes': manifest_file.stat().st_size, 'sha256': digest(manifest_file)}
    verified = set()
    with tarfile.open(output, mode='r|gz') as archive:
        for member in archive:
            assert member.isfile(), member.name
            name = str(Path(member.name).relative_to(root.name))
            assert name not in verified and name in expected_entries, name
            expected = expected_entries[name]
            assert member.size == expected['bytes'], name
            source = archive.extractfile(member)
            checksum = hashlib.sha256()
            for block in iter(lambda: source.read(4 * 1024 * 1024), b''):
                checksum.update(block)
            assert checksum.hexdigest() == expected['sha256'], name
            verified.add(name)
    assert verified == set(expected_entries)
    checksum = digest(output)
    assets = {
        'archive': output.name, 'archive_bytes': output.stat().st_size,
        'archive_sha256': checksum, 'verified_file_count': len(verified),
        'all_archive_file_hashes_verified': True,
        'manifest_sha256': digest(manifest_file),
        'engine_commit': engine['commit'],
    }
    (root / 'packaging/release-assets.json').write_text(json.dumps(assets, indent=2) + '\n')
    (output.parent / 'archive-manifest.json').write_bytes(manifest_file.read_bytes())
    (output.parent / 'SHA256SUMS').write_text(
        f'{checksum}  {output.name}\n{digest(manifest_file)}  archive-manifest.json\n')
    print(json.dumps(assets, indent=2), flush=True)


if __name__ == '__main__':
    main()

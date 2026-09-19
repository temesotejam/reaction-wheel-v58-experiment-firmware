"""Verify the files of an extracted V58 package without hardware access."""
import hashlib
import json
from pathlib import Path


def verify(root):
    root = root.resolve()
    manifest = json.loads((root / 'PACKAGE_MANIFEST.json').read_text(encoding='utf-8'))
    checked = 0
    for item in manifest['files']:
        path = (root / item['path']).resolve()
        if root not in path.parents:
            raise ValueError('Unsafe manifest path: ' + item['path'])
        data = path.read_bytes()
        if len(data) != item['bytes'] or hashlib.sha256(data).hexdigest() != item['sha256']:
            raise ValueError('Mismatch: ' + item['path'])
        checked += 1
    print('PASS: {} payload files match PACKAGE_MANIFEST.json'.format(checked))
    print('This checks archive integrity, not physical hardware behavior.')
    return checked


if __name__ == '__main__':
    verify(Path(__file__).resolve().parent)

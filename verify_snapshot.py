"""Verify the public source archive without running an experiment."""
from pathlib import Path
from zipfile import ZipFile
import ast
import hashlib
import json

root = Path(__file__).resolve().parent
manifest = json.loads((root / "SOURCE_MANIFEST.json").read_text(encoding="utf-8"))["files"]
with ZipFile(root / "cavr_wbc_snapshot.zip") as archive:
    assert set(archive.namelist()) == set(manifest)
    assert archive.testzip() is None
    for name, expected in manifest.items():
        data = archive.read(name)
        assert hashlib.sha256(data).hexdigest() == expected, name
        ast.parse(data.decode("utf-8-sig"), filename=name)
print(f"Verified {len(manifest)} Python source files.")

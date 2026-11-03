"""Check the bootstrap environment and local source, using the standard library."""

import argparse
import hashlib
import json
from pathlib import Path
import platform
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-pdf", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / "docs/source_manifest.json").read_text(encoding="utf-8"))
    source = args.source_pdf or (root / manifest["relative_path"])
    required = [
        "README.md", "pyproject.toml",
        "configs/paper_ch3_reference.json", "docs/methods.md",
        "docs/experiments.md", "docs/sources.md",
        "src/berthing_planner/__init__.py", "tests/README.md", "data/README.md",
    ]
    missing = [name for name in required if not (root / name).is_file()]
    source_exists = source.is_file()
    digest = hashlib.sha256(source.read_bytes()).hexdigest() if source_exists else None
    version_ok = sys.version_info[:2] == (3, 12)
    source_ok = digest == manifest["sha256"] if source_exists else args.source_pdf is None
    ok = version_ok and not missing and source_ok
    result = {
        "status": "ok" if ok else "needs_attention",
        "check_scope": "bootstrap_only_not_algorithm_validation",
        "python": platform.python_version(),
        "python_executable": sys.executable,
        "python_version_ok": version_ok,
        "platform": platform.platform(),
        "project_root": str(root),
        "missing_project_files": missing,
        "source_pdf": str(source.resolve()),
        "source_exists": source_exists,
        "source_status": "verified" if digest == manifest["sha256"] else "optional_not_provided" if not source_exists and args.source_pdf is None else "mismatch_or_missing",
        "source_sha256": digest,
        "source_sha256_matches": digest == manifest["sha256"],
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

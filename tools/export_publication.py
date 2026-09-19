#!/usr/bin/env python3
"""Create a local, one-commit source snapshot; never configure a remote or push.

Only committed files are exported, respecting .gitattributes export-ignore.
Submodule gitlinks are preserved without copying their working directories.
The destination must not exist. The original repository is never rewritten.
"""
import argparse
from pathlib import Path
import subprocess
import sys


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args])


def export(source, destination):
    source = source.resolve()
    destination = destination.absolute()
    revision = git(source, "rev-parse", "HEAD").decode().strip()
    if git(source, "status", "--porcelain", "--untracked-files=no").strip():
        raise ValueError("commit tracked/index changes before exporting")
    subprocess.run([sys.executable, str(source / "tools/check_publication.py")],
                   cwd=source, check=True)
    # mkdir is exclusive; no overwrite or deletion of a pre-existing destination.
    destination.mkdir(parents=False, exist_ok=False)
    archive = subprocess.Popen(["git", "-C", str(source), "archive", "HEAD"],
                               stdout=subprocess.PIPE)
    try:
        subprocess.run(["tar", "-xf", "-", "-C", str(destination)],
                       stdin=archive.stdout, check=True)
    finally:
        archive.stdout.close()
        archive_status = archive.wait()
    if archive_status:
        raise RuntimeError("git archive failed; incomplete destination retained for inspection")
    git(destination, "init", "--initial-branch=main")
    git(destination, "add", "--all")
    for entry in git(source, "ls-tree", "-rz", "HEAD").split(b"\0"):
        if not entry:
            continue
        metadata, name = entry.split(b"\t", 1)
        mode, kind, oid = metadata.decode().split()
        if mode == "160000":
            git(destination, "update-index", "--add", "--cacheinfo",
                mode, oid, name.decode())
    subprocess.run([sys.executable, str(destination / "tools/check_publication.py")],
                   cwd=destination, check=True)
    git(destination, "commit", "-m", f"Initial source release (development revision {revision})")
    if git(destination, "rev-list", "--count", "HEAD").strip() != b"1":
        raise RuntimeError("snapshot unexpectedly has history")
    if git(destination, "remote").strip():
        raise RuntimeError("snapshot unexpectedly has a remote")
    print(f"Created {destination}: one commit, no remote, CPU gitlink retained.")
    print("Next: git submodule update --init --recursive; supply firmware; run release gates.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        export(args.source, args.destination)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Export failed: {error}\nNo existing repository was rewritten.\n")


if __name__ == "__main__":
    main()

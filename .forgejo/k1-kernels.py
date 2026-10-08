#!/usr/bin/env python3
"""Cross-build the SpacemiT K1 kernels (R2SPROD, RV2PROD) on a Linux runner.

Follows the Linux cross-build of freebsd-img-maker's build-freebsd.sh:
tools/build/make.py with the host's clang/lld (--cross-bindir=/usr/bin),
kernel-toolchain, buildkernel, then installkernel -DNO_ROOT into one stage
directory per configuration. Unprivileged; nothing here needs root.

The checked-out commit is copied into a persistent source tree under the
cache directory (default ~/.cache/freebsd-k1-kernels) and built with
--no-clean against a persistent MAKEOBJDIRPREFIX there, so later runs on the
same runner are incremental: git rewrites only the files that changed.

For each configuration OUT/<KERNCONF>/ receives
  <KERNCONF>.tar.xz        boot/kernel (kernel and modules) and METALOG
  <KERNCONF>-debug.tar.xz  usr/lib/debug/boot/kernel (separate debug files)
  BUILD                    source commit/tree, configuration, host tools
  SHA256SUMS               of the two archives and of boot/kernel/kernel
"""

import argparse
import fcntl
import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

KERNCONFS = ("R2SPROD", "RV2PROD")
# Host commands the build needs, with the Fedora package that provides each:
# the cross toolchain, and the tools tools/build/Makefile links into the
# bootstrap tree (_host_tools_to_symlink, /bin/bash).
HOST_TOOLS = {
    "clang": "clang",
    "ld.lld": "lld",
    "llvm-ar": "llvm",
    "llvm-nm": "llvm",
    "llvm-objcopy": "llvm",
    "cc": "gcc",
    "git": "git",
    "tar": "tar",
    "/bin/bash": "bash",
    "bzip2": "bzip2",
    "bunzip2": "bzip2",
    "cmp": "diffutils",
    "find": "findutils",
    "gzip": "gzip",
    "gunzip": "gzip",
    "hostname": "hostname",
    "patch": "patch",
    "time": "time",
    "which": "which",
    "xz": "xz",
    "unxz": "xz",
}
DATE_SHIM = """#!/bin/sh
# The bootstrap uses BSD date -ur EPOCH; translate it for GNU date.
if [ "$1" = "-ur" ]; then
    shift
    epoch=$1
    shift
    exec /bin/date -u -d "@$epoch" "$@"
fi
exec /bin/date "$@"
"""


def run(cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kw)


def output(cmd, **kw):
    return subprocess.run(
        [str(c) for c in cmd], check=True, capture_output=True, text=True, **kw
    ).stdout.strip()


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def sync_source(checkout, src):
    """Make src a detached checkout of checkout's HEAD, touching only changes."""
    if not (src / ".git").exists():
        src.mkdir(parents=True, exist_ok=True)
        run(["git", "-C", src, "init", "-q"])
    head = output(["git", "-C", checkout, "rev-parse", "HEAD"])
    run(
        ["git", "-C", src, "fetch", "-q", "--no-tags", "--depth", "1", checkout, "HEAD"]
    )
    run(["git", "-C", src, "checkout", "-q", "--force", "--detach", "FETCH_HEAD"])
    if output(["git", "-C", src, "rev-parse", "HEAD"]) != head:
        sys.exit(f"k1-kernels: {src} is not at {head}")
    run(["git", "-C", src, "clean", "-q", "-ffdx"])
    return head, output(["git", "-C", src, "rev-parse", "HEAD^{tree}"])


def tar(stage, archive, members, epoch):
    run(
        [
            "tar",
            "--sort=name",
            "--owner=0",
            "--group=0",
            "--numeric-owner",
            f"--mtime=@{epoch}",
            "-C",
            stage,
            "-cJf",
            archive,
            *members,
        ],
        env={**os.environ, "XZ_OPT": "-T0 -6"},
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, type=Path, help="artifact directory")
    ap.add_argument(
        "--cache",
        type=Path,
        default=Path.home() / ".cache" / "freebsd-k1-kernels",
        help="persistent source and object directory",
    )
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    ap.add_argument("--kernconf", action="append", choices=KERNCONFS)
    args = ap.parse_args()
    kernconfs = args.kernconf or list(KERNCONFS)
    # make runs in the source tree: DESTDIR must be absolute.
    args.out = args.out.resolve()
    checkout = Path.cwd()

    missing = {
        tool: pkg
        for tool, pkg in HOST_TOOLS.items()
        if shutil.which(tool, path="/usr/bin:/bin") is None
    }
    if missing:
        sys.exit(
            "k1-kernels: missing host tools: "
            + " ".join(missing)
            + "; Fedora packages: "
            + " ".join(sorted(set(missing.values())))
        )

    args.cache.mkdir(parents=True, exist_ok=True)
    lock = open(args.cache / "lock", "w")
    fcntl.flock(lock, fcntl.LOCK_EX)

    src, obj, tools = (args.cache / d for d in ("src", "obj", "host-tools"))
    commit, tree = sync_source(checkout, src)
    epoch = output(["git", "-C", src, "log", "-1", "--format=%ct"])
    tools.mkdir(exist_ok=True)
    (tools / "date").write_text(DATE_SHIM)
    (tools / "date").chmod(0o755)
    obj.mkdir(exist_ok=True)

    env = {
        **os.environ,
        "MAKEOBJDIRPREFIX": str(obj),
        "SRCCONF": "/dev/null",
        "__MAKE_CONF": "/dev/null",
        "PATH": f"{tools}:/usr/bin:/bin",
    }

    def make(*targets):
        run(
            [
                "python3",
                "tools/build/make.py",
                "--no-clean",
                "--cross-bindir=/usr/bin",
                f"-j{args.jobs}",
                "TARGET=riscv",
                "TARGET_ARCH=riscv64",
                "NEWVERS_ARGS=-r",
                "-DWITH_REPRODUCIBLE_BUILD",
                "-DWITH_REPRODUCIBLE_PATHS",
                *targets,
            ],
            cwd=src,
            env=env,
            stdin=subprocess.DEVNULL,
        )

    print(f"==> {commit} (tree {tree}) in {src}", flush=True)
    print("==> kernel-toolchain", flush=True)
    make("kernel-toolchain")
    print(f"==> buildkernel {' '.join(kernconfs)}", flush=True)
    make(f"KERNCONF={' '.join(kernconfs)}", "buildkernel")

    host = {
        "clang": output(["clang", "--version"]).splitlines()[0],
        "ld.lld": output(["ld.lld", "--version"]).splitlines()[0],
    }
    for conf in kernconfs:
        stage = args.out / "stage" / conf
        dest = args.out / conf
        shutil.rmtree(stage, ignore_errors=True)
        shutil.rmtree(dest, ignore_errors=True)
        stage.mkdir(parents=True)
        dest.mkdir(parents=True)
        print(f"==> installkernel {conf}", flush=True)
        make("-DNO_ROOT", f"DESTDIR={stage}", f"KERNCONF={conf}", "installkernel")
        kernel = stage / "boot" / "kernel" / "kernel"
        if not kernel.is_file() or not (stage / "METALOG").is_file():
            sys.exit(f"k1-kernels: {conf}: no kernel or METALOG in {stage}")
        tar(stage, dest / f"{conf}.tar.xz", ["METALOG", "boot"], epoch)
        debug = [] if not (stage / "usr").exists() else ["usr"]
        if debug:
            tar(stage, dest / f"{conf}-debug.tar.xz", debug, epoch)
        (dest / "BUILD").write_text(
            f"kernconf={conf}\n"
            f"freebsd_commit={commit}\n"
            f"freebsd_tree={tree}\n"
            f"source_date_epoch={epoch}\n"
            f"compiler={host['clang']}\n"
            f"linker={host['ld.lld']}\n"
            f"modules={len(list((stage / 'boot' / 'kernel').glob('*.ko')))}\n"
        )
        sums = {p.name: sha256(p) for p in sorted(dest.glob("*.tar.xz"))}
        sums["boot/kernel/kernel"] = sha256(kernel)
        text = "".join(f"{h}  {n}\n" for n, h in sums.items())
        (dest / "SHA256SUMS").write_text(text)
        print(f"== {conf}\n{text}", end="", flush=True)
    shutil.rmtree(args.out / "stage")


if __name__ == "__main__":
    main()

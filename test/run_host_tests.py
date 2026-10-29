#!/usr/bin/env python3
"""Build and run the incunest_afe4490 host unit tests (Unity, one executable per test/test_*/ suite).

    python test/run_host_tests.py                     # configure + build + run every suite
    python test/run_host_tests.py hr1 spo2            # only suites whose name contains a filter
    python test/run_host_tests.py --clean             # wipe the build directory first
    python test/run_host_tests.py --build <dir>       # build somewhere else (default: test/build)

Needs cmake, ninja and a host g++ on PATH (MinGW-w64 on Windows). Exit code = number of failing
suites. GitHub Actions runs it on every push and tag (.github/workflows/tests.yml); PulseNest's
test_lib/runner/run_host_tests.py calls it with --build pointing into PulseNest.
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def run(cmd, **kw):
    print("$ " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, **kw)


def main(argv):
    build = os.path.join(HERE, "build")
    if "--build" in argv:
        i = argv.index("--build")
        if i + 1 >= len(argv):
            print("error: --build needs a directory", file=sys.stderr)
            return 1
        build = os.path.abspath(argv[i + 1])
        argv = argv[:i] + argv[i + 2:]
    filters = [a for a in argv if not a.startswith("--")]
    if "--clean" in argv and os.path.isdir(build):
        shutil.rmtree(build)
    for tool in ("cmake", "ninja"):
        if shutil.which(tool) is None:
            print(f"error: '{tool}' not found on PATH", file=sys.stderr)
            return 1
    if not os.path.exists(os.path.join(build, "build.ninja")):
        r = run(["cmake", "-S", HERE, "-B", build, "-G", "Ninja", "-DCMAKE_BUILD_TYPE=Release"])
        if r.returncode:
            return 1
    r = run(["cmake", "--build", build])
    if r.returncode:
        return 1

    suites = sorted(d for d in os.listdir(HERE) if d.startswith("test_") and os.path.isdir(os.path.join(HERE, d)))
    if filters:
        suites = [s for s in suites if any(f in s for f in filters)]
    failed = []
    print()
    for suite in suites:
        exe = os.path.join(build, suite + (".exe" if os.name == "nt" else ""))
        if not os.path.exists(exe):
            print(f"{suite:20s} MISSING (did not build)")
            failed.append(suite)
            continue
        p = subprocess.run([exe], capture_output=True, text=True)
        lines = [l for l in p.stdout.splitlines() if l.strip()]
        summary = next((l for l in reversed(lines) if "Tests" in l and "Failures" in l), "(no Unity summary)")
        verdict = "OK" if p.returncode == 0 else f"FAIL (exit {p.returncode})"
        print(f"{suite:20s} {verdict:16s} {summary}")
        if p.returncode != 0:
            failed.append(suite)
            for l in lines:
                if ":FAIL" in l or l.startswith("FAIL"):
                    print("    " + l)
            if p.stderr.strip():
                print("    stderr: " + p.stderr.strip()[:500])
    print()
    print(f"{len(suites) - len(failed)}/{len(suites)} suites OK" + (f" — failing: {', '.join(failed)}" if failed else ""))
    return len(failed)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

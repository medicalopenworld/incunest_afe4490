#!/usr/bin/env python3
"""Check that every place carrying the library version says the same thing, and, for a tag, that
they all say the tag.

    python scripts/check_version.py              # all places agree with each other
    python scripts/check_version.py v0.111       # ... and with this tag
    python scripts/check_version.py --tag-from-ref refs/tags/v0.111
                                                 # CI: the tag when the ref is a tag, else no tag

Places (all must name the same version, e.g. 0.111 or 0.90b):
  incunest_afe4490.h          #define INCUNEST_AFE4490_VERSION "0.111" and "// Library version: v0.111"
  incunest_afe4490.cpp        "// Library version: v0.111"
  examples/basic/main.cpp     "// Library version: v0.111"
  incunest_afe4490_spec.md    title "# incunest_afe4490 — Specification v0.111" and the first row of
                              the §14 version history table, which must also carry a date (YYYY-MM-DD)
  library.json                "version": "0.111.0" (semantic version: 0.111 -> 0.111.0, 0.90b -> 0.90.1)

Exit code 0 when everything agrees. Run by .github/workflows/tests.yml before the unit tests.
"""
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VERSION_RE = r"(\d+\.\d+[a-z]?)"


def read(rel):
    with open(os.path.join(ROOT, rel), encoding="utf-8") as f:
        return f.read()


def find(rel, pattern, flags=0):
    m = re.search(pattern, read(rel), flags)
    return m.group(1) if m else None


def semver(version):
    """0.111 -> 0.111.0; 0.90b -> 0.90.1 (the letter of a quick fix is the patch number)."""
    m = re.fullmatch(r"(\d+)\.(\d+)([a-z]?)", version)
    if not m:
        return None
    patch = ord(m.group(3)) - ord("a") if m.group(3) else 0
    return f"{m.group(1)}.{m.group(2)}.{patch}"


def main(argv):
    tag = None
    if argv[:1] == ["--tag-from-ref"]:
        ref = argv[1] if len(argv) > 1 else ""
        if ref.startswith("refs/tags/"):
            tag = ref[len("refs/tags/"):]
    elif argv:
        tag = argv[0]

    places = {
        "incunest_afe4490.h #define": find("incunest_afe4490.h", r'#define\s+INCUNEST_AFE4490_VERSION\s+"' + VERSION_RE + '"'),
        "incunest_afe4490.h header comment": find("incunest_afe4490.h", r"^// Library version: v" + VERSION_RE, re.M),
        "incunest_afe4490.cpp header comment": find("incunest_afe4490.cpp", r"^// Library version: v" + VERSION_RE, re.M),
        "examples/basic/main.cpp header comment": find("examples/basic/main.cpp", r"^// Library version: v" + VERSION_RE, re.M),
        "spec title": find("incunest_afe4490_spec.md", r"^# incunest_afe4490 — Specification v" + VERSION_RE + r"\s*$", re.M),
        "spec section 14 first row": find("incunest_afe4490_spec.md", r"^## 14\. Version history$.*?^\|\s*v" + VERSION_RE + r"\s*\|", re.M | re.S),
    }
    lib_json = json.loads(read("library.json")).get("version")
    # The first row of section 14 is the date of the specification: version | date | description.
    first_row = re.search(r"^## 14\. Version history$.*?^\|\s*v\d+\.\d+[a-z]?\s*\|([^|\n]*)\|",
                          read("incunest_afe4490_spec.md"), re.M | re.S)
    first_row_date = first_row and re.fullmatch(r"\s*(\d{4}-\d{2}-\d{2})\s*", first_row.group(1))
    first_row_date = first_row_date and first_row_date.group(1)

    expected = places["incunest_afe4490.h #define"]
    if tag is not None:
        if not re.fullmatch("v" + VERSION_RE, tag):
            print(f"error: tag '{tag}' is not a library version tag (vX.Y or vX.Yb)", file=sys.stderr)
            return 1
        expected = tag[1:]

    errors = []
    for name, found in places.items():
        if found != expected:
            errors.append(f"{name}: {'not found' if found is None else 'v' + found} (expected v{expected})")
    if expected is None or lib_json != semver(expected):
        errors.append(f"library.json version: {lib_json} (expected {expected and semver(expected)})")
    if first_row_date is None:
        errors.append("spec section 14 first row: no date (YYYY-MM-DD) in the Date column")

    if errors:
        print(f"version check FAILED{' against tag ' + tag if tag else ''}:", file=sys.stderr)
        for e in errors:
            print("  " + e, file=sys.stderr)
        return 1
    print(f"version check OK: v{expected} everywhere (library.json {lib_json}, spec dated {first_row_date})"
          + (f", matches tag {tag}" if tag else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

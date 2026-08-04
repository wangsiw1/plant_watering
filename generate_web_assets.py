Import("env")

import gzip
from pathlib import Path


ASSETS = (
    ("root.html", "ROOT_HTML_GZ"),
    ("ota.html", "OTA_HTML_GZ"),
)


def byte_array(data):
    lines = []
    for offset in range(0, len(data), 16):
        chunk = data[offset : offset + 16]
        lines.append("    " + ", ".join(f"0x{value:02x}" for value in chunk) + ",")
    return "\n".join(lines)


project_dir = Path(env.subst("$PROJECT_DIR"))
build_dir = Path(env.subst("$BUILD_DIR"))
web_dir = project_dir / "web"
generated_path = build_dir / "generated_web_assets.h"

sections = [
    "#pragma once",
    "",
    "#include <stddef.h>",
    "#include <stdint.h>",
    "",
    "// Generated from main/web/*.html. Do not edit this build artifact.",
]

for filename, symbol in ASSETS:
    source_path = web_dir / filename
    if not source_path.is_file():
        raise RuntimeError(f"Required web asset is missing: {source_path}")
    raw = source_path.read_bytes()
    if not raw:
        raise RuntimeError(f"Required web asset is empty: {source_path}")

    compressed = bytearray(gzip.compress(raw, compresslevel=9, mtime=0))
    # Python versions before 3.13 can emit a platform-specific gzip OS byte.
    compressed[9] = 255
    compressed = bytes(compressed)
    if gzip.decompress(compressed) != raw:
        raise RuntimeError(f"Generated gzip verification failed: {source_path}")

    sections.extend(
        (
            "",
            f"static const uint8_t {symbol}[] = {{",
            byte_array(compressed),
            "};",
            f"static constexpr size_t {symbol}_LEN = sizeof({symbol});",
        )
    )
    print(f"Web asset: {filename} raw={len(raw)} bytes gzip={len(compressed)} bytes")

generated = ("\n".join(sections) + "\n").encode("ascii")
build_dir.mkdir(parents=True, exist_ok=True)
if not generated_path.exists() or generated_path.read_bytes() != generated:
    generated_path.write_bytes(generated)

env.Append(CPPPATH=[str(build_dir)])

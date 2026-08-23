Import("env")

import gzip
import json
import re
from pathlib import Path


ASSETS = (
    ("root.html", "ROOT_HTML_GZ"),
    ("ota.html", "OTA_HTML_GZ"),
)
LOCALES = ("en", "zh-CN")
LOCALE_MARKER = "/*__I18N_MESSAGES__*/"


def load_json_object(path):
    def reject_duplicate_keys(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate translation key {key!r} in {path}")
            result[key] = value
        return result

    try:
        with path.open("r", encoding="utf-8") as source:
            value = json.load(source, object_pairs_hook=reject_duplicate_keys)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        raise RuntimeError(f"Invalid locale file {path}: {error}") from error
    if not isinstance(value, dict):
        raise RuntimeError(f"Locale file must contain a JSON object: {path}")
    non_strings = sorted(key for key, text in value.items()
                         if not isinstance(key, str) or not isinstance(text, str))
    if non_strings:
        raise RuntimeError(
            f"Locale values must be strings in {path}: {', '.join(non_strings)}")
    return value


def build_locale_script(locales_dir):
    catalogs = {
        locale: load_json_object(locales_dir / f"{locale}.json")
        for locale in LOCALES
    }
    english_keys = set(catalogs["en"])
    for locale in LOCALES[1:]:
        locale_keys = set(catalogs[locale])
        missing = sorted(english_keys - locale_keys)
        extra = sorted(locale_keys - english_keys)
        if missing or extra:
            details = []
            if missing:
                details.append("missing: " + ", ".join(missing))
            if extra:
                details.append("extra: " + ", ".join(extra))
            raise RuntimeError(
                f"Locale key mismatch for {locale}: {'; '.join(details)}")
        placeholder_mismatches = []
        for key in sorted(english_keys):
            english_placeholders = set(re.findall(r"\{(\w+)\}", catalogs["en"][key]))
            locale_placeholders = set(re.findall(r"\{(\w+)\}", catalogs[locale][key]))
            if english_placeholders != locale_placeholders:
                placeholder_mismatches.append(key)
        if placeholder_mismatches:
            raise RuntimeError(
                f"Locale placeholder mismatch for {locale}: "
                + ", ".join(placeholder_mismatches))
    encoded = json.dumps(catalogs, ensure_ascii=False,
                         separators=(",", ":"), sort_keys=True)
    encoded = (encoded.replace("<", "\\u003c")
               .replace("\u2028", "\\u2028")
               .replace("\u2029", "\\u2029"))
    return f"const I18N_MESSAGES={encoded};"


def byte_array(data):
    lines = []
    for offset in range(0, len(data), 16):
        chunk = data[offset : offset + 16]
        lines.append("    " + ", ".join(f"0x{value:02x}" for value in chunk) + ",")
    return "\n".join(lines)


project_dir = Path(env.subst("$PROJECT_DIR"))
build_dir = Path(env.subst("$BUILD_DIR"))
web_dir = project_dir / "web"
locale_script = build_locale_script(web_dir / "locales")
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
    try:
        source = source_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise RuntimeError(f"Unable to read web asset {source_path}: {error}") from error
    marker_count = source.count(LOCALE_MARKER)
    if marker_count != 1:
        raise RuntimeError(
            f"Expected exactly one locale marker in {source_path}, found {marker_count}")
    raw = source.replace(LOCALE_MARKER, locale_script).encode("utf-8")
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

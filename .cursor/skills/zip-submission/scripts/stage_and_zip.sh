#!/usr/bin/env bash
# Stage the junk-audit include list, LF-normalize text, zip from the stage root.
# Run inside drone-mapper-ex3-dev with the repo bind-mounted at /work.
set -euo pipefail

REPO="${REPO:-/work}"
STAGE="${STAGE:-/tmp/ex3_stage}"
EXTRACT="${EXTRACT:-/tmp/ex3_unzipped}"
ZIP_NAME="${ZIP_NAME:-ex3_207190406_209543255.zip}"
ZIP_PATH="${ZIP_PATH:-${REPO}/${ZIP_NAME}}"
SKIP_BUILD="${SKIP_BUILD:-0}"
STRICT_INPUTS="${STRICT_INPUTS:-0}"

need_root=(
  CMakeLists.txt CMakePresets.json vcpkg.json vcpkg-configuration.json
  README.md students.txt HLD.pdf bonus.txt "Known Issues.xlsx"
)
for f in "${need_root[@]}"; do
  if [[ ! -f "${REPO}/${f}" ]]; then
    echo "MISSING_ROOT_FILE ${f}" >&2
    exit 1
  fi
done
if grep -q 'TODO:' "${REPO}/students.txt"; then
  echo "students.txt still has TODO:" >&2
  exit 1
fi

rm -rf "${STAGE}" "${EXTRACT}"
mkdir -p "${STAGE}"

rsync -a --exclude '.gitkeep' \
  "${REPO}/Algorithm/" "${STAGE}/Algorithm/"
rsync -a --exclude '.gitkeep' \
  "${REPO}/MissionControl/" "${STAGE}/MissionControl/"
rsync -a --exclude '.gitkeep' \
  "${REPO}/UserCommon/" "${STAGE}/UserCommon/"
rsync -a \
  "${REPO}/common/" "${STAGE}/common/"
rsync -a \
  --exclude '.gitkeep' \
  --exclude 'tests/manual/' \
  --exclude 'tests/fixtures/adversarial_plugins_ASSUMPTIONS.md' \
  --exclude 'tests/fixtures/foreign_hits_only_mission_control_ASSUMPTIONS.md' \
  --exclude 'tests/fixtures/baseline_lawnmower_algorithm_ASSUMPTIONS.md' \
  --exclude 'tests/fixtures/tiny_compose.yaml' \
  --exclude 'tests/fixtures/tiny_compose_adversarial.yaml' \
  --exclude 'tests/fixtures/tiny_mission_adversarial.yaml' \
  --exclude 'tests/hosts/skeleton_host/ASSUMPTIONS.md' \
  --exclude 'tests/hosts/skeleton_host/CMakeLists.txt' \
  "${REPO}/Simulator/" "${STAGE}/Simulator/"

inputs_exclude=(--exclude 'profile_cell.yaml')
if [[ "${STRICT_INPUTS}" == "1" ]]; then
  inputs_exclude+=(--exclude 'map/*.cw' --exclude 'npy_to_cw.py')
fi
rsync -a "${inputs_exclude[@]}" \
  "${REPO}/inputs/" "${STAGE}/inputs/"

cp -a \
  "${REPO}/CMakeLists.txt" \
  "${REPO}/CMakePresets.json" \
  "${REPO}/vcpkg.json" \
  "${REPO}/vcpkg-configuration.json" \
  "${REPO}/README.md" \
  "${REPO}/students.txt" \
  "${REPO}/HLD.pdf" \
  "${REPO}/bonus.txt" \
  "${REPO}/Known Issues.xlsx" \
  "${STAGE}/"

# LF-normalize text; leave binary maps/pdf/xlsx alone. Stage only — not the repo.
find "${STAGE}" -type f \
  ! -name '*.npy' ! -name '*.pdf' ! -name '*.xlsx' \
  -print0 | xargs -0 sed -i 's/\r$//'

rm -f "${ZIP_PATH}"
if command -v zip >/dev/null 2>&1; then
  ( cd "${STAGE}" && zip -r "${ZIP_PATH}" . -x '*.DS_Store' )
else
  python3 - <<PY
import zipfile
from pathlib import Path
stage = Path("${STAGE}")
out = Path("${ZIP_PATH}")
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zf:
    for p in stage.rglob("*"):
        if p.is_file() and p.name != ".DS_Store":
            zf.write(p, p.relative_to(stage).as_posix())
print("wrote", out, "via python zipfile")
PY
fi

python3 - <<PY
import sys
import zipfile
from pathlib import Path

zpath = Path("${ZIP_PATH}")
if zpath.name != "${ZIP_NAME}":
    print("ZIP-01 FAIL name", zpath.name)
    sys.exit(1)
zf = zipfile.ZipFile(zpath)
names = zf.namelist()
errors = []
if any(chr(92) in n for n in names):
    errors.append("backslash in zip paths")
roots = sorted({n.split("/")[0] for n in names if n and not n.startswith("__")})
need_dirs = {"Algorithm", "MissionControl", "Simulator", "common", "UserCommon", "inputs"}
need_files = {
    "CMakeLists.txt", "CMakePresets.json", "vcpkg.json", "vcpkg-configuration.json",
    "README.md", "students.txt", "HLD.pdf", "bonus.txt", "Known Issues.xlsx",
}
if not need_dirs.issubset(roots):
    errors.append(f"missing folders: {need_dirs - set(roots)}")
if not need_files.issubset(roots):
    errors.append(f"missing root files: {need_files - set(roots)}")
if "${ZIP_NAME%.zip}" in roots:
    errors.append("extra wrapper folder in zip")
forbid = (
    ".cursor", "AGENTS.md", "docs", "context", "scripts", "build", "tmp",
    ".cache", ".venv", ".git", ".vscode", ".superpowers", "compile_commands.json",
)
hit = [n for n in names if n.split("/")[0] in forbid or n.startswith(".git/")]
if hit:
    errors.append(f"junk in zip: {hit[:12]}")
if any(n.endswith((".so", ".o", ".exe", ".dll")) for n in names):
    errors.append("binaries in zip")
if "inputs/profile_cell.yaml" in names:
    errors.append("profile_cell.yaml should be omitted")
if "${STRICT_INPUTS}" == "1":
    extra = [n for n in names if n.endswith(".cw") or n.endswith("npy_to_cw.py")]
    if extra:
        errors.append(f"strict inputs still present: {extra}")
if any(n == "UserCommon/CMakeLists.txt" for n in names):
    errors.append("UserCommon has CMakeLists.txt")
omit = [
    "Simulator/tests/manual/",
    "Simulator/tests/hosts/skeleton_host/CMakeLists.txt",
    "Simulator/tests/hosts/skeleton_host/ASSUMPTIONS.md",
]
for pref in omit:
    hits = [n for n in names if n.startswith(pref) or n == pref.rstrip("/")]
    if hits:
        errors.append(f"should omit {hits[:8]}")
print("zip_entries", len(names))
print("zip_roots", roots)
for n in names[:40]:
    print(n)
print(f"... {len(names)} entries total")
if errors:
    print("ZIP_CHECK_FAIL", errors)
    sys.exit(1)
print("ZIP_CHECK_PASS")
PY

mkdir -p "${EXTRACT}"
python3 - <<PY
import zipfile
from pathlib import Path
dest = Path("${EXTRACT}")
with zipfile.ZipFile("${ZIP_PATH}") as zf:
    zf.extractall(dest)
print("extracted to", dest)
PY

echo "===== CRLF remaining in extracted text ====="
cr_hits=$(grep -rIl $'\r' "${EXTRACT}" --exclude='*.npy' --exclude='*.pdf' --exclude='*.xlsx' || true)
if [ -n "${cr_hits}" ]; then
  echo "CRLF_FAIL"
  echo "${cr_hits}"
  exit 1
fi
echo "CRLF_PASS (none)"

echo "===== encoding on root text ====="
python3 - <<PY
from pathlib import Path
root = Path("${EXTRACT}")
errors = []
for name in ["students.txt", "README.md", "CMakeLists.txt", "bonus.txt"]:
    data = (root / name).read_bytes()
    if data.startswith(bytes([0xFF, 0xFE])) or data.startswith(bytes([0xFE, 0xFF])):
        errors.append(f"{name}: UTF-16")
    if data.startswith(bytes([0xEF, 0xBB, 0xBF])):
        errors.append(f"{name}: UTF-8 BOM")
    try:
        data.decode("utf-8")
    except UnicodeDecodeError as e:
        errors.append(f"{name}: not utf-8 ({e})")
    else:
        print(f"{name}: UTF-8, no BOM, {len(data)} bytes")
if errors:
    print("ENCODING_FAIL", errors)
    raise SystemExit(1)
print("ENCODING_PASS")
PY

echo "===== binaries in extract ====="
bins=$(find "${EXTRACT}" \( -name '*.so' -o -name '*.o' -o -name '*.exe' -o -name '*.dll' \) || true)
if [ -n "${bins}" ]; then
  echo "BINARY_FAIL"
  echo "${bins}"
  exit 1
fi
echo "BINARY_PASS (none)"

if [[ "${SKIP_BUILD}" == "1" ]]; then
  echo "EXTRACT_BUILD_SKIP"
  echo "ZIP_READY ${ZIP_PATH}"
  exit 0
fi

echo "===== cmake from extracted zip ====="
cd "${EXTRACT}"
cmake --preset default
cmake --build --preset default
echo "EXTRACT_BUILD_PASS"
echo "ZIP_READY ${ZIP_PATH}"

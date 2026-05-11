#!/usr/bin/env bash
set -euo pipefail

# Run tile_top_formal via pre-staged per-task workdirs.
# This bypasses SymbiYosys setup/copy path handling by preparing src/ ourselves
# and then invoking "sby <workdir>" in directory mode.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SBY_FILE="${SCRIPT_DIR}/tile_top_formal.sby"
OUT_ROOT="${SCRIPT_DIR}/.prestaged"

TASKS=(
  reset_quiescence
  ena_disabled_no_output
  host_priority_mux
  noc_yields_to_host
  output_valid_stable_until_ready
  host_payload_stable
  noc_payload_stable
)

if [[ ${#} -gt 0 ]]; then
  TASKS=("$@")
fi

mkdir -p "${OUT_ROOT}"

for task in "${TASKS[@]}"; do
  workdir="${OUT_ROOT}/tile_top_formal_${task}"
  cfg="${workdir}/config.sby"
  srcdir="${workdir}/src"

  rm -rf "${workdir}"
  mkdir -p "${srcdir}"

  # 1) Emit the fully preprocessed one-task config.
  sby --dumpcfg "${SBY_FILE}" "${task}" > "${cfg}"

  # 2) Copy required sources to src/ using basename layout.
  # Parse [files] from the preprocessed config to avoid --dumpfiles
  # type-conversion regressions seen in this toolchain.
  while IFS= read -r path_entry; do
    [[ -z "${path_entry}" ]] && continue
    if [[ "${path_entry}" = /* ]]; then
      srcpath="${path_entry}"
    else
      srcpath="${SCRIPT_DIR}/${path_entry}"
    fi
    cp -f "${srcpath}" "${srcdir}/$(basename "${path_entry}")"
  done < <(
    awk '
      BEGIN { in_files=0 }
      /^\[files\]/ { in_files=1; next }
      /^\[/ { in_files=0 }
      in_files {
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", $0)
        if ($0 != "") print $0
      }
    ' "${cfg}"
  )

  # 3) Execute in directory mode (no setup copy phase).
  echo "===== PRESTAGED ${task} ====="
  sby "${workdir}"
done

echo "All requested tile_top_formal prestaged tasks completed."

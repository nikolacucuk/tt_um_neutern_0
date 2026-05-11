#!/usr/bin/env bash
set -euo pipefail

# Generic pre-staged SymbiYosys runner.
# Usage:
#   ./run_tile_formal_prestaged.sh <job.sby> [task ...]
# Examples:
#   ./run_tile_formal_prestaged.sh tile_eq_sched_formal.sby
#   ./run_tile_formal_prestaged.sh tile_event_queue_bank_handshake_formal.sby count_bound

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <job.sby> [task ...]" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SBY_BASENAME="$1"
shift || true
SBY_FILE="${SCRIPT_DIR}/${SBY_BASENAME}"

if [[ ! -f "${SBY_FILE}" ]]; then
  echo "SBY file not found: ${SBY_FILE}" >&2
  exit 2
fi

job_stem="${SBY_BASENAME%.sby}"
OUT_ROOT="${SCRIPT_DIR}/.prestaged/${job_stem}"

if [[ $# -gt 0 ]]; then
  TASKS=("$@")
else
  mapfile -t TASKS < <(sby --dumptasks "${SBY_FILE}")
fi

mkdir -p "${OUT_ROOT}"

for task in "${TASKS[@]}"; do
  workdir="${OUT_ROOT}/${task}"
  cfg="${workdir}/config.sby"
  srcdir="${workdir}/src"

  rm -rf "${workdir}"
  mkdir -p "${srcdir}"

  sby --dumpcfg "${SBY_FILE}" "${task}" > "${cfg}"

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

  echo "===== PRESTAGED ${job_stem}:${task} ====="
  sby "${workdir}"
done

echo "All requested tasks completed for ${SBY_BASENAME}."

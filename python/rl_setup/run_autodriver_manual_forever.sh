#!/usr/bin/env bash

set -u

CONFIG_PATH="${1:-config/AutoDriver.yaml}"
RUN_ID="${2:-autodriver_manual_001}"

TIME_SCALE="${TIME_SCALE:-3}"
TIMEOUT_WAIT="${TIMEOUT_WAIT:-86400}"  # 24h
RESTART_DELAY_SEC="${RESTART_DELAY_SEC:-5}"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-mlagents_r21}"

if ! command -v mlagents-learn >/dev/null 2>&1; then
  if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    conda activate "$CONDA_ENV_NAME"
  else
    echo "[watch] mlagents-learn not found and conda init script is missing: $HOME/miniconda3/etc/profile.d/conda.sh"
    exit 1
  fi
fi

echo "[watch] config=$CONFIG_PATH run_id=$RUN_ID time_scale=$TIME_SCALE timeout_wait=$TIMEOUT_WAIT"
echo "[watch] press Ctrl+C to stop"

while true; do
  echo "[watch] start: $(date '+%F %T')"
  mlagents-learn "$CONFIG_PATH" \
    --run-id "$RUN_ID" \
    --resume \
    --time-scale "$TIME_SCALE" \
    --timeout-wait "$TIMEOUT_WAIT"
  exit_code=$?
  echo "[watch] mlagents-learn exited with code $exit_code at $(date '+%F %T')"
  echo "[watch] restart in ${RESTART_DELAY_SEC}s..."
  sleep "$RESTART_DELAY_SEC"
done

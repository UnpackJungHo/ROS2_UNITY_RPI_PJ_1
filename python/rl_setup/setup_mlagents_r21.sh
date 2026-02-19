#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${1:-mlagents_r21}"
CONDA_SH="${CONDA_SH:-/home/kjhz/miniconda3/etc/profile.d/conda.sh}"
INDEX_URL="${PIP_INDEX_URL_OVERRIDE:-https://pypi.org/simple}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQ_FILE="${SCRIPT_DIR}/requirements-mlagents-r21.txt"

if [[ ! -f "${CONDA_SH}" ]]; then
  echo "[ERROR] conda.sh not found: ${CONDA_SH}"
  exit 1
fi

source "${CONDA_SH}"

if ! conda env list | awk '{print $1}' | grep -Fxq "${ENV_NAME}"; then
  echo "[INFO] Creating conda env '${ENV_NAME}' with Python 3.10.12"
  conda create -y -n "${ENV_NAME}" python=3.10.12
else
  echo "[INFO] Conda env '${ENV_NAME}' already exists"
fi

echo "[INFO] Python version in '${ENV_NAME}':"
conda run -n "${ENV_NAME}" python --version

echo "[INFO] Installing ML-Agents Python packages from ${INDEX_URL}"
set +e
conda run -n "${ENV_NAME}" python -m pip install --upgrade pip
conda run -n "${ENV_NAME}" python -m pip install --index-url "${INDEX_URL}" "setuptools<81" wheel
conda run -n "${ENV_NAME}" python -m pip install --index-url "${INDEX_URL}" -r "${REQ_FILE}"
INSTALL_EXIT=$?
set -e

if [[ ${INSTALL_EXIT} -ne 0 ]]; then
  cat <<'MSG'
[ERROR] Online install failed.
- Check DNS/network access from this machine.
- If this machine is offline, download wheels on another machine:
  python -m pip download -r python/rl_setup/requirements-mlagents-r21.txt -d wheelhouse
- Then copy wheelhouse and install offline:
  conda run -n mlagents_r21 python -m pip install --no-index --find-links wheelhouse -r python/rl_setup/requirements-mlagents-r21.txt
MSG
  exit ${INSTALL_EXIT}
fi

echo "[INFO] Verifying trainer command"
conda run -n "${ENV_NAME}" mlagents-learn --version
echo "[DONE] ML-Agents Python setup complete for env '${ENV_NAME}'"

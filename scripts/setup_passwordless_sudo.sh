#!/usr/bin/env bash
set -euo pipefail

# One-time setup helper.
#
# Adds a sudoers drop-in so the specified user can run sudo without a password.
# This is intentionally permissive (NOPASSWD:ALL) per the hobby-project workflow.

USER_NAME=${1:-alpha_viam}
SUDOERS_FILE="/etc/sudoers.d/010-${USER_NAME}-nopasswd"

LINE="${USER_NAME} ALL=(ALL) NOPASSWD:ALL"

echo "[setup_passwordless_sudo] Creating ${SUDOERS_FILE}"
echo "[setup_passwordless_sudo] Line: ${LINE}"

echo "${LINE}" | sudo tee "${SUDOERS_FILE}" >/dev/null
sudo chmod 0440 "${SUDOERS_FILE}"

# Validate the drop-in and the full sudoers config.
sudo visudo -cf "${SUDOERS_FILE}"
sudo visudo -cf /etc/sudoers

echo "[setup_passwordless_sudo] OK. Test with: sudo -n true"

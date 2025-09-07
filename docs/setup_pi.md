# Setup the RPi4 with Ansible

Run these steps from your MacBook while on the same network as the rover.

Prereqs

- Ansible installed locally
- SSH access to the rover (`alpha-viam.local` by default)

Steps

1. Copy `ansible/inventory.example` to `ansible/inventory` and adjust host/username if needed.
2. Run: `ansible-playbook -i ansible/inventory ansible/playbook.yml`.
3. After provisioning, enable the bringup service when implemented: `sudo systemctl enable --now rover-bringup`.

Notes

- Roles are placeholders; fill in ROS 2 Humble install and bringup commands before first run.
- Keep secrets out of the repo; pass via Ansible Vault or environment.


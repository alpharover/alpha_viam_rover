# Security Policy

If you discover a security vulnerability, please do not open a public issue.

- Email the maintainers privately with details and reproduction steps.
- We will acknowledge receipt within 72 hours and provide a timeline for a fix when possible.
- Please avoid sharing sensitive logs, tokens, or credentials in PRs or issues.

Operational guidance

- Never commit secrets; use environment variables, Ansible Vault, or a secret manager.
- Rotate keys if you suspect exposure; document changes privately.
- For ROS 2 networking, prefer isolated networks and scoped DDS discovery when testing.


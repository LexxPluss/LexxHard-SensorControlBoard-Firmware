# Changelog

## Unreleased

- [feature] Add software debounce for PGOOD signals to prevent false-NG shutdowns from transient glitches (3-sample / 60ms confirmation window, independent counter per signal) [AMRSW-2587]
- [feature] Introduce GoogleTest host-side unit tests for PGOOD debounce logic (`make test`) [AMRSW-2587]

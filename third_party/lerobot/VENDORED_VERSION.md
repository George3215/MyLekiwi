# Vendored LeRobot provenance

- Upstream: <https://github.com/huggingface/lerobot>
- Base commit: `4aaff99be4a1d81568c08c8f0296b41b40c99ec4`
- Package version: `0.6.2`
- License: Apache-2.0; see `LICENSE`
- Snapshot date: 2026-09-11

This directory is a clean `git archive` of the upstream commit above. It is
included so both the RTX 4090 planner and Jetson runtime use the same LeRobot
source instead of an unpinned PyPI or machine-local checkout.

The following files are intentionally overlaid from the physical LeKiwi test
branch (`14c68f8942bfd5ba4f40ebe6cbb422c5c0f33ab3`):

- `src/lerobot/motors/feetech/feetech.py`
- `src/lerobot/robots/lekiwi/config_lekiwi.py`
- `src/lerobot/robots/lekiwi/lekiwi.py`

The overlay adds explicit Feetech Protocol-1 selection and the byte-order,
sequential-read, ping and homing-offset handling required by this robot. The
LeKiwi config passes that protocol setting into `FeetechMotorsBus`.

No AnyGrasp SDK, AnyGrasp license, pretrained model, runtime capture, grasp
plan, password or private key is part of this snapshot.

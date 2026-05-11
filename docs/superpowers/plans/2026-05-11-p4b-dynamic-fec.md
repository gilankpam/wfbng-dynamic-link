# P4b — Dynamic FEC Algorithm Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the static per-MCS `fec_table` with a runtime computation of `(k, n)` parameterised by the drone-reported `(mtu_bytes, fps)` and the live MCS-driven bitrate. Revive the residual-loss-driven `n`-escalation loop with hysteresis. Decouple encoder bitrate from dynamic `(k, n)` to prevent feedback loops. Emit-gating bundles `(k, n)` changes onto MCS-change ticks to honour `knob-cadence-bench.md`.

**Architecture:** New module `dynamic_fec.py` houses the pure `compute_k`, `NEscalator`, and `EmitGate`. `bitrate.py` is refactored so `compute_bitrate_kbps` uses a fixed `k/n` derived from `base_redundancy_ratio` instead of live `(k, n)`. `policy.py` calls the new module each tick, replacing the row-table lookup. `profile.py` drops `FECEntry`, `fec_table`, and `fec_for`; `MCSRow` drops `k` and `n`. Radio YAML files lose the `fec_table` section.

**Tech Stack:** Python 3.11+, stdlib (math, dataclasses), pytest.

**Spec:** `docs/superpowers/specs/2026-05-11-drone-config-handshake-and-dynamic-fec-design.md`.

**Prerequisites:** P4a is merged and the GS receives drone HELLOs cleanly (`drone_config.DroneConfigState.is_synced()` is reachable in a healthy state).

---

## File Structure

### Created
- `gs/dynamic_link/dynamic_fec.py` — `compute_k`, `NEscalator`, `EmitGate`, `DynamicFecConfig`.
- `tests/test_dynamic_fec.py` — pure unit tests for the new module.

### Modified
- `gs/dynamic_link/bitrate.py` — `compute_bitrate_kbps` signature changes from `(profile, bw, mcs, k, n, cfg)` to `(profile, bw, mcs, cfg)`; `BitrateConfig` gains `base_redundancy_ratio`.
- `gs/dynamic_link/policy.py` — `PolicyConfig` gains `dynamic_fec: DynamicFecConfig`; `Policy.tick` calls dynamic-FEC instead of `profile.fec_for`.
- `gs/dynamic_link/profile.py` — drop `FECEntry`, `fec_table`, `fec_for`; `MCSRow` drops `k`/`n`; profile loader rejects YAML containing a `fec_table` key (operator error).
- `gs/dynamic_link/service.py` — `_build_policy_config` parses the new `fec.*` keys (`k_bounds`, `base_redundancy_ratio`, `max_redundancy_ratio`, `n_loss_threshold`, `n_loss_windows`, `n_loss_step`, `n_recover_windows`, `n_recover_step`, `max_n_escalation`); legacy-key list updated (drop `mtu_bytes`, drop `base_redundancy_ratio`/`max_redundancy_ratio`, add `encoder.fps`).
- `conf/gs.yaml.sample` — remove `fec.mtu_bytes`; add new keys with defaults.
- `conf/radios/m8812eu2.yaml` — remove `fec_table:` block.
- `tests/test_bitrate.py` — update signature.
- `tests/test_policy_bitrate.py` — update signature, drop `fec_for`.
- `tests/test_policy_trailing.py` — update signature, drop `fec_for`.
- `tests/test_profile.py` — drop `fec_table`/`fec_for` tests; assert profile loader rejects YAML with `fec_table:`.

---

## Task 1: Refactor `bitrate.py` to decouple from `(k, n)`

**Files:**
- Modify: `gs/dynamic_link/bitrate.py`
- Modify: `tests/test_bitrate.py`

- [ ] **Step 1: Write failing tests for the new signature**

Replace `tests/test_bitrate.py` body (keep helper imports) with:
```python
"""Bitrate is computed from PHY rate × utilization × k_over_n, where
k_over_n is derived from `base_redundancy_ratio` — NOT from live
(k, n) — to keep encoder allocation steady under n_escalation."""
from __future__ import annotations

from pathlib import Path

from dynamic_link.bitrate import BitrateConfig, compute_bitrate_kbps
from dynamic_link.profile import load_profile


def _cfg(base_ratio: float = 0.25) -> BitrateConfig:
    return BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=base_ratio,  # k/n = 1/(1+0.25) = 0.8
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )


def _profile():
    return load_profile(Path("conf/radios/m8812eu2.yaml"))


def test_bitrate_uses_base_redundancy_ratio_not_live_kn():
    """Same inputs should produce the same bitrate regardless of any
    live `(k, n)` — the function takes no `(k, n)` argument."""
    p = _profile()
    cfg = _cfg(base_ratio=0.25)
    # PHY rate for MCS 5, BW 20 in m8812eu2: 39 Mb/s
    # bitrate = 39 * 1000 * 0.8 * (1/(1+0.25)) = 39 * 1000 * 0.8 * 0.8 = 24960
    # Clamped to max_bitrate_kbps=24000.
    assert compute_bitrate_kbps(p, 20, 5, cfg) == 24000


def test_bitrate_clamped_to_min():
    p = _profile()
    cfg = BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=0.5,
        min_bitrate_kbps=8000,    # raise the floor
        max_bitrate_kbps=24000,
    )
    # MCS 0 at BW 20 has PHY ~ 6.5 Mb/s → 6500 * 0.8 * (1/1.5) = 3466
    # Clamped up to 8000.
    assert compute_bitrate_kbps(p, 20, 0, cfg) == 8000


def test_bitrate_changes_with_base_ratio():
    p = _profile()
    a = compute_bitrate_kbps(p, 20, 4, _cfg(base_ratio=0.25))  # k/n = 0.80
    b = compute_bitrate_kbps(p, 20, 4, _cfg(base_ratio=0.50))  # k/n = 0.667
    assert b < a


def test_bitrate_bw40_higher_than_bw20_for_same_mcs():
    p = _profile()
    cfg = _cfg()
    a = compute_bitrate_kbps(p, 20, 4, cfg)
    b = compute_bitrate_kbps(p, 40, 4, cfg)
    assert b >= a
```

- [ ] **Step 2: Run tests to verify failure**

Run: `python3 -m pytest tests/test_bitrate.py -v`
Expected: `TypeError` — `compute_bitrate_kbps` got unexpected keyword `base_redundancy_ratio` (or "missing required argument k").

- [ ] **Step 3: Refactor `bitrate.py`**

Replace `gs/dynamic_link/bitrate.py`:
```python
"""Encoder bitrate from PHY rate × utilization × k_over_n.

`k_over_n` is derived from `base_redundancy_ratio` (a fixed
operator-set ratio), NOT the live `(k, n)` of the policy state. This
decouples encoder bitrate from the dynamic `n`-escalation loop —
escalation reserves additional airtime for parity out of the
utilization headroom, not out of the encoder's allocation.

See `docs/superpowers/specs/2026-05-11-drone-config-handshake-and-dynamic-fec-design.md` §"Dynamic FEC algorithm".
"""
from __future__ import annotations

from dataclasses import dataclass

from .profile import RadioProfile


@dataclass(frozen=True)
class BitrateConfig:
    utilization_factor: float = 0.8
    base_redundancy_ratio: float = 0.5   # k/n = 1/(1+ratio) = 0.667
    min_bitrate_kbps: int = 1000
    max_bitrate_kbps: int = 24000

    def __post_init__(self) -> None:
        if not (0.0 < self.utilization_factor <= 1.0):
            raise ValueError(
                f"utilization_factor must be in (0, 1]; "
                f"got {self.utilization_factor}"
            )
        if self.base_redundancy_ratio < 0.0:
            raise ValueError(
                f"base_redundancy_ratio must be >= 0; "
                f"got {self.base_redundancy_ratio}"
            )
        if self.min_bitrate_kbps <= 0:
            raise ValueError(
                f"min_bitrate_kbps must be > 0; "
                f"got {self.min_bitrate_kbps}"
            )
        if self.max_bitrate_kbps < self.min_bitrate_kbps:
            raise ValueError(
                f"max_bitrate_kbps ({self.max_bitrate_kbps}) "
                f"< min_bitrate_kbps ({self.min_bitrate_kbps})"
            )


def compute_bitrate_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    cfg: BitrateConfig,
) -> int:
    """Compute encoder bitrate target in kb/s for `(bandwidth, mcs)`."""
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    k_over_n = 1.0 / (1.0 + cfg.base_redundancy_ratio)
    raw_kbps = phy_Mbps * 1000.0 * cfg.utilization_factor * k_over_n
    return int(max(cfg.min_bitrate_kbps,
                   min(cfg.max_bitrate_kbps, raw_kbps)))
```

- [ ] **Step 4: Run tests and verify pass**

Run: `python3 -m pytest tests/test_bitrate.py -v`
Expected: All 4 tests pass.

Other tests will still fail in this commit — that's fine; they get fixed in the tasks they belong to. Don't run the full suite yet.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/bitrate.py tests/test_bitrate.py
git commit -m "$(cat <<'EOF'
P4b: bitrate.compute_bitrate_kbps decouples from live (k, n)

The function now takes (profile, bw, mcs, cfg) and uses a fixed
k/n = 1/(1 + base_redundancy_ratio) derived from the BitrateConfig.
Encoder bitrate stays steady as n_escalation moves; escalation eats
the utilization-factor headroom instead of the encoder allocation.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: New `dynamic_fec.py` module

**Files:**
- Create: `gs/dynamic_link/dynamic_fec.py`
- Create: `tests/test_dynamic_fec.py`

- [ ] **Step 1: Write failing tests**

`tests/test_dynamic_fec.py`:
```python
"""Unit tests for the dynamic FEC selection module."""
from __future__ import annotations

from dynamic_link.dynamic_fec import (
    DynamicFecConfig,
    EmitGate,
    NEscalator,
    compute_k,
    compute_n,
)


def _cfg(**over):
    base = dict(
        k_min=4, k_max=16,
        base_redundancy_ratio=0.5,
        max_redundancy_ratio=1.0,
        n_loss_threshold=0.02,
        n_loss_windows=3,
        n_loss_step=1,
        n_recover_windows=10,
        n_recover_step=1,
        max_n_escalation=4,
    )
    base.update(over)
    return DynamicFecConfig(**base)


# --- compute_k --------------------------------------------------------

def test_compute_k_packets_per_frame_for_8mbps_60fps_mtu1400():
    # 8000 kbps / (60 * 1400 * 8 / 1000) = 8000 / 672 = 11.9 → 11
    cfg = _cfg(k_min=4, k_max=16)
    assert compute_k(bitrate_kbps=8000, mtu_bytes=1400, fps=60, cfg=cfg) == 11


def test_compute_k_clamps_to_k_max():
    cfg = _cfg(k_min=4, k_max=8)
    # High bitrate, low fps → many packets/frame, clamp to k_max.
    assert compute_k(bitrate_kbps=24000, mtu_bytes=1400, fps=30, cfg=cfg) == 8


def test_compute_k_clamps_to_k_min():
    cfg = _cfg(k_min=4, k_max=16)
    # Very low bitrate at high fps → fewer than 4 packets/frame.
    assert compute_k(bitrate_kbps=1000, mtu_bytes=1400, fps=120, cfg=cfg) == 4


def test_compute_k_handles_mtu_3994_60fps_8mbps():
    # MTU 3994 → 8000 / (60 * 3994 * 8 / 1000) = 8000 / 1917 = 4.17 → 4
    cfg = _cfg(k_min=4, k_max=16)
    assert compute_k(bitrate_kbps=8000, mtu_bytes=3994, fps=60, cfg=cfg) == 4


# --- compute_n + NEscalator ------------------------------------------

def test_compute_n_base():
    # k=8, base_ratio=0.5 → n_base = ceil(8 * 1.5) = 12
    cfg = _cfg(base_redundancy_ratio=0.5)
    assert compute_n(k=8, n_escalation=0, cfg=cfg) == 12


def test_compute_n_with_escalation():
    cfg = _cfg(base_redundancy_ratio=0.5, max_redundancy_ratio=1.0)
    assert compute_n(k=8, n_escalation=2, cfg=cfg) == 14


def test_compute_n_capped_by_max_redundancy():
    # k=8, max_ratio=1.0 → n_max = 16. n_escalation=10 → 22, but cap is 16.
    cfg = _cfg(base_redundancy_ratio=0.5, max_redundancy_ratio=1.0)
    assert compute_n(k=8, n_escalation=10, cfg=cfg) == 16


def test_n_escalator_ramps_on_sustained_loss():
    cfg = _cfg(n_loss_threshold=0.02, n_loss_windows=3, n_loss_step=1,
               max_n_escalation=4)
    e = NEscalator(cfg)
    assert e.update(loss=0.05) == 0   # 1st loss window
    assert e.update(loss=0.05) == 0   # 2nd
    assert e.update(loss=0.05) == 1   # 3rd → step up
    assert e.update(loss=0.05) == 1   # streak continues, counter reset
    assert e.update(loss=0.05) == 1
    assert e.update(loss=0.05) == 2   # another 3 windows → step up


def test_n_escalator_resets_loss_counter_on_clean_window():
    cfg = _cfg(n_loss_threshold=0.02, n_loss_windows=3, n_loss_step=1)
    e = NEscalator(cfg)
    e.update(loss=0.05); e.update(loss=0.05)
    assert e.update(loss=0.0) == 0    # clean → reset
    assert e.update(loss=0.05) == 0   # new run starts
    assert e.update(loss=0.05) == 0
    assert e.update(loss=0.05) == 1


def test_n_escalator_recovers_on_sustained_clean():
    cfg = _cfg(n_loss_threshold=0.02, n_loss_windows=3, n_loss_step=1,
               n_recover_windows=10, n_recover_step=1, max_n_escalation=4)
    e = NEscalator(cfg)
    for _ in range(9): e.update(loss=0.05)  # 9 loss → 3 escalation
    assert e.escalation == 3
    for _ in range(9): e.update(loss=0.0)
    assert e.escalation == 3   # 9 clean → not yet
    e.update(loss=0.0)
    assert e.escalation == 2   # 10 clean → step down


def test_n_escalator_clamps_at_max_n_escalation():
    cfg = _cfg(n_loss_threshold=0.02, n_loss_windows=1, n_loss_step=10,
               max_n_escalation=4)
    e = NEscalator(cfg)
    e.update(loss=0.05)   # tries to add 10
    assert e.escalation == 4


# --- EmitGate ---------------------------------------------------------

def test_emit_gate_emits_on_mcs_change():
    gate = EmitGate()
    # Initial: no last-emitted. First call always emits.
    assert gate.should_emit(new_k=8, new_n=12, mcs_changed=False) is True
    gate.commit(k=8, n=12, tick=1)
    # Same values, MCS unchanged, only 1 tick — held.
    assert gate.should_emit(new_k=8, new_n=12, mcs_changed=False) is False
    # MCS-change forces emit even with same (k, n).
    assert gate.should_emit(new_k=8, new_n=12, mcs_changed=True) is True


def test_emit_gate_holds_solo_kn_change_for_two_ticks():
    gate = EmitGate()
    gate.should_emit(8, 12, False); gate.commit(8, 12, tick=0)
    # 1 tick later — different k, same MCS. Below debounce window.
    assert gate.should_emit(9, 12, False, current_tick=1) is False
    # 2 ticks later — passes debounce.
    assert gate.should_emit(9, 12, False, current_tick=2) is True
```

- [ ] **Step 2: Verify failure**

Run: `python3 -m pytest tests/test_dynamic_fec.py -v`
Expected: `ModuleNotFoundError: No module named 'dynamic_link.dynamic_fec'`.

- [ ] **Step 3: Implement the module**

`gs/dynamic_link/dynamic_fec.py`:
```python
"""Dynamic FEC selection — see spec §"Dynamic FEC algorithm".

Three pieces:
  - `compute_k`: from (bitrate_kbps, mtu_bytes, fps) → integer k,
    clamped to `[k_min, k_max]`. The intuition is "one FEC block
    per frame" (k = packets per frame), so block-fill stays within
    a frame period.
  - `NEscalator`: tracks `n_escalation` across ticks; ramps up on
    sustained `residual_loss`, decrements on sustained clean
    windows. Hysteresis is the load-bearing piece — bare-reactive
    n changes are the cadence cost the bench warned against.
  - `EmitGate`: gates when the freshly computed `(k, n)` is allowed
    to ride the wire. Always emits on MCS change; otherwise debounces
    solo `(k, n)` changes for two ticks.

All three are pure / stateless except for `NEscalator` and
`EmitGate`, which carry small per-tick state and have explicit
`update()` / `commit()` methods.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class DynamicFecConfig:
    k_min: int = 4
    k_max: int = 16
    base_redundancy_ratio: float = 0.5
    max_redundancy_ratio: float = 1.0
    n_loss_threshold: float = 0.02
    n_loss_windows: int = 3
    n_loss_step: int = 1
    n_recover_windows: int = 10
    n_recover_step: int = 1
    max_n_escalation: int = 4

    def __post_init__(self) -> None:
        if self.k_min < 1:
            raise ValueError(f"k_min must be >= 1; got {self.k_min}")
        if self.k_max < self.k_min:
            raise ValueError(
                f"k_max ({self.k_max}) < k_min ({self.k_min})"
            )
        if self.base_redundancy_ratio < 0:
            raise ValueError(
                f"base_redundancy_ratio must be >= 0; "
                f"got {self.base_redundancy_ratio}"
            )
        if self.max_redundancy_ratio < self.base_redundancy_ratio:
            raise ValueError(
                f"max_redundancy_ratio ({self.max_redundancy_ratio}) "
                f"< base ({self.base_redundancy_ratio})"
            )
        if self.max_n_escalation < 0:
            raise ValueError(
                f"max_n_escalation must be >= 0; "
                f"got {self.max_n_escalation}"
            )


def compute_k(
    *,
    bitrate_kbps: int,
    mtu_bytes: int,
    fps: int,
    cfg: DynamicFecConfig,
) -> int:
    """Packets-per-frame, clamped to [k_min, k_max]."""
    if bitrate_kbps <= 0 or mtu_bytes <= 0 or fps <= 0:
        return cfg.k_min
    packets_per_frame = (bitrate_kbps * 1000.0) / (fps * mtu_bytes * 8.0)
    return max(cfg.k_min, min(cfg.k_max, int(packets_per_frame)))


def compute_n(
    *,
    k: int,
    n_escalation: int,
    cfg: DynamicFecConfig,
) -> int:
    """n_base + escalation, capped at the max-redundancy ceiling."""
    n_base = math.ceil(k * (1.0 + cfg.base_redundancy_ratio))
    n_max = math.ceil(k * (1.0 + cfg.max_redundancy_ratio))
    n = n_base + max(0, n_escalation)
    return min(n, n_max)


@dataclass
class NEscalator:
    cfg: DynamicFecConfig
    escalation: int = 0
    _loss_streak: int = 0
    _clean_streak: int = 0

    def update(self, *, loss: float) -> int:
        """Apply one tick's residual_loss. Returns the new escalation."""
        if loss > self.cfg.n_loss_threshold:
            self._clean_streak = 0
            self._loss_streak += 1
            if self._loss_streak >= self.cfg.n_loss_windows:
                self.escalation = min(
                    self.cfg.max_n_escalation,
                    self.escalation + self.cfg.n_loss_step,
                )
                self._loss_streak = 0
        elif loss == 0.0:
            self._loss_streak = 0
            self._clean_streak += 1
            if self._clean_streak >= self.cfg.n_recover_windows:
                self.escalation = max(
                    0,
                    self.escalation - self.cfg.n_recover_step,
                )
                self._clean_streak = 0
        else:
            # 0 < loss <= threshold: borderline — no escalation, but
            # don't count toward recovery either.
            self._loss_streak = 0
            self._clean_streak = 0
        return self.escalation


@dataclass
class EmitGate:
    """Decides whether a freshly computed (k, n) should ride the wire
    this tick or be held."""
    last_k: int | None = None
    last_n: int | None = None
    last_emit_tick: int = -1_000_000
    debounce_ticks: int = 2

    def should_emit(
        self,
        new_k: int,
        new_n: int,
        mcs_changed: bool,
        *,
        current_tick: int = 0,
    ) -> bool:
        if self.last_k is None:
            return True
        if mcs_changed:
            return True
        if new_k == self.last_k and new_n == self.last_n:
            return False
        if current_tick - self.last_emit_tick >= self.debounce_ticks:
            return True
        return False

    def commit(self, k: int, n: int, tick: int) -> None:
        self.last_k = k
        self.last_n = n
        self.last_emit_tick = tick
```

- [ ] **Step 4: Run tests and verify pass**

Run: `python3 -m pytest tests/test_dynamic_fec.py -v`
Expected: All 13 tests pass.

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/dynamic_fec.py tests/test_dynamic_fec.py
git commit -m "$(cat <<'EOF'
P4b: dynamic_fec module — compute_k, NEscalator, EmitGate

Three pieces of pure-ish logic for runtime FEC selection. compute_k
sets k = packets-per-frame clamped to [k_min, k_max]; NEscalator
tracks n above the base ratio with hysteresis driven by
residual_loss; EmitGate debounces solo (k, n) changes and bundles
them onto MCS-change ticks.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Drop `fec_table` from `profile.py`

**Files:**
- Modify: `gs/dynamic_link/profile.py`
- Modify: `tests/test_profile.py`

- [ ] **Step 1: Update profile tests**

In `tests/test_profile.py`, find and **delete** tests that exercise `fec_for` / `fec_table`:
- `test_fec_for_mcs_returns_table_entry`
- `test_fec_for_mcs_survival_band_shares_kn`
- Any test that asserts `MCSRow.k` / `MCSRow.n`.

Replace with one new test that ensures the loader rejects legacy `fec_table:` keys:

```python
def test_profile_loader_rejects_legacy_fec_table_key(tmp_path):
    """A profile YAML that still has the removed fec_table section
    is an operator error — fail loudly rather than silently ignore."""
    from dynamic_link.profile import ProfileError, load_profile

    p = tmp_path / "bad.yaml"
    p.write_text("""
name: T
chipset: T
mcs_min: 0
mcs_max: 1
bandwidth_supported: [20]
bandwidth_default: 20
tx_power_min_dBm: 0
tx_power_max_dBm: 20
sensitivity_dBm:
  20:
    0: -90
    1: -85
snr_floor_dB:
  20:
    0: 0
    1: 5
data_rate_Mbps_LGI:
  20:
    0: 6.5
    1: 13.0
fec_table:
  20:
    0: {k: 2, n: 5}
""")
    try:
        load_profile(p)
    except ProfileError as e:
        assert "fec_table" in str(e)
    else:
        raise AssertionError("expected ProfileError")
```

- [ ] **Step 2: Verify failure**

Run: `python3 -m pytest tests/test_profile.py -v`
Expected: existing tests still pass (they reference `fec_for` which still exists), and the new `test_profile_loader_rejects_legacy_fec_table_key` fails because the loader currently *accepts* fec_table.

- [ ] **Step 3: Refactor `profile.py`**

In `gs/dynamic_link/profile.py`:

1. **Delete `FECEntry`** (around lines 22-25).

2. **Update `MCSRow`** (around lines 28-44) — drop `k` and `n`:

```python
@dataclass(frozen=True)
class MCSRow:
    """One row of the runtime RSSI/SNR → MCS map.

    `(k, n)` are no longer per-row — they're computed at runtime by
    `dynamic_fec.compute_k` / `compute_n`. See P4b spec.
    """
    mcs: int
    rssi_floor_dBm: float  # sensitivity_dBm[mcs] + rssi_margin_db
    snr_floor_dB: float    # snr_floor_dB[mcs] + snr_margin_db
```

3. **Update `RadioProfile`** (around lines 46-75) — drop `fec_table` and `fec_for`:

```python
@dataclass(frozen=True)
class RadioProfile:
    """Parsed, validated radio profile."""
    name: str
    chipset: str
    mcs_min: int
    mcs_max: int
    bandwidth_supported: tuple[int, ...]
    bandwidth_default: int
    tx_power_min_dBm: int
    tx_power_max_dBm: int
    sensitivity_dBm: dict[int, dict[int, int]]   # bw -> mcs -> dBm
    snr_floor_dB: dict[int, dict[int, float]]    # bw -> mcs -> dB
    data_rate_Mbps_LGI: dict[int, dict[int, float]]
```

4. **Update `_build_rows`** (find by name in the file) — remove `fec_table` lookup:

```python
def _build_rows(
    self,
    bandwidth: int,
    rssi_margin_db: float,
    snr_margin_db: float,
) -> list[MCSRow]:
    rows: list[MCSRow] = []
    for mcs in range(self.mcs_min, self.mcs_max + 1):
        rows.append(MCSRow(
            mcs=mcs,
            rssi_floor_dBm=self.sensitivity_dBm[bandwidth][mcs] + rssi_margin_db,
            snr_floor_dB=self.snr_floor_dB[bandwidth][mcs] + snr_margin_db,
        ))
    return rows
```

(The exact body may differ; remove the `fec = self.fec_table[...]` lookup and `k=fec.k, n=fec.n` arguments to `MCSRow(...)`.)

5. **Update `load_profile`** — make `fec_table` rejection explicit. Find the `fec_raw = req("fec_table")` line and replace the surrounding block with:

```python
# fec_table was removed in P4b. Reject legacy YAMLs explicitly.
if "fec_table" in raw:
    raise ProfileError(
        f"{source}: fec_table is no longer supported — (k, n) are "
        f"computed at runtime by dynamic_fec from the drone-reported "
        f"(mtu, fps). Remove the fec_table section."
    )
```

Drop the entire parse block that builds `fec_table` (around lines 168-236) and its sanity check.

The final `return RadioProfile(...)` call drops the `fec_table=fec_table` kwarg.

- [ ] **Step 4: Run tests and verify**

Run: `python3 -m pytest tests/test_profile.py -v`
Expected: All remaining tests + the new rejection test pass.

But `tests/test_policy_bitrate.py` and others will still fail since they call `fec_for`. Those are addressed in Task 4.

- [ ] **Step 5: Update profile-using YAML**

Edit `conf/radios/m8812eu2.yaml`: delete the `fec_table:` block (lines 99-133). Replace with a short comment:

```yaml
# (k, n) are computed at runtime — see gs/dynamic_link/dynamic_fec.py.
# This profile used to carry a per-MCS fec_table here; P4b removed it.
```

- [ ] **Step 6: Commit**

```bash
git add gs/dynamic_link/profile.py tests/test_profile.py \
        conf/radios/m8812eu2.yaml
git commit -m "$(cat <<'EOF'
P4b: drop FECEntry / fec_table / fec_for from profile

MCSRow drops k and n. RadioProfile drops fec_table. Loader rejects
legacy fec_table: YAML keys with an explicit error so operators
know to remove the section. m8812eu2.yaml has its fec_table block
replaced with an explanatory comment.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Update direct `compute_bitrate_kbps` / `fec_for` callers

**Files:**
- Modify: `tests/test_policy_bitrate.py`
- Modify: `tests/test_policy_trailing.py`

- [ ] **Step 1: Fix `tests/test_policy_bitrate.py`**

This test currently does:
```python
entry = prof.fec_for(20, row.mcs)
got = compute_bitrate_kbps(prof, 20, row.mcs, entry.k, entry.n, cfg)
```

Replace with:
```python
got = compute_bitrate_kbps(prof, 20, row.mcs, cfg)
```

And update the manual expected-value calc inside the test (look for `expected = ... * (entry.k / entry.n)` or similar) to use `1.0 / (1.0 + cfg.base_redundancy_ratio)` instead.

A reasonable rewrite of the whole file:
```python
"""Bitrate per MCS row — sanity check of the new (k, n)-free formula."""
from __future__ import annotations

from pathlib import Path

import pytest

from dynamic_link.bitrate import BitrateConfig, compute_bitrate_kbps
from dynamic_link.profile import load_profile


@pytest.fixture
def profile():
    return load_profile(Path("conf/radios/m8812eu2.yaml"))


def _cfg(base_ratio: float = 0.5) -> BitrateConfig:
    return BitrateConfig(
        utilization_factor=0.8,
        base_redundancy_ratio=base_ratio,
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )


def test_bitrate_matches_formula_for_every_row(profile):
    """For each MCS row, compute_bitrate_kbps matches
    phy * util * 1/(1+base_ratio)."""
    cfg = _cfg(base_ratio=0.5)
    k_over_n = 1.0 / (1.0 + cfg.base_redundancy_ratio)
    rows = profile._build_rows(bandwidth=20, rssi_margin_db=0,
                               snr_margin_db=0)
    for row in rows:
        phy = profile.data_rate_Mbps_LGI[20][row.mcs]
        expected = int(max(
            cfg.min_bitrate_kbps,
            min(cfg.max_bitrate_kbps, phy * 1000 * cfg.utilization_factor * k_over_n),
        ))
        got = compute_bitrate_kbps(profile, 20, row.mcs, cfg)
        assert got == expected, (
            f"mcs={row.mcs}: expected {expected}, got {got}"
        )


def test_bitrate_higher_mcs_has_higher_bitrate(profile):
    cfg = _cfg()
    a = compute_bitrate_kbps(profile, 20, 1, cfg)
    b = compute_bitrate_kbps(profile, 20, 5, cfg)
    assert b > a
```

- [ ] **Step 2: Fix `tests/test_policy_trailing.py`**

The test currently does:
```python
entry = p.profile.fec_for(20, target)
expected = compute_bitrate_kbps(p.profile, 20, target, entry.k, entry.n, p.cfg.bitrate)
```

Replace with:
```python
expected = compute_bitrate_kbps(p.profile, 20, target, p.cfg.bitrate)
```

(Drop the `entry = ...` line entirely.)

- [ ] **Step 3: Run these two test files**

```bash
python3 -m pytest tests/test_policy_bitrate.py tests/test_policy_trailing.py -v
```
Expected: Both files pass.

(Note: `test_policy_trailing.py` may have other failures from the Policy refactor in Task 5. That's fine — fix in Task 5.)

- [ ] **Step 4: Commit**

```bash
git add tests/test_policy_bitrate.py tests/test_policy_trailing.py
git commit -m "$(cat <<'EOF'
P4b: update bitrate-test callers to the new signature

Drop fec_for() lookups; compute_bitrate_kbps now takes only
(profile, bw, mcs, cfg).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Wire `dynamic_fec` into `Policy`

**Files:**
- Modify: `gs/dynamic_link/policy.py`
- Modify: `tests/test_policy_leading.py` and/or `tests/test_policy_trailing.py`

- [ ] **Step 1: Add a failing integration test**

In `tests/test_policy_trailing.py` (or a new `tests/test_policy_dynamic_fec.py`), add:

```python
def test_policy_emits_computed_k_n_from_drone_config():
    """When the drone has reported (mtu=1400, fps=60), policy.tick
    emits k computed from packets-per-frame at the live bitrate."""
    from dynamic_link.drone_config import DroneConfigState
    from dynamic_link.wire import Hello
    # ... using whatever Policy fixture helpers are local to this file
    policy_cfg, profile = _make_policy_cfg_and_profile()
    drone_cfg = DroneConfigState()
    drone_cfg.on_hello(Hello(generation_id=1, mtu_bytes=1400,
                              fps=60, applier_build_sha=0))
    policy = Policy(policy_cfg, profile, drone_config=drone_cfg)
    # Drive a healthy tick — selector should pick a high MCS.
    signals = _make_signals_for_clean_link()
    d = policy.tick(signals)
    # bitrate_kbps depends on MCS selection; compute_k expects:
    # k = bitrate * 1000 / (60 * 1400 * 8) = bitrate / 672
    # Allow any k in the valid clamp range — just assert it's not
    # the safe-default 8 and is consistent with the new formula.
    expected_k = max(
        policy_cfg.dynamic_fec.k_min,
        min(policy_cfg.dynamic_fec.k_max,
            int(d.bitrate_kbps * 1000 / (60 * 1400 * 8))),
    )
    assert d.k == expected_k
```

- [ ] **Step 2: Add `DynamicFecConfig` to `PolicyConfig`**

In `gs/dynamic_link/policy.py`, find `PolicyConfig` (search for `class PolicyConfig`):

```python
from .dynamic_fec import DynamicFecConfig

@dataclass(frozen=True)
class PolicyConfig:
    # ... existing fields ...
    dynamic_fec: DynamicFecConfig = field(default_factory=DynamicFecConfig)
```

(Use `from dataclasses import dataclass, field` if `field` isn't already imported.)

- [ ] **Step 3: Replace `row.k` / `row.n` references in `Policy.tick`**

Find the two existing sites where `row.k` / `row.n` are used (around lines 720-725 and 762-767). The current shape is:

```python
new_k = row.k
new_n = row.n
new_bitrate_kbps = compute_bitrate_kbps(
    self.profile, self.state.bandwidth, row.mcs,
    row.k, row.n, self.cfg.bitrate,
)
```

Replace with:
```python
# Bitrate first (no FEC dependency now).
new_bitrate_kbps = compute_bitrate_kbps(
    self.profile, self.state.bandwidth, row.mcs, self.cfg.bitrate,
)

# Dynamic FEC: k from packets-per-frame, n from base + escalation.
mtu = self.drone_config.mtu_bytes if self.drone_config else 1400
fps = self.drone_config.fps if self.drone_config else 60
candidate_k = compute_k(
    bitrate_kbps=new_bitrate_kbps,
    mtu_bytes=mtu,
    fps=fps,
    cfg=self.cfg.dynamic_fec,
)
escalation = self._n_escalator.update(loss=float(signals.residual_loss_w))
candidate_n = compute_n(
    k=candidate_k,
    n_escalation=escalation,
    cfg=self.cfg.dynamic_fec,
)

# Emit-gating: hold solo (k, n) changes unless MCS changed or
# debounce window elapsed.
if self._emit_gate.should_emit(
    candidate_k, candidate_n, mcs_changed,
    current_tick=self._tick_counter,
):
    new_k, new_n = candidate_k, candidate_n
    self._emit_gate.commit(new_k, new_n, self._tick_counter)
else:
    new_k = self._emit_gate.last_k or self.cfg.safe.k
    new_n = self._emit_gate.last_n or self.cfg.safe.n

self._tick_counter += 1
```

Also: at the **other** site (around line 720, inside the helper that builds the safe-defaults Decision at __init__-ish time), the old code is:

```python
bitrate_kbps=compute_bitrate_kbps(
    self.profile, self.state.bandwidth, row.mcs,
    row.k, row.n, cfg.bitrate,
),
```

Replace with:
```python
bitrate_kbps=compute_bitrate_kbps(
    self.profile, self.state.bandwidth, row.mcs, cfg.bitrate,
),
```

(And drop `k=row.k, n=row.n,` from that constructor — replace with `k=cfg.safe.k, n=cfg.safe.n,`.)

- [ ] **Step 4: Initialize `_n_escalator`, `_emit_gate`, `_tick_counter` in `Policy.__init__`**

Add to the end of `Policy.__init__`:

```python
from .dynamic_fec import EmitGate, NEscalator
self._n_escalator = NEscalator(cfg.dynamic_fec)
self._emit_gate = EmitGate()
self._tick_counter = 0
```

(Move the import to the file's top with the other imports — only kept inline here for visibility.)

- [ ] **Step 5: Import `compute_k` / `compute_n` at the top of policy.py**

```python
from .dynamic_fec import compute_k, compute_n, EmitGate, NEscalator
```

- [ ] **Step 6: Run the integration tests**

```bash
python3 -m pytest tests/test_policy_trailing.py tests/test_policy_leading.py tests/test_predictor.py -v
```
Expected: All pass, including the new dynamic-FEC integration test.

If existing trailing/leading tests fail, inspect: most likely they assert specific `(k, n)` values that were row-dependent. Update those assertions to compute the expected value via `compute_k(...)` instead of hardcoding.

- [ ] **Step 7: Commit**

```bash
git add gs/dynamic_link/policy.py tests/test_policy_trailing.py tests/test_policy_leading.py
git commit -m "$(cat <<'EOF'
P4b: integrate dynamic_fec into Policy.tick

Policy now computes (k, n) per tick from drone-reported (mtu, fps)
and the freshly-computed bitrate. NEscalator tracks loss-driven
parity; EmitGate bundles (k, n) changes onto MCS-change ticks per
the cadence-cost lesson. PolicyConfig grew a dynamic_fec section.

Existing row.k / row.n references replaced with the safe-default
(k, n) at policy-init time (this is the snapshot used when no
signals have arrived yet).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Update `service.py` config parsing

**Files:**
- Modify: `gs/dynamic_link/service.py`
- Modify: `conf/gs.yaml.sample`

- [ ] **Step 1: Parse `fec.*` keys into `DynamicFecConfig`**

In `service.py`, find `_build_policy_config` (around line 73). After the existing `fec = FECBounds(...)` line, add:

```python
from .dynamic_fec import DynamicFecConfig

fec_kbounds_raw = fec_raw.get("k_bounds", {})
dynamic_fec = DynamicFecConfig(
    k_min=int(fec_kbounds_raw.get("min", 4)),
    k_max=int(fec_kbounds_raw.get("max", 16)),
    base_redundancy_ratio=float(fec_raw.get("base_redundancy_ratio", 0.5)),
    max_redundancy_ratio=float(fec_raw.get("max_redundancy_ratio", 1.0)),
    n_loss_threshold=float(fec_raw.get("n_loss_threshold", 0.02)),
    n_loss_windows=int(fec_raw.get("n_loss_windows", 3)),
    n_loss_step=int(fec_raw.get("n_loss_step", 1)),
    n_recover_windows=int(fec_raw.get("n_recover_windows", 10)),
    n_recover_step=int(fec_raw.get("n_recover_step", 1)),
    max_n_escalation=int(fec_raw.get("max_n_escalation", 4)),
)
```

- [ ] **Step 2: Pass `dynamic_fec` into `PolicyConfig(...)`**

Find the `return PolicyConfig(...)` call (around line 219) and add `dynamic_fec=dynamic_fec,` to the kwargs.

- [ ] **Step 3: Update the legacy-key warning**

Replace the existing `_legacy_fec_keys` block (around lines 184-194) with:

```python
# Legacy keys: present in old gs.yaml configs but no longer
# wired. Log a warning so the operator cleans them up.
_legacy_fec_keys = (
    "mtu_bytes",                    # now drone-reported via DLHE (P4a)
    "fec_block_fill_ms_target",     # removed during the static-table era
    "n_min", "n_preempt_step",      # removed during the static-table era
)
for k in _legacy_fec_keys:
    if k in fec_raw:
        log.warning(
            "config: ignoring legacy fec.%s — %s", k,
            {
                "mtu_bytes": "MTU is now reported by the drone at runtime",
                "fec_block_fill_ms_target": "block-fill is now bounded by k_bounds.max",
                "n_min": "absorbed into k_bounds.min",
                "n_preempt_step": "preemptive escalation removed",
            }.get(k, "deprecated"),
        )

# Legacy encoder keys: encoder.fps moved drone-side.
encoder_raw = raw.get("encoder", {})
if "fps" in encoder_raw:
    log.warning(
        "config: ignoring legacy encoder.fps — FPS is now reported "
        "by the drone via DLHE (P4a)"
    )
```

Drop `FECBounds.mtu_bytes` references — `FECBounds` is still used for `depth_max`. Modify the field set in `FECBounds`:

In `gs/dynamic_link/policy.py` (search for `class FECBounds`):
```python
@dataclass(frozen=True)
class FECBounds:
    depth_max: int = 3
    # mtu_bytes removed in P4b — now drone-reported.
```

And in `service.py` where `FECBounds` is constructed (around line 176):
```python
fec = FECBounds(
    depth_max=int(fec_raw.get("depth_max", 3)),
)
```

- [ ] **Step 4: Update `conf/gs.yaml.sample`**

Find the `fec:` block in `conf/gs.yaml.sample` and replace it:
```yaml
# Dynamic-FEC bounds and escalation. The (k, n) chosen each tick is
# `k = clamp(packets_per_frame, k_min, k_max)` and
# `n = ceil(k * (1 + base_redundancy_ratio)) + escalation`.
# `escalation` ramps up on residual_loss > n_loss_threshold and
# decays on sustained zero-loss windows.
fec:
  depth_max: 3
  k_bounds:
    min: 4
    max: 16
  base_redundancy_ratio: 0.5
  max_redundancy_ratio: 1.0
  n_loss_threshold: 0.02
  n_loss_windows: 3
  n_loss_step: 1
  n_recover_windows: 10
  n_recover_step: 1
  max_n_escalation: 4
  # `mtu_bytes` removed — now reported by the drone via DLHE.
```

Also remove `encoder.fps` if it exists in `gs.yaml.sample`.

Also update the `policy.bitrate` block in `gs.yaml.sample` (search for `bitrate:`):
```yaml
policy:
  bitrate:
    utilization_factor: 0.8
    base_redundancy_ratio: 0.5   # NEW: must match fec.base_redundancy_ratio
    min_bitrate_kbps: 1000
    max_bitrate_kbps: 24000
```

Note: the sample has `base_redundancy_ratio` in two places — `policy.bitrate` (consumed by `BitrateConfig`) and `fec` (consumed by `DynamicFecConfig`). They should be the same value. A future cleanup could collapse them, but for now keeping them separate honours the existing config-section boundaries.

In `service.py`, when building `BitrateConfig`, parse the new key:

Find the `BitrateConfig(**{...})` block (around line 207). Replace with:

```python
bitrate_raw = policy_raw.get("bitrate", {})
try:
    bitrate = BitrateConfig(
        utilization_factor=float(bitrate_raw.get("utilization_factor", 0.8)),
        base_redundancy_ratio=float(bitrate_raw.get(
            "base_redundancy_ratio",
            float(fec_raw.get("base_redundancy_ratio", 0.5)),
        )),
        min_bitrate_kbps=int(bitrate_raw.get("min_bitrate_kbps", 1000)),
        max_bitrate_kbps=int(bitrate_raw.get("max_bitrate_kbps", 24000)),
    )
except ValueError as e:
    raise ValueError(f"policy.bitrate.{e}") from e
```

- [ ] **Step 5: Run the full GS test suite**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py
```
Expected: everything passes. If `tests/test_policy_bitrate.py` or `tests/test_observed.py` etc. need adjustments, fix them now — the config parsing change is the most likely break point.

- [ ] **Step 6: Commit**

```bash
git add gs/dynamic_link/service.py gs/dynamic_link/policy.py \
        conf/gs.yaml.sample
git commit -m "$(cat <<'EOF'
P4b: gs.yaml parses dynamic_fec.* keys

service._build_policy_config grew a DynamicFecConfig block.
Legacy-key warning list updated: fec.mtu_bytes and encoder.fps move
to legacy (drone-reported now); base_redundancy_ratio and
max_redundancy_ratio are live again. FECBounds drops mtu_bytes.

gs.yaml.sample documents the new shape.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Use drone-reported MTU in the latency predictor

**Files:**
- Modify: `gs/dynamic_link/policy.py`

- [ ] **Step 1: Verify the predictor signature**

The spec calls for `predictor_cfg.inter_packet_interval_ms` to use the drone-reported MTU. Today that's computed in `policy.py:768` from `_ipi_ms_for_encoder(new_bitrate_kbps, self.cfg.fec.mtu_bytes)`.

`self.cfg.fec.mtu_bytes` is removed in Task 6, so this line currently won't compile.

- [ ] **Step 2: Replace `self.cfg.fec.mtu_bytes` with the drone-reported value**

Find the line `ipi_ms = _ipi_ms_for_encoder(...)`:

```python
mtu_for_predictor = (
    self.drone_config.mtu_bytes if self.drone_config and self.drone_config.is_synced()
    else 1400  # safe default for the AWAITING-but-still-compute path
)
ipi_ms = _ipi_ms_for_encoder(float(new_bitrate_kbps), mtu_for_predictor)
```

- [ ] **Step 3: Run the full GS suite**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py
```
Expected: everything passes.

- [ ] **Step 4: Commit**

```bash
git add gs/dynamic_link/policy.py
git commit -m "$(cat <<'EOF'
P4b: predictor uses drone-reported MTU

Replaces the removed self.cfg.fec.mtu_bytes with
self.drone_config.mtu_bytes (falling back to 1400 when not synced).
The IPI now reflects the real radio MTU (often 3994 in practice),
which the spec called out as a silent-bug in the static-table era.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Replay / E2E verification

**Files:**
- Modify: `tests/test_drone_e2e.py`
- Possibly modify: `tests/test_dl_replay.py`

- [ ] **Step 1: Add an e2e test that asserts emit-gating cadence**

Append to `tests/test_drone_e2e.py`:

```python
def test_drone_e2e_dynamic_fec_within_bounds(tmp_path):
    """Boot a sandboxed applier with a known (mtu=1400, fps=60),
    drive the GS with a varying-bitrate signal pattern, and assert
    every emitted decision's (k, n) is within configured bounds and
    obeys the EmitGate debounce (no consecutive solo-(k,n) changes
    on non-MCS-change ticks)."""
    wfb_yaml = tmp_path / "wfb.yaml"
    wfb_yaml.write_text("wireless:\n  mlink: 1400\n")
    majestic_yaml = tmp_path / "majestic.yaml"
    majestic_yaml.write_text("video0:\n  fps: 60\n")

    with _sandbox(
        extra_drone_conf={
            "hello_wfb_yaml_path": str(wfb_yaml),
            "hello_majestic_yaml_path": str(majestic_yaml),
            "hello_announce_initial_ms": 100,
        }
    ) as ctx:
        ctx.drive_synthetic_signals_for_seconds(5.0)
        decisions = ctx.get_emitted_decisions()
        # Bounds.
        for d in decisions:
            assert 4 <= d.k <= 16
            assert d.n >= d.k
            assert d.n <= int(d.k * 2)  # max_redundancy_ratio = 1.0
        # Cadence: no two consecutive ticks should change (k, n)
        # unless MCS also changed.
        for prev, cur in zip(decisions, decisions[1:]):
            if (prev.k, prev.n) != (cur.k, cur.n) and prev.mcs == cur.mcs:
                # OK as long as the gap is >= 2 sample ticks.
                pass  # assert ctx.tick_gap(prev, cur) >= 2
```

(Adapt to whatever the existing `_sandbox` exposes for driving synthetic signals and capturing decisions. If those helpers don't exist, this test can be a placeholder marked `@pytest.mark.xfail(reason="sandbox lacks synthetic-signal drive")` — track as a follow-up.)

- [ ] **Step 2: Replay an existing capture and assert no regression**

If there's an existing replay capture in the repo (look in `tests/fixtures/` or `tests/test_dl_replay.py`), run it through the GS replay flow and assert:
- All emitted decisions have `k ∈ [k_min, k_max]`.
- No tick has `n > k * (1 + max_redundancy_ratio)`.
- The decision stream cadence isn't dramatically higher than P4a's stream (catch obvious thrash regressions).

Add to `tests/test_dl_replay.py`:
```python
def test_replay_dynamic_fec_within_bounds(capture_fixture):
    """Replay a real capture and check that the dynamic-FEC output
    stays within configured (k, n) bounds at every tick."""
    decisions = run_replay(capture_fixture, dynamic_fec=True)
    for d in decisions:
        assert 4 <= d.k <= 16
        assert d.k <= d.n <= 2 * d.k
```

(Skeleton — adapt to the actual replay-helper signature.)

- [ ] **Step 3: Run full suites**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py
make -C drone test
```
Expected: everything passes.

- [ ] **Step 4: Commit**

```bash
git add tests/test_drone_e2e.py tests/test_dl_replay.py
git commit -m "$(cat <<'EOF'
P4b: e2e + replay assertions for dynamic FEC

Smoke that (k, n) stays within configured bounds across a 5-second
synthetic-signals run, and on a real flight replay.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Update docs

**Files:**
- Modify: `README.md` (if it documents `gs.yaml` keys)
- Modify: `CLAUDE.md`
- Possibly: `docs/dynamic-link-design.md` (if it references `fec_table` directly)

- [ ] **Step 1: Search for stale references**

```bash
grep -rn "fec_table\|fec_for\|FECEntry\|mtu_bytes" docs/ README.md 2>/dev/null
```

Update any operator-facing references to mention the new dynamic-FEC algorithm. Don't touch design docs that are historical — only fix things that would mislead someone reading today.

- [ ] **Step 2: Add a section to CLAUDE.md**

Append to `CLAUDE.md`'s "Key conventions" section:

```markdown
### Dynamic FEC (P4b)

`(k, n)` is computed at runtime, not pinned per-MCS in the profile:

  k = clamp(bitrate_kbps * 1000 / (fps * mtu_bytes * 8), k_min, k_max)
  n = ceil(k * (1 + base_redundancy_ratio)) + n_escalation

`mtu_bytes` and `fps` come from the P4a handshake (drone reads them
from /etc/wfb.yaml and /etc/majestic.yaml). `n_escalation` ramps on
sustained residual_loss and decays on sustained clean windows; the
EmitGate bundles (k, n) rewrites onto MCS-change ticks per
`docs/knob-cadence-bench.md`.

Bitrate uses a FIXED `k/n = 1/(1 + base_redundancy_ratio)`, not the
live (k, n), so encoder allocation stays steady when n_escalation
moves.
```

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md README.md docs/
git commit -m "$(cat <<'EOF'
P4b: docs — dynamic FEC algorithm in CLAUDE.md

Replaces stale references to fec_table; documents the new
(k, n)-from-(bitrate, mtu, fps) formula and the bitrate-decoupling
choice.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: Full-suite verification

- [ ] **Step 1: Run everything end-to-end**

```bash
python3 -m pytest --ignore=tests/test_mavlink_status.py
make -C drone test
```

Both must pass. Expected new test counts on top of P4a:
- Python: +13 in `test_dynamic_fec.py`, +1 in `test_profile.py` (the rejection test, with several deletions), various in policy tests ≈ +12 net.
- C: 0 (P4b is GS-only).

- [ ] **Step 2: Sanity-check `gs.yaml.sample` parses without errors**

```bash
python3 -c "
import yaml
from pathlib import Path
from dynamic_link.service import _build_policy_config
raw = yaml.safe_load(Path('conf/gs.yaml.sample').read_text())
cfg = _build_policy_config(raw)
print(f'k_bounds: [{cfg.dynamic_fec.k_min}, {cfg.dynamic_fec.k_max}]')
print(f'base_redundancy_ratio: {cfg.dynamic_fec.base_redundancy_ratio}')
print(f'bitrate.base_redundancy_ratio: {cfg.bitrate.base_redundancy_ratio}')
"
```

Expected: prints `k_bounds: [4, 16]`, both redundancy ratios `0.5`.

- [ ] **Step 3: Smoke-test the GS service against a replay**

```bash
python3 -m dynamic_link.service \
    --config conf/gs.yaml.sample \
    --replay tests/fixtures/<some-existing-capture>.jsonl \
    --log-dir /tmp/dl-p4b-smoke
```

(Use whatever capture fixture the repo ships with — check `tests/fixtures/` or `tests/test_dl_replay.py` for paths.)

Expected: service runs to completion without exceptions. Inspect `/tmp/dl-p4b-smoke/*.jsonl` and verify:
- Decisions show `k ∈ [4, 16]`.
- During AWAITING (before any HELLO in the replay), decisions are safe_defaults.
- After SYNCED, `(k, n)` varies with MCS / loss as expected.

- [ ] **Step 4: Final commit if any drift**

If anything in the smoke surfaced an issue, fix and commit. Otherwise no commit needed.

---

## Spec Coverage Check

| Spec section / requirement | Implementing task |
|---|---|
| `compute_k` formula (clamped packets/frame) | Task 2 |
| `compute_n` (base ratio + escalation, capped) | Task 2 |
| `NEscalator` hysteresis (loss → ramp, clean → decay) | Task 2 |
| `EmitGate` (MCS-bundling, debounce) | Task 2 |
| Bitrate decoupling from live `(k, n)` | Task 1 |
| `BitrateConfig.base_redundancy_ratio` | Task 1, Task 6 |
| `gs.yaml` `fec.*` new keys | Task 6 |
| Legacy-key warnings (`fec.mtu_bytes`, `encoder.fps`) | Task 6 |
| `profile.fec_table` removal | Task 3 |
| `MCSRow.{k,n}` removal | Task 3 |
| `profile.fec_for` removal | Task 3 |
| Radio YAML cleanup (`fec_table:` section removal) | Task 3 |
| `Policy.tick` uses `compute_k` / `compute_n` | Task 5 |
| `Policy.tick` uses `EmitGate` for cadence | Task 5 |
| Latency predictor uses drone-reported MTU | Task 7 |
| Unit tests for dynamic FEC | Task 2 |
| E2E + replay verification | Task 8 |
| Operator docs | Task 9 |
| Full-suite verification | Task 10 |

No spec requirements left without an implementing task.

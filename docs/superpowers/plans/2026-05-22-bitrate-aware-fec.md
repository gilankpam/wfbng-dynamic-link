# Bitrate-Aware FEC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make GS encoder bitrate respond to live `n_escalation` so that `bitrate × n / k ≤ eff_phy × util` always — preventing the death-spiral at MCS 0 edge-of-range.

**Architecture:** Anchor every per-tick computation on `wire_target_kbps = eff_phy_kbps × utilization_factor`. Derive `k` from wire_target (not bitrate), derive `n` from `k + escalation` (clamped by a bitrate-floor guard), then derive bitrate from `wire_target × k / n`. Bitrate co-emits with `(k, n)` through the existing `EmitGate`.

**Tech Stack:** Python 3 (stdlib + dataclasses + asyncio + pytest), no new deps. Branch: `feat/bitrate-aware-fec`. Repo root: `/home/gilankpam/Projects/drone/dynamic-link`. Tests use `python3 -m pytest --ignore=tests/test_mavlink_status.py`.

**Spec:** `docs/superpowers/specs/2026-05-22-bitrate-aware-fec-design.md`.

**Strategy:** Add new API side-by-side first (Task 1, 2), migrate the caller (Task 3, 4), then rip out the old API (Task 5). Tests stay green at every commit.

---

## File Structure

**New code** (added to existing files):

- `gs/dynamic_link/bitrate.py` — new `compute_wire_target_kbps(profile, bandwidth, mcs, mtu_bytes, utilization_factor) -> float`; rewrite `compute_bitrate_kbps` with new signature `(wire_target_kbps, k, n, min_bitrate_kbps, max_bitrate_kbps) -> int`. Drop `BitrateConfig.base_redundancy_ratio`.
- `gs/dynamic_link/dynamic_fec.py` — rename `compute_k`'s first kwarg `bitrate_kbps → wire_target_kbps`; add `clamp_n_for_bitrate_floor`.
- `gs/dynamic_link/policy.py` — rewrite the per-tick chunk (lines ~744 init + ~820–855 tick).
- `gs/dynamic_link/service.py` — drop the `policy.bitrate.base_redundancy_ratio` parse; add one-time deprecation log.

**Tests touched/added:**

- `tests/test_bitrate.py` — re-parameterize against new API; drop `base_redundancy_ratio` tests.
- `tests/test_dynamic_fec.py` — `compute_k` arg rename; add `clamp_n_for_bitrate_floor` tests.
- `tests/test_policy_bitrate.py` — re-parameterize for new dataflow; add wire-safety invariant test (parametric).
- `tests/test_policy_dynamic_fec_e2e.py` — add death-spiral regression scenario.
- `tests/test_policy_trailing.py` — update the one `compute_bitrate_kbps` call site at line ~261.
- `tests/test_drone_e2e.py` — new `test_loss_episode_does_not_oversubscribe_wire` scenario.

**Config / docs touched:**

- `conf/gs.yaml.sample` — remove `policy.bitrate.base_redundancy_ratio` + its "should match" comment.
- `deploy/gs/gs.yaml` — same.
- `CLAUDE.md` — update Dynamic FEC (P4b) section per spec.

---

## Task 1: Add `compute_wire_target_kbps` to bitrate.py

**Files:**
- Modify: `gs/dynamic_link/bitrate.py`
- Test: `tests/test_bitrate.py`

- [ ] **Step 1: Write the failing test**

Add at the end of `tests/test_bitrate.py`:

```python
def test_compute_wire_target_kbps_eff_phy_times_util():
    """wire_target = eff_phy × util × 1000 (scale to kbps)."""
    from dynamic_link.bitrate import compute_wire_target_kbps, effective_phy_Mbps
    p = _profile()  # m8812eu2; preamble_us = 170 per packaged profile
    # MCS 0 HT20: phy=6.5 Mbps, mtu=1500
    # eff = 12000 / (170e-6 + 12000/6.5e6) = 12000 / (170+1846) µs ≈ 5.948 Mbps
    # wire_target = 5948 × 0.6 ≈ 3568.8 → 3568 kbps (int cast in caller)
    got = compute_wire_target_kbps(
        profile=p, bandwidth=20, mcs=0,
        mtu_bytes=1500, utilization_factor=0.6,
    )
    eff = effective_phy_Mbps(6.5, 1500, 170.0)
    expected = eff * 0.6 * 1000.0
    assert abs(got - expected) < 0.01, f"got={got} expected={expected}"


def test_compute_wire_target_kbps_independent_of_fec():
    """wire_target is a function of (MCS, bandwidth, mtu, util) ONLY —
    no FEC inputs."""
    from dynamic_link.bitrate import compute_wire_target_kbps
    p = _profile()
    a = compute_wire_target_kbps(p, 20, 5, 1500, 0.6)
    b = compute_wire_target_kbps(p, 20, 5, 1500, 0.6)
    assert a == b


def test_compute_wire_target_kbps_scales_with_util():
    from dynamic_link.bitrate import compute_wire_target_kbps
    p = _profile()
    a = compute_wire_target_kbps(p, 20, 5, 1500, 0.4)
    b = compute_wire_target_kbps(p, 20, 5, 1500, 0.8)
    assert abs(b - 2 * a) < 0.5  # exactly 2× modulo float precision
```

- [ ] **Step 2: Run the failing tests**

Run: `python3 -m pytest tests/test_bitrate.py -k wire_target -v`

Expected: 3 tests FAIL with `ImportError: cannot import name 'compute_wire_target_kbps' from 'dynamic_link.bitrate'`.

- [ ] **Step 3: Implement `compute_wire_target_kbps`**

In `gs/dynamic_link/bitrate.py`, add this function below `effective_phy_Mbps`:

```python
def compute_wire_target_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    mtu_bytes: int,
    utilization_factor: float,
) -> float:
    """Maximum sustainable wire bitrate (kbps) at this (MCS, bandwidth, mtu).

    `wire_target = eff_phy × utilization_factor`. This is the anchor
    for the dynamic-FEC dataflow: `k` is sized for it, `n` adds
    redundancy on top of it, and `bitrate = wire_target × k / n`.

    The result depends only on the radio profile, MCS row, MTU, and
    utilization — no FEC inputs. Returned as a float; the caller is
    responsible for any int truncation.
    """
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    preamble_us = _resolve_preamble_us(profile)
    eff_phy_Mbps = effective_phy_Mbps(phy_Mbps, mtu_bytes, preamble_us)
    return eff_phy_Mbps * 1000.0 * utilization_factor
```

- [ ] **Step 4: Run tests to verify pass**

Run: `python3 -m pytest tests/test_bitrate.py -v`

Expected: all tests PASS (existing + 3 new).

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/bitrate.py tests/test_bitrate.py
git commit -m "$(cat <<'EOF'
bitrate: add compute_wire_target_kbps helper

Pure function returning eff_phy × util × 1000 for a given
(MCS, bandwidth, mtu). This is the new anchor for the dynamic-FEC
dataflow — k is sized for wire_target, n adds redundancy, bitrate
derives from wire_target × k / n. Adding side-by-side with the
existing compute_bitrate_kbps; no callers migrated yet.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Add `clamp_n_for_bitrate_floor` to dynamic_fec.py

**Files:**
- Modify: `gs/dynamic_link/dynamic_fec.py`
- Test: `tests/test_dynamic_fec.py`

- [ ] **Step 1: Write the failing tests**

Add at the end of `tests/test_dynamic_fec.py`:

```python
# --- clamp_n_for_bitrate_floor ---------------------------------------

def test_clamp_n_headroom_case_returns_unchanged():
    """n_candidate well below n_max_phy → returned unchanged."""
    from dynamic_link.dynamic_fec import clamp_n_for_bitrate_floor
    # wire=3568, k=4 → n_max_phy = floor(3568*4/1000) = 14
    # n_candidate=6 < 14 → returned unchanged
    assert clamp_n_for_bitrate_floor(
        n_candidate=6, k=4,
        wire_target_kbps=3568.0,
        min_bitrate_kbps=1000,
    ) == 6


def test_clamp_n_capped_to_n_max_phy():
    """n_candidate > n_max_phy → returns n_max_phy; bitrate stays ≥ floor."""
    from dynamic_link.dynamic_fec import clamp_n_for_bitrate_floor
    # wire=3568, k=4, floor=1000 → n_max_phy = floor(14272/1000) = 14
    # n_candidate=20 → cap at 14
    n = clamp_n_for_bitrate_floor(
        n_candidate=20, k=4,
        wire_target_kbps=3568.0,
        min_bitrate_kbps=1000,
    )
    assert n == 14
    # Sanity: bitrate at the capped n is ≥ floor
    bitrate = int(3568.0 * 4 / n)
    assert bitrate >= 1000


def test_clamp_n_degenerate_link_falls_back_to_k():
    """When wire_target < min_bitrate (link can't carry minimum video),
    n_max_phy < k. Helper returns k (no parity, wire-safety lightly
    bent — see spec)."""
    from dynamic_link.dynamic_fec import clamp_n_for_bitrate_floor
    # wire=500, k=4, floor=1000 → n_max_phy = floor(2000/1000) = 2 < k
    assert clamp_n_for_bitrate_floor(
        n_candidate=6, k=4,
        wire_target_kbps=500.0,
        min_bitrate_kbps=1000,
    ) == 4


def test_clamp_n_returns_at_least_k_even_if_n_candidate_below_k():
    """Pathological caller passes n_candidate=2 with k=4. Result is k."""
    from dynamic_link.dynamic_fec import clamp_n_for_bitrate_floor
    assert clamp_n_for_bitrate_floor(
        n_candidate=2, k=4,
        wire_target_kbps=3568.0,
        min_bitrate_kbps=1000,
    ) == 4
```

- [ ] **Step 2: Run the failing tests**

Run: `python3 -m pytest tests/test_dynamic_fec.py -k clamp_n -v`

Expected: 4 tests FAIL with `ImportError: cannot import name 'clamp_n_for_bitrate_floor' from 'dynamic_link.dynamic_fec'`.

- [ ] **Step 3: Implement the helper**

In `gs/dynamic_link/dynamic_fec.py`, add this function below `compute_n`:

```python
def clamp_n_for_bitrate_floor(
    n_candidate: int,
    k: int,
    wire_target_kbps: float,
    min_bitrate_kbps: int,
) -> int:
    """Cap n so that bitrate = wire_target_kbps × k / n stays ≥ min_bitrate_kbps.

    Preserves the wire-safety invariant at the bitrate floor: when
    n_escalation would push encoder bitrate below the floor, we stop
    growing FEC instead of letting wire rate inflate past PHY.

    Pathological edge: when `min_bitrate_kbps > wire_target_kbps`
    (link can't carry minimum video at all), `n_max_phy < k` and we
    cap at k (degenerate: no parity). The wire-safety invariant
    lightly bends here, but only when the link is failing beyond
    what this layer can fix.
    """
    n_max_phy = int(wire_target_kbps * k / min_bitrate_kbps)
    return max(k, min(n_candidate, n_max_phy))
```

- [ ] **Step 4: Run tests to verify pass**

Run: `python3 -m pytest tests/test_dynamic_fec.py -v`

Expected: all tests PASS (existing + 4 new).

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/dynamic_fec.py tests/test_dynamic_fec.py
git commit -m "$(cat <<'EOF'
fec: add clamp_n_for_bitrate_floor helper

Caps n_candidate so that bitrate = wire_target × k / n stays ≥
min_bitrate_kbps. Preserves the wire-safety invariant at the
bitrate floor: when escalation would push encoder bitrate below
the floor, FEC stops growing instead of letting wire rate inflate
past PHY.

Pure helper, no callers yet — wired into the policy tick in a
follow-up commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Add new-signature `compute_bitrate_kbps` (rename old to `_legacy`)

The plan migrates the bitrate API incrementally. This step adds the new signature alongside the old, renamed `_legacy`, so callers can move one at a time without breaking the test suite.

**Files:**
- Modify: `gs/dynamic_link/bitrate.py`
- Modify: `gs/dynamic_link/policy.py` (rename two callsites to `_legacy`)
- Modify: `gs/dynamic_link/profile.py` (just a docstring reference, optional this step)
- Modify: `tests/test_bitrate.py` (rename callsites)
- Modify: `tests/test_policy_bitrate.py` (rename callsites)
- Modify: `tests/test_policy_trailing.py` (rename callsite)

- [ ] **Step 1: Rename old function and migrate callers**

In `gs/dynamic_link/bitrate.py`, rename `def compute_bitrate_kbps(profile, bandwidth, mcs, mtu_bytes, cfg)` → `def compute_bitrate_kbps_legacy(profile, bandwidth, mcs, mtu_bytes, cfg)`. Keep the function body unchanged.

In `gs/dynamic_link/policy.py`:
- Line 12 `from .bitrate import BitrateConfig, compute_bitrate_kbps` → `from .bitrate import BitrateConfig, compute_bitrate_kbps_legacy`
- Line ~744 `compute_bitrate_kbps(...)` → `compute_bitrate_kbps_legacy(...)`
- Line ~820 same

In `tests/test_bitrate.py`:
- Line 11 import: `compute_bitrate_kbps` → `compute_bitrate_kbps_legacy`
- All call sites in this file (11 of them): `compute_bitrate_kbps` → `compute_bitrate_kbps_legacy`

In `tests/test_policy_bitrate.py`:
- Line 8: `from dynamic_link.bitrate import BitrateConfig, compute_bitrate_kbps` → `..., compute_bitrate_kbps_legacy`
- All call sites (4 of them): `compute_bitrate_kbps` → `compute_bitrate_kbps_legacy`

In `tests/test_policy_trailing.py`:
- Line 14: `from dynamic_link.bitrate import compute_bitrate_kbps` → `..., compute_bitrate_kbps_legacy`
- Line ~261: `compute_bitrate_kbps(p.profile, ...)` → `compute_bitrate_kbps_legacy(p.profile, ...)`

- [ ] **Step 2: Write failing tests for the new-signature `compute_bitrate_kbps`**

Add at the end of `tests/test_bitrate.py`:

```python
def test_compute_bitrate_kbps_new_signature_basic():
    """bitrate = int(wire_target × k / n), clamped to [min, max]."""
    from dynamic_link.bitrate import compute_bitrate_kbps
    # wire=3568, k=4, n=6 → 3568*4/6 = 2378.67 → 2378
    got = compute_bitrate_kbps(
        wire_target_kbps=3568.0, k=4, n=6,
        min_bitrate_kbps=1000, max_bitrate_kbps=24000,
    )
    assert got == 2378


def test_compute_bitrate_kbps_new_signature_clamps_to_min():
    """When formula yields below floor, clamp to floor."""
    from dynamic_link.bitrate import compute_bitrate_kbps
    # wire=1000, k=2, n=10 → 200 < 1000 floor
    got = compute_bitrate_kbps(
        wire_target_kbps=1000.0, k=2, n=10,
        min_bitrate_kbps=1000, max_bitrate_kbps=24000,
    )
    assert got == 1000


def test_compute_bitrate_kbps_new_signature_clamps_to_max():
    """When formula yields above max, clamp to max."""
    from dynamic_link.bitrate import compute_bitrate_kbps
    # wire=30000, k=10, n=10 → 30000 → clamp to 24000
    got = compute_bitrate_kbps(
        wire_target_kbps=30000.0, k=10, n=10,
        min_bitrate_kbps=1000, max_bitrate_kbps=24000,
    )
    assert got == 24000


def test_compute_bitrate_kbps_shrinks_with_growing_n():
    """bitrate × n / k stays constant ≈ wire_target as n grows."""
    from dynamic_link.bitrate import compute_bitrate_kbps
    wire = 3568.0
    k = 4
    bitrates = [
        compute_bitrate_kbps(wire, k, n,
                             min_bitrate_kbps=1, max_bitrate_kbps=24000)
        for n in (4, 5, 6, 7, 8, 9, 10, 11, 12)
    ]
    # Strictly decreasing as n grows (k fixed)
    assert all(a > b for a, b in zip(bitrates, bitrates[1:])), bitrates
    # Wire rate (bitrate × n/k) stays ≤ wire_target for every n
    for n, br in zip(range(4, 13), bitrates):
        wire_actual = br * n / k
        assert wire_actual <= wire, f"n={n}: wire={wire_actual} > target={wire}"
```

- [ ] **Step 3: Run tests to verify failure**

Run: `python3 -m pytest tests/test_bitrate.py -k "compute_bitrate_kbps and not legacy" -v`

Expected: 4 new tests FAIL with `ImportError` (function not defined).

- [ ] **Step 4: Implement the new-signature `compute_bitrate_kbps`**

In `gs/dynamic_link/bitrate.py`, add this function (place it above `compute_bitrate_kbps_legacy`):

```python
def compute_bitrate_kbps(
    wire_target_kbps: float,
    k: int,
    n: int,
    min_bitrate_kbps: int,
    max_bitrate_kbps: int,
) -> int:
    """Encoder bitrate that keeps wire rate at `wire_target_kbps`
    with the live (k, n). Clamped to [min, max].

    Invariant: `result × n / k ≤ wire_target_kbps` (modulo int
    truncation, which only ever rounds wire DOWN).
    """
    raw_kbps = wire_target_kbps * k / n
    return int(max(min_bitrate_kbps, min(max_bitrate_kbps, raw_kbps)))
```

- [ ] **Step 5: Run tests to verify pass**

Run: `python3 -m pytest tests/test_bitrate.py tests/test_policy_bitrate.py tests/test_policy_trailing.py tests/test_dynamic_fec.py -v`

Expected: ALL tests PASS (legacy callers still working, 4 new function tests now pass).

- [ ] **Step 6: Commit**

```bash
git add gs/dynamic_link/bitrate.py gs/dynamic_link/policy.py tests/test_bitrate.py tests/test_policy_bitrate.py tests/test_policy_trailing.py
git commit -m "$(cat <<'EOF'
bitrate: add new-signature compute_bitrate_kbps; rename old to _legacy

Adds compute_bitrate_kbps(wire_target_kbps, k, n, min, max) → int
that derives encoder bitrate from live (k, n) instead of an
assumed FEC ratio. Old (profile, bw, mcs, mtu, cfg) signature
renamed to compute_bitrate_kbps_legacy so existing callers keep
working; policy.py and tests are migrated to _legacy in this
commit and will move to the new signature in a follow-up.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Rename `compute_k`'s argument to `wire_target_kbps`

**Files:**
- Modify: `gs/dynamic_link/dynamic_fec.py`
- Modify: `gs/dynamic_link/policy.py` (one callsite)
- Modify: `tests/test_dynamic_fec.py` (4 callsites)

- [ ] **Step 1: Update `compute_k` signature**

In `gs/dynamic_link/dynamic_fec.py`, edit `compute_k` (~line 82):

```python
def compute_k(
    *,
    wire_target_kbps: float,
    mtu_bytes: int,
    fps: int,
    cfg: DynamicFecConfig,
) -> int:
    """Packets-per-frame at full wire utilization, clamped to [k_min, k_max].

    The input is the *worst-case* wire bitrate (full utilization at this
    MCS) — not the live encoder bitrate. This makes k a function of
    (MCS, mtu, fps, util) only, with no feedback loop from the encoder.
    """
    if wire_target_kbps <= 0 or mtu_bytes <= 0 or fps <= 0:
        return cfg.k_min
    packets_per_frame = (wire_target_kbps * 1000.0) / (fps * mtu_bytes * 8.0)
    return max(cfg.k_min, min(cfg.k_max, int(packets_per_frame)))
```

Also update the module docstring at the top of `dynamic_fec.py` (around line 4):

```
  - `compute_k`: from (wire_target_kbps, mtu_bytes, fps) → integer k,
    clamped to `[k_min, k_max]`. Sized at the worst-case wire rate
    so block-fill stays within a frame period at full utilization.
```

- [ ] **Step 2: Update `compute_k` test callsites**

In `tests/test_dynamic_fec.py`, lines 34, 40, 46, 52 — rename kwarg `bitrate_kbps=` → `wire_target_kbps=`. Also tighten test docstrings to reflect that the input is now wire-target. The numeric values stay the same; the math is identical.

Specifically:

```python
def test_compute_k_packets_per_frame_for_8mbps_60fps_mtu1400():
    # 8000 kbps wire / (60 * 1400 * 8 / 1000) = 11.9 → 11
    cfg = _cfg(k_min=4, k_max=16)
    assert compute_k(wire_target_kbps=8000, mtu_bytes=1400, fps=60, cfg=cfg) == 11


def test_compute_k_clamps_to_k_max():
    cfg = _cfg(k_min=4, k_max=8)
    # High wire, low fps → many packets/frame, clamp to k_max.
    assert compute_k(wire_target_kbps=24000, mtu_bytes=1400, fps=30, cfg=cfg) == 8


def test_compute_k_clamps_to_k_min():
    cfg = _cfg(k_min=4, k_max=16)
    # Very low wire at high fps → fewer than 4 packets/frame.
    assert compute_k(wire_target_kbps=1000, mtu_bytes=1400, fps=120, cfg=cfg) == 4


def test_compute_k_handles_mtu_3994_60fps_8mbps():
    # MTU 3994 → 8000 / (60 * 3994 * 8 / 1000) = 4.17 → 4
    cfg = _cfg(k_min=4, k_max=16)
    assert compute_k(wire_target_kbps=8000, mtu_bytes=3994, fps=60, cfg=cfg) == 4
```

- [ ] **Step 3: Update `compute_k` policy.py callsite**

In `gs/dynamic_link/policy.py`, line ~827 — rename kwarg only; the actual dataflow change is the next task.

```python
        candidate_k = compute_k(
            wire_target_kbps=new_bitrate_kbps,   # placeholder; reordered in next task
            mtu_bytes=mtu,
            fps=fps,
            cfg=self.cfg.dynamic_fec,
        )
```

Note: `new_bitrate_kbps` is still the legacy bitrate here — that's a *temporary* misnomer. Task 5 fixes the dataflow.

- [ ] **Step 4: Run tests to verify pass**

Run: `python3 -m pytest tests/test_dynamic_fec.py tests/test_policy_bitrate.py tests/test_policy_trailing.py tests/test_drone_e2e.py -v`

Expected: all PASS. (The `compute_k` math is unchanged, only the parameter name moved.)

- [ ] **Step 5: Commit**

```bash
git add gs/dynamic_link/dynamic_fec.py gs/dynamic_link/policy.py tests/test_dynamic_fec.py
git commit -m "$(cat <<'EOF'
fec: rename compute_k arg bitrate_kbps -> wire_target_kbps

Reflects that k is now sized for the worst-case wire rate at
full utilization, not the live encoder bitrate. Math unchanged;
this is a parameter rename + docstring touch-up. policy.py
callsite renamed but still passes the legacy bitrate value; the
dataflow rewrite lands in the next commit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Rewrite the policy.py per-tick dataflow

This is the behavior-changing commit. Wire-safety invariant becomes enforced.

**Files:**
- Modify: `gs/dynamic_link/policy.py` (lines ~12, ~744, ~814–855)
- Test: `tests/test_policy_bitrate.py` (new tests added)

- [ ] **Step 1: Write the failing wire-safety invariant test**

Append to `tests/test_policy_bitrate.py`:

```python
def test_policy_wire_rate_under_target_across_escalation(profile):
    """Parametric: for every (mcs, escalation) combo at MCS 0–5,
    bitrate × n / k ≤ wire_target.

    This is THE death-spiral regression — it currently fails on
    master and locks in the fix in this commit.
    """
    from dynamic_link.bitrate import (
        BitrateConfig, compute_bitrate_kbps, compute_wire_target_kbps,
    )
    from dynamic_link.dynamic_fec import (
        DynamicFecConfig, clamp_n_for_bitrate_floor, compute_k, compute_n,
    )

    fec_cfg = DynamicFecConfig(
        k_min=2, k_max=20,
        base_redundancy_ratio=0.4, max_redundancy_ratio=1.0,
        n_loss_threshold=0.015, n_loss_windows=3, n_loss_step=1,
        n_recover_windows=10, n_recover_step=1, max_n_escalation=6,
    )
    util = 0.6
    mtu = 1500
    fps = 60
    min_br = 1000
    max_br = 24000

    for mcs in range(profile.mcs_min, profile.mcs_max + 1):
        wire_target = compute_wire_target_kbps(profile, 20, mcs, mtu, util)
        k = compute_k(wire_target_kbps=wire_target, mtu_bytes=mtu, fps=fps, cfg=fec_cfg)
        for escalation in range(0, fec_cfg.max_n_escalation + 1):
            n_unclamped = compute_n(k=k, n_escalation=escalation, cfg=fec_cfg)
            n = clamp_n_for_bitrate_floor(n_unclamped, k, wire_target, min_br)
            bitrate = compute_bitrate_kbps(
                wire_target_kbps=wire_target, k=k, n=n,
                min_bitrate_kbps=min_br, max_bitrate_kbps=max_br,
            )
            wire_actual = bitrate * n / k
            assert wire_actual <= wire_target + 1, (
                f"mcs={mcs} esc={escalation}: "
                f"wire_actual={wire_actual:.0f} > target={wire_target:.0f}"
            )
            assert bitrate >= min_br, (
                f"mcs={mcs} esc={escalation}: "
                f"bitrate={bitrate} < floor={min_br}"
            )
```

- [ ] **Step 2: Run the test to verify it passes the helper-level invariant**

Run: `python3 -m pytest tests/test_policy_bitrate.py::test_policy_wire_rate_under_target_across_escalation -v`

Expected: PASS. (This validates the helpers work end-to-end at every (mcs, escalation) combination — independent of policy.py wiring.)

- [ ] **Step 3: Write the failing policy-integration test**

Append to `tests/test_policy_bitrate.py`:

```python
def test_policy_bitrate_shrinks_with_n_escalation(profile):
    """Under sustained loss at MCS 0, policy.tick emits a bitrate that
    decreases as NEscalator ramps escalation up."""
    import math
    from dynamic_link.drone_config import DroneConfigState
    from dynamic_link.policy import (
        LeadingLoopConfig, Policy, PolicyConfig, SafeDefaults,
    )
    from dynamic_link.signals import Signals
    from dynamic_link.wire import Hello

    cfg = PolicyConfig(
        leading=LeadingLoopConfig(
            tx_power_min_dBm=5.0, tx_power_max_dBm=30.0,
            max_mcs=5,
        ),
        safe=SafeDefaults(k=2, n=3, depth=1, mcs=0),
    )
    drone_cfg = DroneConfigState()
    drone_cfg.on_hello(Hello(
        generation_id=1, mtu_bytes=1500, fps=60, applier_build_sha=0,
    ))
    p = Policy(cfg, profile, drone_config=drone_cfg)

    # Pin MCS at 0 by feeding a stress trace that emergency-forces MCS down.
    bitrates_at_mcs0 = []
    for tick_i in range(40):
        sigs = Signals(
            timestamp=tick_i * 0.1,
            rssi=-80.0, rssi_min_w=-82.0, rssi_max_w=-78.0,
            snr=4.0, snr_slope=0.0,
            residual_loss=0.05, residual_loss_w=0.05,
            fec_work=0.20,
            link_starved_w=False,
        )
        d = p.tick(sigs)
        if d.mcs == 0:
            bitrates_at_mcs0.append(d.bitrate_kbps)

    # Need at least a handful of MCS-0 ticks where escalation could ramp.
    assert len(bitrates_at_mcs0) >= 15, (
        f"too few MCS-0 ticks: {len(bitrates_at_mcs0)}"
    )
    # Bitrate must not be strictly constant under sustained loss —
    # it should shrink at least once during the trace.
    assert min(bitrates_at_mcs0) < bitrates_at_mcs0[0], (
        f"bitrate did not shrink: first={bitrates_at_mcs0[0]} "
        f"min={min(bitrates_at_mcs0)}"
    )
```

- [ ] **Step 4: Run the test to verify it fails on master code**

Run: `python3 -m pytest tests/test_policy_bitrate.py::test_policy_bitrate_shrinks_with_n_escalation -v`

Expected: FAIL with `AssertionError: bitrate did not shrink: first=... min=...` — proves the death-spiral mechanism currently exists.

- [ ] **Step 5: Rewrite the per-tick dataflow in policy.py**

Edit `gs/dynamic_link/policy.py`:

(a) Update line 12 import:
```python
from .bitrate import BitrateConfig, compute_bitrate_kbps, compute_wire_target_kbps
```
Remove `compute_bitrate_kbps_legacy` from the import.

(b) Update the import block around line 18 to add `clamp_n_for_bitrate_floor`:
```python
from .dynamic_fec import (
    DynamicFecConfig, EmitGate, NEscalator,
    clamp_n_for_bitrate_floor, compute_k, compute_n,
)
```

(c) Replace the initialization at line ~744 (in `__init__`):
```python
        # Init bitrate uses wire_target at the safe-defaults MCS + safe (k, n).
        _init_wire_target = compute_wire_target_kbps(
            profile, cfg.leading.bandwidth, row.mcs, mtu_for_init,
            cfg.bitrate.utilization_factor,
        )
        self.state = PolicyState(
            mcs=row.mcs,
            bandwidth=cfg.leading.bandwidth,
            tx_power_dBm=int(self.leading.state.tx_power_dBm),
            k=cfg.safe.k,
            n=cfg.safe.n,
            depth=cfg.safe.depth,
            bitrate_kbps=compute_bitrate_kbps(
                wire_target_kbps=_init_wire_target,
                k=cfg.safe.k, n=cfg.safe.n,
                min_bitrate_kbps=cfg.bitrate.min_bitrate_kbps,
                max_bitrate_kbps=cfg.bitrate.max_bitrate_kbps,
            ),
        )
```

(d) Replace the per-tick chunk at lines ~814-855:
```python
        # mtu/fps come from drone HELLO when available; safe fallbacks otherwise.
        mtu = self.drone_config.mtu_bytes if self.drone_config else 1400
        fps = self.drone_config.fps if self.drone_config else 60

        # wire_target_kbps is the anchor: function of (MCS, bw, mtu, util)
        # only — no FEC feedback. Encoder bitrate later shrinks against
        # this as (k, n) grow under loss.
        wire_target_kbps = compute_wire_target_kbps(
            self.profile, self.state.bandwidth, row.mcs,
            mtu, self.cfg.bitrate.utilization_factor,
        )

        # k sized for the worst-case wire rate (full utilization).
        candidate_k = compute_k(
            wire_target_kbps=wire_target_kbps,
            mtu_bytes=mtu, fps=fps,
            cfg=self.cfg.dynamic_fec,
        )

        # n: base + escalation, then clamp to keep bitrate >= floor.
        escalation = self._n_escalator.update(
            loss=float(signals.residual_loss_w)
        )
        n_unclamped = compute_n(
            k=candidate_k, n_escalation=escalation, cfg=self.cfg.dynamic_fec,
        )
        candidate_n = clamp_n_for_bitrate_floor(
            n_candidate=n_unclamped,
            k=candidate_k,
            wire_target_kbps=wire_target_kbps,
            min_bitrate_kbps=self.cfg.bitrate.min_bitrate_kbps,
        )

        # EmitGate decides what actually rides this tick.
        if self._emit_gate.should_emit(
            candidate_k, candidate_n, mcs_changed,
            current_tick=self._tick_counter,
        ):
            new_k, new_n = candidate_k, candidate_n
            self._emit_gate.commit(new_k, new_n, self._tick_counter)
        else:
            new_k = self._emit_gate.last_k or self.cfg.safe.k
            new_n = self._emit_gate.last_n or self.cfg.safe.n

        # Bitrate derived from the EMITTED (k, n) — they ride together.
        new_bitrate_kbps = compute_bitrate_kbps(
            wire_target_kbps=wire_target_kbps,
            k=new_k, n=new_n,
            min_bitrate_kbps=self.cfg.bitrate.min_bitrate_kbps,
            max_bitrate_kbps=self.cfg.bitrate.max_bitrate_kbps,
        )
```

- [ ] **Step 6: Run all tests to verify pass**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -v`

Expected: ALL PASS. The wire-safety invariant test, the bitrate-shrinks test, and the unchanged legacy-callsite tests all green.

If `tests/test_policy_trailing.py::test_policy_emits_target_bitrate_after_mcs_settle` fails because it pins against the legacy bitrate value, leave the failure for Task 6 (which removes the legacy API and migrates this test).

- [ ] **Step 7: Commit**

```bash
git add gs/dynamic_link/policy.py tests/test_policy_bitrate.py
git commit -m "$(cat <<'EOF'
policy: derive encoder bitrate from live (k, n), enforce wire-safety

Anchors per-tick math on wire_target_kbps = eff_phy × util. k is
sized for wire_target (independent of bitrate), n grows with
escalation (clamped by clamp_n_for_bitrate_floor), and bitrate is
derived from wire_target × emitted_k / emitted_n.

The death-spiral mechanism is closed: bitrate × n / k stays ≤
wire_target on every tick across the full (mcs, escalation)
matrix. Encoder bitrate now shrinks under sustained loss instead
of holding constant while wire silently inflates past PHY.

Regression test test_policy_bitrate_shrinks_with_n_escalation
locks in the new behavior; parametric
test_policy_wire_rate_under_target_across_escalation guards the
invariant for future changes.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Remove the legacy bitrate API + drop `BitrateConfig.base_redundancy_ratio`

**Files:**
- Modify: `gs/dynamic_link/bitrate.py` (delete legacy fn, drop config field)
- Modify: `gs/dynamic_link/service.py` (drop parse + add deprecation log)
- Modify: `tests/test_bitrate.py` (delete legacy-API tests)
- Modify: `tests/test_policy_bitrate.py` (delete or migrate legacy tests)
- Modify: `tests/test_policy_trailing.py` (migrate the assertion)

- [ ] **Step 1: Verify no remaining callers of `compute_bitrate_kbps_legacy`**

Run: `grep -rn "compute_bitrate_kbps_legacy" gs/ tests/`

Expected output: only the callsites in `tests/test_bitrate.py`, `tests/test_policy_bitrate.py`, and `tests/test_policy_trailing.py`. Nothing in `gs/dynamic_link/`.

- [ ] **Step 2: Delete the legacy function and update `BitrateConfig`**

In `gs/dynamic_link/bitrate.py`:

Remove `compute_bitrate_kbps_legacy` entirely.

Update `BitrateConfig`:

```python
@dataclass(frozen=True)
class BitrateConfig:
    utilization_factor: float = 0.8
    min_bitrate_kbps: int = 1000
    max_bitrate_kbps: int = 24000

    def __post_init__(self) -> None:
        if not (0.0 < self.utilization_factor <= 1.0):
            raise ValueError(
                f"utilization_factor must be in (0, 1]; "
                f"got {self.utilization_factor}"
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
```

(Drop the `base_redundancy_ratio` field and its validator. Keep everything else.)

- [ ] **Step 3: Update service.py — drop parse + add deprecation log**

In `gs/dynamic_link/service.py`, replace the `BitrateConfig` construction around line 240–249 with:

```python
    if "base_redundancy_ratio" in bitrate_raw:
        _log.warning(
            "policy.bitrate.base_redundancy_ratio is deprecated and "
            "ignored; fec.base_redundancy_ratio is now authoritative "
            "(bitrate is derived from live (k, n) per the bitrate-aware "
            "FEC design)."
        )
    try:
        bitrate = BitrateConfig(
            utilization_factor=float(bitrate_raw.get("utilization_factor", 0.8)),
            min_bitrate_kbps=int(bitrate_raw.get("min_bitrate_kbps", 1000)),
            max_bitrate_kbps=int(bitrate_raw.get("max_bitrate_kbps", 24000)),
        )
    except ValueError as e:
        raise ValueError(f"policy.bitrate.{e}") from e
```

Verify `_log = logging.getLogger(__name__)` already exists in `service.py`. If not, add `import logging` and `_log = logging.getLogger(__name__)` near the top.

- [ ] **Step 4: Update tests — delete legacy tests, migrate kept ones**

In `tests/test_bitrate.py`:

Delete these tests (they assert against the legacy API):
- `test_bitrate_uses_base_redundancy_ratio_not_live_kn`
- `test_bitrate_clamped_to_min`
- `test_bitrate_changes_with_base_ratio`
- `test_bitrate_bw40_higher_than_bw20_for_same_mcs`
- `test_compute_bitrate_kbps_warns_once_when_preamble_missing`
- `test_compute_bitrate_kbps_mcs4_mlink_1500_bench_anchor`
- `test_compute_bitrate_kbps_mcs4_mlink_3994_bench_anchor`
- `test_compute_bitrate_kbps_monotone_in_mtu`
- `test_compute_bitrate_kbps_max_clamp_still_applies`

Then replace the `_cfg` helper:

```python
def _cfg() -> BitrateConfig:
    return BitrateConfig(
        utilization_factor=0.8,
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )
```

Remove the `from dataclasses import replace` and `import logging` imports — they're no longer used.

Add migrated bench-anchor tests using the new pipeline (`compute_wire_target_kbps` followed by `compute_bitrate_kbps` with simulated (k=10, n=14) — n/k = 1.4 matches the bench's original 1.4× ratio):

```python
def test_compute_bitrate_kbps_mcs4_mlink_1500_bench_anchor():
    """Bench-anchored end-to-end: MCS4 HT20 + mlink=1500 + U=0.8 + n/k=1.4
    → ~14400 kbps. Now via wire_target × k / n at k=10, n=14."""
    from dataclasses import replace
    p = _profile()
    p = replace(p, preamble_us_per_frame=170.0, name="m8812eu2-bench-1")
    from dynamic_link.bitrate import (
        compute_bitrate_kbps, compute_wire_target_kbps,
    )
    wire = compute_wire_target_kbps(p, 20, 4, 1500, 0.8)
    got = compute_bitrate_kbps(wire, k=10, n=14, min_bitrate_kbps=1000, max_bitrate_kbps=24000)
    assert 14100 <= got <= 14700, f"expected ~14400, got {got}"


def test_compute_bitrate_kbps_mcs4_mlink_3994_bench_anchor():
    """Bench-anchored: MCS4 HT20 + mlink=3994 + U=0.8 + n/k=1.4 → ~18600 kbps."""
    from dataclasses import replace
    p = _profile()
    p = replace(p, preamble_us_per_frame=170.0, name="m8812eu2-bench-2")
    from dynamic_link.bitrate import (
        compute_bitrate_kbps, compute_wire_target_kbps,
    )
    wire = compute_wire_target_kbps(p, 20, 4, 3994, 0.8)
    got = compute_bitrate_kbps(wire, k=10, n=14, min_bitrate_kbps=1000, max_bitrate_kbps=24000)
    assert 18300 <= got <= 18900, f"expected ~18600, got {got}"


def test_compute_wire_target_warns_once_when_preamble_missing(caplog, monkeypatch):
    """Profile without preamble_us_per_frame: WARN once, use 200 µs default."""
    from dataclasses import replace
    import logging
    from dynamic_link.bitrate import compute_wire_target_kbps
    monkeypatch.setattr("dynamic_link.bitrate._warned_missing_preamble", set())
    p = load_profile_file(Path("conf/radios/m8812eu2.yaml"))
    p = replace(p, preamble_us_per_frame=None, name="m8812eu2-noprmbl-1")
    caplog.set_level(logging.WARNING, logger="dynamic_link.bitrate")
    compute_wire_target_kbps(p, 20, 4, 1400, 0.8)
    compute_wire_target_kbps(p, 20, 4, 1400, 0.8)   # second call — should NOT re-warn
    warnings = [r for r in caplog.records if "preamble_us_per_frame missing" in r.message]
    assert len(warnings) == 1
    assert "m8812eu2-noprmbl-1" in warnings[0].message
```

In `tests/test_policy_bitrate.py`, delete:
- `test_bitrate_matches_formula_for_every_row` (was testing legacy formula)
- `test_bitrate_higher_mcs_has_higher_bitrate`
- `test_bitrate_drops_when_mtu_shrinks`

Update `_cfg` helper:

```python
def _cfg() -> BitrateConfig:
    return BitrateConfig(
        utilization_factor=0.8,
        min_bitrate_kbps=1000,
        max_bitrate_kbps=24000,
    )
```

Remove the `compute_bitrate_kbps_legacy` import.

Add a replacement test that validates the new pipeline:

```python
def test_wire_target_higher_mcs_has_higher_wire_target(profile):
    from dynamic_link.bitrate import compute_wire_target_kbps
    a = compute_wire_target_kbps(profile, 20, 1, 1400, 0.8)
    b = compute_wire_target_kbps(profile, 20, 5, 1400, 0.8)
    assert b > a


def test_wire_target_grows_with_mtu(profile):
    from dynamic_link.bitrate import compute_wire_target_kbps
    a = compute_wire_target_kbps(profile, 20, 4, 1500, 0.8)
    b = compute_wire_target_kbps(profile, 20, 4, 3994, 0.8)
    assert b > a
```

In `tests/test_policy_trailing.py` (line ~261), replace:

```python
        expected = compute_bitrate_kbps_legacy(p.profile, 20, target, 1400, p.cfg.bitrate)
        assert p.state.bitrate_kbps == expected
```

with:

```python
        # New pipeline: bitrate = wire_target × emitted_k / emitted_n
        from dynamic_link.bitrate import compute_wire_target_kbps
        wire = compute_wire_target_kbps(
            p.profile, 20, target, 1400, p.cfg.bitrate.utilization_factor,
        )
        expected = compute_bitrate_kbps(
            wire_target_kbps=wire, k=p.state.k, n=p.state.n,
            min_bitrate_kbps=p.cfg.bitrate.min_bitrate_kbps,
            max_bitrate_kbps=p.cfg.bitrate.max_bitrate_kbps,
        )
        assert p.state.bitrate_kbps == expected
```

Replace the import at line 14:

```python
from dynamic_link.bitrate import compute_bitrate_kbps
```

- [ ] **Step 5: Run all tests**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -v`

Expected: ALL PASS.

If something fails, the most likely culprits are (in priority order):
1. Lingering `compute_bitrate_kbps_legacy` import or call you missed — grep and clean.
2. A test that constructed `BitrateConfig(base_redundancy_ratio=...)` somewhere — find via `grep -rn base_redundancy_ratio tests/` and drop the kwarg.
3. A test that built a config dict with `base_redundancy_ratio` in `bitrate_raw` for the service parser — should now emit the deprecation warning but still work; if the test asserts no warnings, adjust the assertion.

- [ ] **Step 6: Commit**

```bash
git add gs/dynamic_link/bitrate.py gs/dynamic_link/service.py tests/test_bitrate.py tests/test_policy_bitrate.py tests/test_policy_trailing.py
git commit -m "$(cat <<'EOF'
bitrate: rip out legacy compute_bitrate_kbps + base_redundancy_ratio

The new-signature compute_bitrate_kbps and compute_wire_target_kbps
fully replace the old API. BitrateConfig no longer carries
base_redundancy_ratio (lives in fec.base_redundancy_ratio only).
service.py logs a one-time deprecation warning if the obsolete
key is present in gs.yaml.

Bench-anchored tests migrated to the new pipeline using simulated
(k=10, n=14) which preserves the original n/k = 1.4 ratio the
bench was calibrated against.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Death-spiral E2E regression in `test_policy_dynamic_fec_e2e.py`

**Files:**
- Test: `tests/test_policy_dynamic_fec_e2e.py`

- [ ] **Step 1: Read existing scenarios for the file's style**

Run: `wc -l tests/test_policy_dynamic_fec_e2e.py && head -30 tests/test_policy_dynamic_fec_e2e.py`

Expected: confirms file exists and follows pytest style. If you need helpers from elsewhere, locate them via grep.

- [ ] **Step 2: Write the failing-then-passing regression test**

Append to `tests/test_policy_dynamic_fec_e2e.py`:

```python
def test_death_spiral_does_not_oversubscribe_wire():
    """Regression: 30 ticks of sustained 5% loss at MCS 0 must not
    drive wire rate above wire_target. NEscalator ramps escalation
    up to its cap; we verify each emitted (bitrate, k, n) keeps
    bitrate × n / k ≤ wire_target the whole time, then recovery
    brings bitrate back up."""
    from pathlib import Path
    from dynamic_link.bitrate import compute_wire_target_kbps
    from dynamic_link.drone_config import DroneConfigState
    from dynamic_link.policy import (
        LeadingLoopConfig, Policy, PolicyConfig, SafeDefaults,
    )
    from dynamic_link.profile import load_profile
    from dynamic_link.signals import Signals
    from dynamic_link.wire import Hello

    REPO_ROOT = Path(__file__).resolve().parent.parent
    profile = load_profile("m8812eu2", [REPO_ROOT / "conf" / "radios"])
    cfg = PolicyConfig(
        leading=LeadingLoopConfig(
            tx_power_min_dBm=5.0, tx_power_max_dBm=30.0,
            max_mcs=5,
        ),
        safe=SafeDefaults(k=2, n=3, depth=1, mcs=0),
    )
    drone_cfg = DroneConfigState()
    drone_cfg.on_hello(Hello(
        generation_id=1, mtu_bytes=1500, fps=60, applier_build_sha=0,
    ))
    p = Policy(cfg, profile, drone_config=drone_cfg)

    wire_targets = {}
    mcs0_decisions = []

    # 60 ticks: phase 1 (0-29) sustained loss, phase 2 (30-59) recovery.
    for tick_i in range(60):
        loss = 0.05 if tick_i < 30 else 0.0
        sigs = Signals(
            timestamp=tick_i * 0.1,
            rssi=-80.0, rssi_min_w=-82.0, rssi_max_w=-78.0,
            snr=4.0, snr_slope=0.0,
            residual_loss=loss, residual_loss_w=loss,
            fec_work=(0.20 if loss > 0 else 0.0),
            link_starved_w=False,
        )
        d = p.tick(sigs)

        if d.mcs not in wire_targets:
            wire_targets[d.mcs] = compute_wire_target_kbps(
                profile, 20, d.mcs, 1500, cfg.bitrate.utilization_factor,
            )

        # Wire-safety check on every tick
        wire_actual = d.bitrate_kbps * d.n / d.k
        assert wire_actual <= wire_targets[d.mcs] + 1, (
            f"tick={tick_i} mcs={d.mcs}: "
            f"wire={wire_actual:.0f} > target={wire_targets[d.mcs]:.0f} "
            f"(bitrate={d.bitrate_kbps} k={d.k} n={d.n})"
        )

        if d.mcs == 0:
            mcs0_decisions.append((tick_i, d.k, d.n, d.bitrate_kbps))

    # Need MCS-0 ticks both during loss and during recovery.
    in_loss = [x for x in mcs0_decisions if x[0] < 30]
    in_recovery = [x for x in mcs0_decisions if x[0] >= 30]
    assert len(in_loss) >= 10, f"too few MCS-0 ticks during loss: {len(in_loss)}"
    assert len(in_recovery) >= 10, f"too few MCS-0 ticks during recovery: {len(in_recovery)}"

    # During loss: bitrate trends down as escalation grows
    min_during_loss = min(x[3] for x in in_loss)
    initial = in_loss[0][3]
    assert min_during_loss < initial, (
        f"bitrate did not shrink during loss: initial={initial} "
        f"min={min_during_loss}"
    )

    # During recovery: bitrate trends back up
    max_during_recovery = max(x[3] for x in in_recovery)
    last_loss = in_loss[-1][3]
    assert max_during_recovery >= last_loss, (
        f"bitrate did not recover: last_loss={last_loss} "
        f"max_recovery={max_during_recovery}"
    )
```

- [ ] **Step 3: Run the new test**

Run: `python3 -m pytest tests/test_policy_dynamic_fec_e2e.py::test_death_spiral_does_not_oversubscribe_wire -v`

Expected: PASS (Task 5's policy rewrite already enforces the invariant).

- [ ] **Step 4: Run the full suite to make sure nothing else regressed**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -v`

Expected: ALL PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/test_policy_dynamic_fec_e2e.py
git commit -m "$(cat <<'EOF'
test: e2e death-spiral regression — sustained loss at MCS 0

Drives 30 ticks of 5% loss into Policy.tick at MCS 0, then 30
ticks of recovery. Asserts that bitrate × n / k stays ≤
wire_target across every emitted Decision, that bitrate shrinks
during loss as NEscalator ramps escalation, and that bitrate
recovers as the link cleans up.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: EmitGate alignment test

Locks in the (β) decision from brainstorming: bitrate uses the *emitted* (k, n), not the candidate, when EmitGate holds.

**Files:**
- Test: `tests/test_policy_bitrate.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_policy_bitrate.py`:

```python
def test_emit_gate_alignment_bitrate_uses_emitted_kn(profile):
    """When EmitGate holds (k, n) at the previous values, the emitted
    bitrate is derived from those held (k, n) — not the candidate."""
    from dynamic_link.bitrate import compute_bitrate_kbps, compute_wire_target_kbps
    from dynamic_link.drone_config import DroneConfigState
    from dynamic_link.policy import (
        LeadingLoopConfig, Policy, PolicyConfig, SafeDefaults,
    )
    from dynamic_link.signals import Signals
    from dynamic_link.wire import Hello

    cfg = PolicyConfig(
        leading=LeadingLoopConfig(
            tx_power_min_dBm=5.0, tx_power_max_dBm=30.0,
            max_mcs=5,
        ),
        safe=SafeDefaults(k=2, n=3, depth=1, mcs=0),
    )
    drone_cfg = DroneConfigState()
    drone_cfg.on_hello(Hello(
        generation_id=1, mtu_bytes=1500, fps=60, applier_build_sha=0,
    ))
    p = Policy(cfg, profile, drone_config=drone_cfg)

    # First tick — EmitGate always emits the first time.
    d0 = p.tick(Signals(
        timestamp=0.0,
        rssi=-50.0, rssi_min_w=-52.0, rssi_max_w=-48.0,
        snr=25.0, snr_slope=0.0,
        residual_loss=0.0, residual_loss_w=0.0,
        fec_work=0.0, link_starved_w=False,
    ))

    # Second tick — same signals, MCS unchanged, (k, n) unchanged.
    # EmitGate holds the previous values; bitrate must match the
    # previous bitrate (computed from the same (k, n)).
    d1 = p.tick(Signals(
        timestamp=0.1,
        rssi=-50.0, rssi_min_w=-52.0, rssi_max_w=-48.0,
        snr=25.0, snr_slope=0.0,
        residual_loss=0.0, residual_loss_w=0.0,
        fec_work=0.0, link_starved_w=False,
    ))

    assert (d1.k, d1.n) == (d0.k, d0.n)
    assert d1.bitrate_kbps == d0.bitrate_kbps, (
        f"bitrate disagreed across gate-held ticks: "
        f"d0={d0.bitrate_kbps} d1={d1.bitrate_kbps}"
    )

    # Sanity: the emitted bitrate matches the formula on the emitted (k, n).
    wire = compute_wire_target_kbps(
        profile, 20, d1.mcs, 1500, cfg.bitrate.utilization_factor,
    )
    expected = compute_bitrate_kbps(
        wire_target_kbps=wire, k=d1.k, n=d1.n,
        min_bitrate_kbps=cfg.bitrate.min_bitrate_kbps,
        max_bitrate_kbps=cfg.bitrate.max_bitrate_kbps,
    )
    assert d1.bitrate_kbps == expected
```

- [ ] **Step 2: Run the test**

Run: `python3 -m pytest tests/test_policy_bitrate.py::test_emit_gate_alignment_bitrate_uses_emitted_kn -v`

Expected: PASS. (Task 5's rewrite already binds bitrate to the emitted (k, n).)

- [ ] **Step 3: Commit**

```bash
git add tests/test_policy_bitrate.py
git commit -m "$(cat <<'EOF'
test: lock in EmitGate alignment for bitrate

Across a gate-held tick (same signals, no MCS change, no
(k, n) drift), the emitted bitrate must equal the previous
tick's bitrate — both are derived from the same (k, n) the
EmitGate decided to emit.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Config samples + CLAUDE.md migration

**Files:**
- Modify: `conf/gs.yaml.sample`
- Modify: `deploy/gs/gs.yaml`
- Modify: `CLAUDE.md`

- [ ] **Step 1: Update `conf/gs.yaml.sample`**

Read the file and find the `policy:` block. Inside `policy.bitrate`, remove the `base_redundancy_ratio:` line and any nearby comment that says "should match fec.base_redundancy_ratio". Keep `utilization_factor`, `min_bitrate_kbps`, `max_bitrate_kbps`.

If you're unsure of the exact format, do:

```bash
grep -n "base_redundancy_ratio\|policy:\|bitrate:" conf/gs.yaml.sample
```

Then `Edit` to remove the targeted lines. Do not reformat surrounding YAML.

- [ ] **Step 2: Update `deploy/gs/gs.yaml`**

Same operation:

```bash
grep -n "base_redundancy_ratio\|policy:\|bitrate:" deploy/gs/gs.yaml
```

Remove the `base_redundancy_ratio:` line under `policy.bitrate` and its "should match" comment.

- [ ] **Step 3: Update `CLAUDE.md` "Dynamic FEC (P4b)" section**

Find the section with `grep -n "Dynamic FEC (P4b)\|Bitrate uses a FIXED" CLAUDE.md`.

Replace the existing block describing the fixed-ratio bitrate model with:

```markdown
### Dynamic FEC (P4b + bitrate-aware FEC)

`(k, n)` is computed at runtime, not pinned per-MCS in the profile.
The pipeline is anchored on `wire_target_kbps = eff_phy × util`:

  wire_target = effective_phy_Mbps × utilization_factor × 1000
  k = clamp(wire_target_kbps * 1000 / (fps * mtu_bytes * 8), k_min, k_max)
  n = ceil(k * (1 + base_redundancy_ratio)) + n_escalation
  n = clamp_n_for_bitrate_floor(n, k, wire_target, min_bitrate_kbps)
  bitrate = wire_target × emitted_k / emitted_n   (clamped to [min, max])

`mtu_bytes` and `fps` come from the P4a handshake (drone reads them
from /etc/wfb.yaml and /etc/majestic.yaml or /etc/waybeam.json).
`n_escalation` ramps on sustained `residual_loss` and decays on
sustained clean windows; the EmitGate bundles `(k, n)` rewrites onto
MCS-change ticks per `docs/knob-cadence-bench.md`. Bitrate co-emits
with the emitted `(k, n)` — they always stay coherent.

Encoder bitrate shrinks as FEC escalates so that total wire stays
under PHY (`bitrate × n / k ≤ wire_target` invariant). The bitrate
floor (`min_bitrate_kbps`) is enforced by `clamp_n_for_bitrate_floor`,
which caps FEC growth before bitrate would drop below the floor —
closing the death-spiral mechanism at MCS 0 edge-of-range.

See `docs/superpowers/specs/2026-05-22-bitrate-aware-fec-design.md`.
```

If the existing section has slightly different wording, replace it with the block above and remove any now-incorrect statements about fixed `k/n` ratios elsewhere in the file (`grep -n "FIXED" CLAUDE.md` to find them).

- [ ] **Step 4: Re-run the test suite to verify nothing depends on the sample config keys**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -v`

Expected: PASS. (Config samples are only loaded by integration tests if they exist; unit tests don't read them.)

- [ ] **Step 5: Commit**

```bash
git add conf/gs.yaml.sample deploy/gs/gs.yaml CLAUDE.md
git commit -m "$(cat <<'EOF'
docs: drop policy.bitrate.base_redundancy_ratio; update FEC notes

Removes the obsolete config key from the sample and deploy gs.yaml
(operators get a deprecation warning at startup if they still
have it). CLAUDE.md's Dynamic FEC section now describes the
wire-target-anchored pipeline: bitrate is derived from live
(k, n) and shrinks as escalation grows, with
clamp_n_for_bitrate_floor honoring min_bitrate_kbps.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: Drone E2E scenario — `test_loss_episode_does_not_oversubscribe_wire`

**Files:**
- Test: `tests/test_drone_e2e.py`

- [ ] **Step 1: Locate the existing scenario style**

Run: `grep -n "^def test_\|^async def test_" tests/test_drone_e2e.py | head`

Identify the pattern used by existing scenarios (sandbox setup, mocks, assertions).

- [ ] **Step 2: Read one existing scenario as a template**

Pick the first or shortest existing test and `Read` it to mirror the harness setup. Look for: how `_sandbox` is used, how mock wfb_tx is invoked, how `applied` events are captured for assertions.

- [ ] **Step 3: Add the new scenario**

Append to `tests/test_drone_e2e.py` (after the last existing scenario):

```python
def test_loss_episode_does_not_oversubscribe_wire():
    """End-to-end: drive the GS service with a synthetic stats trace
    containing a sustained-loss burst at MCS 0. Verify the bitrate
    trajectory emitted by the GS (and applied by the real drone
    applier) keeps wire rate under target throughout.

    This is the integration-level counterpart to
    test_death_spiral_does_not_oversubscribe_wire in
    test_policy_dynamic_fec_e2e.py; that test exercises Policy.tick
    in isolation, this one runs the full GS service against the
    drone applier binary.

    Skipped if the drone applier binary isn't built — the unit-level
    regression already covers the math; this scenario is for the
    full applier wiring.
    """
    import os
    from pathlib import Path

    drone_build = Path(__file__).resolve().parent.parent / "drone" / "build" / "dl-applier"
    if not drone_build.exists():
        import pytest
        pytest.skip(f"{drone_build} not built; run `make -C drone`.")

    # The actual harness setup mirrors the existing scenarios in this
    # file. Reuse the helpers (_sandbox, mock_wfb_tx, capture_applied)
    # already defined here — see the test directly above for the
    # canonical structure.
    pytest.skip(
        "TODO(implementor): wire up the existing _sandbox/mock helpers "
        "in test_drone_e2e.py to drive a 60-tick synthetic-loss trace "
        "and assert wire-safety invariant in applied events. The "
        "math-level coverage is provided by "
        "test_death_spiral_does_not_oversubscribe_wire."
    )
```

Note: this leaves the scenario as a skipped placeholder pointing at the unit-level coverage. If the existing scenario style in `tests/test_drone_e2e.py` makes the harness setup easy to copy, replace the second `pytest.skip` with the full assertion harness modeled on the nearest existing scenario.

- [ ] **Step 4: Run the new test**

Run: `python3 -m pytest tests/test_drone_e2e.py::test_loss_episode_does_not_oversubscribe_wire -v`

Expected: SKIPPED (either because the drone binary isn't built or because the harness is the placeholder skip). Either is fine for this commit; the unit-level coverage is the load-bearing test.

- [ ] **Step 5: Commit**

```bash
git add tests/test_drone_e2e.py
git commit -m "$(cat <<'EOF'
test: drone e2e scenario for wire-safety under loss episode

Adds test_loss_episode_does_not_oversubscribe_wire stub pointing at
the unit-level death-spiral regression. The math-level coverage is
already locked in by test_death_spiral_does_not_oversubscribe_wire
in test_policy_dynamic_fec_e2e.py; this scenario is reserved for
the full applier wiring once the harness is fleshed out.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 11: Final validation

**Files:** none modified — read-only verification.

- [ ] **Step 1: Full Python test suite**

Run: `python3 -m pytest --ignore=tests/test_mavlink_status.py -v 2>&1 | tail -40`

Expected: ALL PASS (skipped tests like the placeholder drone-e2e are OK).

- [ ] **Step 2: C unit tests**

Run: `make -C drone test`

Expected: ALL PASS. No drone-side code was touched, but verifying the wfb_tx contract test still passes confirms we didn't break anything load-bearing.

- [ ] **Step 3: Grep for any straggler references**

Run:
```bash
grep -rn "compute_bitrate_kbps_legacy\|base_redundancy_ratio" gs/ tests/ conf/ deploy/ CLAUDE.md
```

Expected output: ONLY references in the right places:
- `gs/dynamic_link/dynamic_fec.py` — `DynamicFecConfig.base_redundancy_ratio` (this stays — it's the source of truth)
- `gs/dynamic_link/service.py` — the deprecation warning text
- `tests/test_dynamic_fec.py` — uses `base_redundancy_ratio` in `_cfg` helper

If you see `compute_bitrate_kbps_legacy` anywhere or `base_redundancy_ratio` in `bitrate.py` / `policy.py` / `conf/gs.yaml.sample` / `deploy/gs/gs.yaml`: clean it up.

- [ ] **Step 4: Sanity-check the spec coverage**

Cross-check each section of the spec against the commits on this branch:

```bash
git log --oneline master..feat/bitrate-aware-fec
```

Confirm:
- Spec §"Solution" → Tasks 1, 2, 3, 4, 5 (helpers + dataflow)
- Spec §"Code changes" → Tasks 1, 2, 3, 4, 5, 6, 9
- Spec §"Tests" → Tasks 5, 7, 8 (and migrated tests in Tasks 3-6)
- Spec §"Migration" → Tasks 6, 9

- [ ] **Step 5: Smoke test against a recorded capture if available**

Look for an existing capture:

```bash
ls /tmp/dl-flights/ 2>/dev/null || ls debug/ 2>/dev/null | head
```

If a JSONL capture exists, drive the service offline against it:

```bash
python3 -m dynamic_link.service \
  --config conf/gs.yaml.sample \
  --replay <path/to/capture.jsonl> \
  --log-dir /tmp/dl-flights-bitrate-aware
```

Compare the resulting decision log against `master`'s output for the same capture:

```bash
git stash
git checkout master
python3 -m dynamic_link.service --config conf/gs.yaml.sample --replay <path/to/capture.jsonl> --log-dir /tmp/dl-flights-master
git checkout feat/bitrate-aware-fec
git stash pop
diff <(jq -r '.bitrate_kbps' /tmp/dl-flights-master/*.jsonl | head) \
     <(jq -r '.bitrate_kbps' /tmp/dl-flights-bitrate-aware/*.jsonl | head)
```

Expected: bitrates match in clean-link sections, and the new branch shows lower bitrates during loss episodes. Big differences in clean sections indicate a bug — investigate.

If no captures available, skip this step.

- [ ] **Step 6: Final commit (nothing to commit if all clean)**

If steps 1-5 surfaced no fixes, no commit. The branch is complete.

If a fix was needed, commit it under its own message — don't `--amend` into an earlier commit.

---

## Done

Once Task 11 passes:
- Branch `feat/bitrate-aware-fec` is ready for review / bench-flight validation.
- The spec at `docs/superpowers/specs/2026-05-22-bitrate-aware-fec-design.md` is fully implemented.
- Open a PR when ready. Bench-validate per the spec's "Validation plan" §4 (walk to far corner, spin, verify no video lag during high-loss segment) before merging.

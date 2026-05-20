# MTU-aware encoder bitrate

Status: design
Owner: Gilang
Date: 2026-05-20

## Summary

Teach `gs/dynamic_link/bitrate.py::compute_bitrate_kbps` that 802.11
airtime depends on packet payload size, not just on PHY rate. Today
the formula treats `data_rate_Mbps_LGI[bw][mcs]` as the available
throughput and assumes `utilization_factor` covers all overhead. That
assumption holds at the reference MTU (~3994 B on the SSC338Q rig)
but breaks at smaller MTUs: per-frame PHY/MAC overhead (~157 µs/frame
on the RTL8812EU) consumes a larger fraction of airtime as packets
shrink, dropping the practical wire-bandwidth ceiling well below the
raw PHY rate. The encoder then overshoots, wfb_tx queues fill, and
video latency grows — exactly the symptom captured in
`docs/mlink-airtime-bench.md`.

Goals:

- Encoder bitrate setpoint respects per-MTU airtime, not just PHY rate.
- `utilization_factor` becomes pure operator safety margin (no longer
  silently compensating for airtime overhead).
- Per-chipset overhead is sourced from one place — the radio profile.
- Backward compatible: existing `gs.yaml` is untouched, profiles
  without the new field fall back to a safe default with a one-shot
  WARNING.

Non-goals:

- Touching `gs/dynamic_link/predictor.py`. It has the parallel
  airtime-blindness issue but is a separate change; this spec calls
  it out as the obvious follow-up.
- A reactive feedback loop off the RTL8812EU `pubq_free_page`
  counter. Useful for verification (see `docs/mlink-airtime-bench.md`
  §"Driver-side saturation signal") but couples the GS controller to
  a chipset-specific procfs path and risks fighting the dynamic-FEC
  `n_escalation` loop.
- A per-airframe `gs.yaml` override for the overhead constant. YAGNI
  — operators who need a per-airframe tweak can fork the profile via
  the existing `radio_profiles_dir`.
- New observability surface (no decision-packet field, no OSD line,
  no flight-log column). The existing `br=` field in
  `dl-applier.log` already moves visibly when the cap kicks in; the
  bench doc covers forensic post-mortems.

## Background

`docs/mlink-airtime-bench.md` derives and validates the model. Brief
recap:

- Per-802.11-frame fixed cost (PHY preamble + MAC header at PHY rate
  + DIFS+backoff) is ~157 µs on this chipset, broadcast, HT20.
- Effective throughput per packet:
  `eff_phy_bps = mtu_bits / (preamble_s + mtu_bits / phy_bps)`.
- Saturation crossover on MCS4+mlink=1500 was bracketed empirically
  between wire 18.4 Mb/s (instant) and 19.7 Mb/s (laggy); model fit
  matches all five tested points.

Today's `compute_bitrate_kbps`:

```python
phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
raw_kbps = phy_Mbps * 1000.0 * cfg.utilization_factor * k_over_n
```

This uses raw PHY rate, so when the operator drops `mlink` from 3994
to 1500 the computed setpoint stays the same (~19500 kbps at MCS4
HT20 with `utilization_factor=0.7`, `base_redundancy_ratio=0.4`) even
though the practical ceiling drops by ~22%.

MTU is already plumbed: the drone reports it in the DLHE HELLO
(`drone/src/dl_applier.c::send_hello`), and `gs/dynamic_link/drone_config.py`
exposes it as `DroneConfig.mtu_bytes`. `policy.py:840` already reads
it for the dynamic-FEC `compute_k` call, two lines after the
`compute_bitrate_kbps` call at line 833.

## Detailed design

### The formula

```
mtu_bits      = mtu_bytes * 8
eff_phy_bps   = mtu_bits / (preamble_s + mtu_bits / phy_bps)
encoder_kbps  = clamp(
                  eff_phy_Mbps * 1000 * utilization_factor * k_over_n,
                  min_bitrate_kbps,
                  max_bitrate_kbps,
                )
```

`preamble_s = preamble_us_per_frame * 1e-6`, sourced from the radio
profile (falls back to a constant — see below).

Monotonicity properties (verified by test):

- Strictly non-decreasing in `mtu_bytes` (bigger packets are always
  at least as efficient).
- Strictly non-decreasing in `phy_Mbps` (faster PHY is always at
  least as good).
- Approaches raw `phy_Mbps × utilization × k/n` as `mtu_bytes → ∞`
  (the existing formula is the asymptote).
- New formula's result is always **≤** the existing formula's result
  at any finite MTU. The change can never raise encoder bitrate
  above what today's code would pick.

### Radio profile change

`conf/radios/m8812eu2.yaml` — add one field:

```yaml
preamble_us_per_frame: 170   # 2026-05-20 bench, HT20 LGI broadcast.
                              # See docs/mlink-airtime-bench.md for the fit.
```

The bench doc's raw physical fit is 157 µs against the on-air payload
(naluSize = mlink − 100). The implementation feeds `mtu_bytes = mlink`
straight into `compute_bitrate_kbps`, so the YAML constant absorbs
the small OpenIPC framing offset between mlink and naluSize. 170 µs
reproduces the published bench table cells when called with
mtu_bytes = mlink.

`gs/dynamic_link/profile.py::RadioProfile` gains an optional field:

```python
preamble_us_per_frame: float | None = None
```

`from_yaml` reads `preamble_us_per_frame` if present, leaves `None`
otherwise. No validation rejection — a profile without the field is
still well-formed.

### Loader behaviour for missing field

`DEFAULT_PREAMBLE_US = 200.0` lives in `bitrate.py`. Rationale:
conservative relative to the measured 157 µs (≈27% higher), so
unknown chipsets bias toward lower encoder bitrate, never higher.
Magic-number-class constant; documented in the same comment block as
the formula.

When `compute_bitrate_kbps` sees `profile.preamble_us_per_frame is
None`, it uses `DEFAULT_PREAMBLE_US` and logs once at WARNING:

```
preamble_us_per_frame missing from radio profile '<name>';
using conservative default <N> µs. See docs/mlink-airtime-bench.md.
```

"Once" is per-profile, tracked by a module-level `set[str]` keyed on
`profile.name`. Two reasons:

1. The bitrate formula is called every tick (10 Hz); a per-call
   WARNING would flood `gs.log` in seconds.
2. A `set` keyed by name makes it deterministic across profile
   reloads (none today, but the daemon restarts cleanly on
   `SIGHUP`-style config changes).

### `bitrate.py` API change

```python
def effective_phy_Mbps(
    phy_Mbps: float, mtu_bytes: int, preamble_us: float,
) -> float:
    """Per-packet airtime model. Returns the wire bandwidth a stream
    of mtu_bytes packets can actually achieve at this PHY rate."""
    mtu_bits = mtu_bytes * 8
    preamble_s = preamble_us * 1e-6
    payload_s = mtu_bits / (phy_Mbps * 1_000_000.0)
    return mtu_bits / (preamble_s + payload_s) / 1_000_000.0


def compute_bitrate_kbps(
    profile: RadioProfile,
    bandwidth: int,
    mcs: int,
    mtu_bytes: int,           # NEW
    cfg: BitrateConfig,
) -> int:
    phy_Mbps = profile.data_rate_Mbps_LGI[bandwidth][mcs]
    preamble_us = (
        profile.preamble_us_per_frame
        if profile.preamble_us_per_frame is not None
        else _default_preamble_with_warn(profile.name)
    )
    eff_phy_Mbps = effective_phy_Mbps(phy_Mbps, mtu_bytes, preamble_us)
    k_over_n = 1.0 / (1.0 + cfg.base_redundancy_ratio)
    raw_kbps = eff_phy_Mbps * 1000.0 * cfg.utilization_factor * k_over_n
    return int(max(cfg.min_bitrate_kbps,
                   min(cfg.max_bitrate_kbps, raw_kbps)))
```

`effective_phy_Mbps` is exported (no leading underscore) so tests can
exercise it directly without going through profile loading.

### `policy.py` call sites

Two call sites get an `mtu_bytes` argument.

**Line 760** — `Policy.__init__`. The drone hasn't sent HELLO yet,
so `drone_config` (if present) is not synced. Use the same fallback
the dynamic-FEC and predictor paths use at policy.py:840 and 872:

```python
mtu_for_init = (
    self.drone_config.mtu_bytes
    if self.drone_config and self.drone_config.is_synced()
    else 1400
)
bitrate_kbps=compute_bitrate_kbps(
    profile, cfg.leading.bandwidth, row.mcs,
    mtu_for_init, cfg.bitrate,
),
```

(`drone_config` may be `None` if the Policy is constructed without
one, e.g. in some unit tests; the existing dynamic-FEC fallback at
line 840 already handles this.)

**Line 833** — per-tick `Policy.tick`. `self.drone_config.mtu_bytes`
is in scope two lines below (line 840). Pass it through:

```python
mtu = self.drone_config.mtu_bytes if self.drone_config else 1400
new_bitrate_kbps = compute_bitrate_kbps(
    self.profile, self.state.bandwidth, row.mcs,
    mtu, self.cfg.bitrate,
)
# … existing compute_k call now reuses the same `mtu` variable …
```

Reusing the local `mtu` for both the bitrate and `compute_k` calls
avoids two near-identical guarded reads. The existing comment on
line 838 already documents the fallback semantics.

### `gs.yaml` change

None. `BitrateConfig` is unchanged. The deprecated `mtu_bytes` key
(removed in P4b — see `service.py:205-211`) stays in the
legacy-rejection list.

`utilization_factor` semantics shift in spirit (now pure safety
margin) but not in default value (0.8) or accepted range. Existing
deployments running `utilization_factor: 0.7` get a slightly lower
encoder bitrate at small MTU, no change at large MTU.

## Testing

Extend `tests/test_bitrate.py` (already exists, covers
`base_redundancy_ratio` semantics, the `min` clamp, and HT20-vs-HT40
ordering). Every new case takes a `mtu_bytes` argument now, so the
existing test signatures need a trivial update first (pass
`mtu_bytes=1400` to keep current behaviour).

| # | Case | Asserts |
|---|------|---------|
| 1 | `effective_phy_Mbps(39, 1500, 157)` | ≈ 25.1 Mb/s (±0.1) |
| 2 | `effective_phy_Mbps(39, 3994, 157)` | ≈ 32.5 Mb/s (±0.1) |
| 3 | `compute_bitrate_kbps(m8812eu2, 20, 4, 1500, default_cfg)` | ≈ 14400 kbps (±300) — matches bench table |
| 4 | `compute_bitrate_kbps(m8812eu2, 20, 4, 3994, default_cfg)` | ≈ 18600 kbps (±400) — matches bench table |
| 5 | Profile without `preamble_us_per_frame` loads; warns once on first `compute_bitrate_kbps` call; later calls don't re-warn | uses 200 µs default |
| 6 | `compute_bitrate_kbps` monotone non-decreasing in `mtu_bytes` over [500, 1500, 3994, 8000] | strict `<=` chain |
| 7 | `max_bitrate_kbps` clamp still applies (mtu=8000, MCS7, max=12000 → returns 12000) | cap honoured |

Existing tests:

- `tests/test_policy_bitrate.py` — both existing tests
  (`test_bitrate_matches_formula_for_every_row`,
  `test_bitrate_higher_mcs_has_higher_bitrate`) need to be updated
  to construct a Policy with a synced `drone_config` (mtu=3994 keeps
  current behaviour) and the formula assertion needs the new
  effective-PHY term. Add one new case: at MCS4, the emitted bitrate
  with `mtu=1500` is strictly less than with `mtu=3994`.
- `tests/test_profile.py` (if it exists, otherwise via the existing
  profile-loading test) — add a case for a YAML missing the new
  field, asserting `RadioProfile.preamble_us_per_frame is None`.

Bench-anchored numerical tests (rows 3, 4) make this change
self-documenting: if someone changes the formula and the bench
numbers drift, those tests fail and point straight at
`docs/mlink-airtime-bench.md`.

## Migration / operator impact

- Drop-in. No `gs.yaml` change, no `drone.conf` change, no wire
  format change.
- Operators running with `mlink=3994` (the common case) see encoder
  bitrate drop by ~5% at MCS4 (~19500 → ~18600 kbps). This matches
  the bench's "safe at 80% airtime" margin; the previous setpoint
  was running at ~84% which worked but had no headroom.
- Operators running with `mlink=1500` see encoder bitrate drop from
  whatever they had to ≤ ~14400 kbps at MCS4 — eliminating the
  latency symptom that motivated the spec.
- Operators with a custom radio profile that lacks
  `preamble_us_per_frame` see the WARNING once at start, then get
  the 200-µs default behaviour (more conservative than 157 µs).

## Risks & open questions

- **The 200 µs default is calibrated against one chipset.** If
  someone adds a profile for, e.g., an AR9271 with materially
  different MAC airtime, the default may be too generous or too
  conservative. Mitigation: WARNING tells the operator to calibrate;
  the bench doc has a refit procedure.
- **`utilization_factor` defaults stay at 0.8.** With the new
  formula, 0.8 corresponds to the "feel instant" boundary observed
  in the bench (~80% airtime). If field deployment shows that
  number wants a different default, change it in a separate commit
  with its own justification — not bundled here.
- **No interaction with `n_escalation`.** Dynamic FEC's
  `max_n_escalation=4` adds up to +24% wire bandwidth under sustained
  loss; that headroom comes out of `1 - utilization_factor = 0.20`.
  Tight but consistent with the existing design. If escalation under
  loss pushes us back into saturation, that's a sign the operator's
  `utilization_factor` is too high — same as today.

## Follow-up (out of scope here, mentioned for the record)

`gs/dynamic_link/predictor.py` has the parallel airtime-blindness
issue: `block_air = n × per_packet_airtime_us / 1000` treats
per-packet airtime as a fixed gs.yaml constant (default 80 µs)
instead of deriving it from `(mtu, preamble, phy_rate)`. The same
`profile.preamble_us_per_frame` would feed it cleanly; the
`predictor.per_packet_airtime_us` knob in gs.yaml would become
redundant. Separate spec when the predictor's underestimate
actually bites in flight data.

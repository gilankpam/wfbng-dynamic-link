# Remove dl IDR path; receive PixelPilot IDR tokens on the drone

Status: design
Owner: Gilang
Date: 2026-05-21

## Summary

Stop generating IDR requests in the dynamic-link GS controller.
Instead, have the drone applier (`dl-applier`) bind a UDP listener
on port **11223** to receive the IDR-token bursts that
PixelPilot_rk already sends from the GS host when it detects RTP
sequence gaps or decode stalls. The listener triggers the existing
`dl_backend_enc_request_idr()` HTTP path to the encoder, reusing
the existing `min_idr_interval_ms` throttle.

The dl GS-side IDR machinery — `idr_burst.py`, the
`Decision.idr_request` field, the `TrailingLoop.tick()` trigger,
the wire-format `FLAG_IDR_REQUEST` bit, and the three drone-side
dispatch sites in `dl_applier.c` — is removed entirely.

## Motivation

The dl IDR trigger is `signals.residual_loss_w > 0` — post-FEC
loss measured on the GS by wfb-ng. This is a **proxy** for the
event we actually care about: a frame the decoder will see as
corrupt. PixelPilot measures the same event **directly** at the
decoder, by tracking RTP sequence numbers
(`PixelPilot_rk/src/gstrtpreceiver.cpp:466–505`,
`maybe_track_rtp_sequence`). The direct signal is:

- **More accurate** — fires on real gaps, not on FEC-recovered
  loss that the decoder never sees.
- **Lower latency** — PixelPilot reacts as packets are demuxed;
  dl's path waits up to one 10 Hz wfb-ng stats window (~100 ms)
  before residual_loss is even visible to the controller.
- **Already present** — PixelPilot is the only GS player we
  support, and its IDR-burst transmitter is unconditional.

The dl path is duplication of a worse signal. Keeping it costs:

- A wire-format flag bit (`FLAG_IDR_REQUEST`).
- A burst transmitter (`idr_burst.py`).
- A controller hook (`TrailingLoop.tick()` IDR triggering).
- Three drone-side dispatch sites in `dl_applier.c`.
- Tests covering all of the above.

## Non-goals

- **Supporting non-PixelPilot GS players** (gstreamer pipeline,
  QGC video, ffplay). Out of scope. Setups that don't run
  PixelPilot lose IDR entirely after this change — accepted.
- **Making PixelPilot's burst parameters configurable.** They are
  hardcoded `static constexpr` in `gstrtpreceiver.cpp:118–133`
  (3 packets × 100 ms, repeated 3× at 100–150 ms, 500 ms gap
  cooldown, 700 ms decode-stall cooldown). Anyone wanting to
  retune forks PixelPilot. The dl drone-side throttle does not
  need to track these values.
- **Reporting drone-side IDR events back to GS over MAVLink.**
  Observability stays where it is: PixelPilot's `[IDR]` log lines
  on the GS, the drone's `dl_event_log` for the encoder HTTP
  outcome. No new MAVLink message types.
- **Token validation or anti-spoof.** The drone accepts any UDP
  datagram on port 11223. PixelPilot sends a random 3-byte token
  but the drone has no way to know which token to expect; the
  `min_idr_interval_ms` throttle limits attack-surface impact to
  ~2 IDR/s worst case. The wfb-tunnel link is the network
  boundary; restricting bind interface (below) further narrows
  exposure.
- **A non-wire-bit reversal path.** `FLAG_IDR_REQUEST` is fully
  removed from `wire.py` and `dl_wire.h`. Reversal means
  reverting the commit; we don't carry reserved bits.

## Background

### dl IDR path today

1. `gs/dynamic_link/policy.py` `TrailingLoop.tick()` (~line 577–692)
   sets `idr=True` when `signals.residual_loss_w > 0`.
2. `Policy.tick()` constructs a `Decision` with
   `idr_request=True` (~line 965).
3. `service.py` calls `idr_burster.trigger(decision)` when
   `decision.idr_request` is set.
4. `gs/dynamic_link/idr_burst.py` `IdrBurster._pump()` sends N
   (default 4) copies of the decision packet at 20 ms spacing on
   the return link, each with a fresh sequence number to defeat
   drone-side dedup.
5. The wire format (`gs/dynamic_link/wire.py:41`,
   `drone/src/dl_wire.h:36`) encodes the request as
   `FLAG_IDR_REQUEST = 0x01` in the flags byte of the 32-byte
   DLK1 decision packet.
6. `drone/src/dl_applier.c` lines 391–393, 415–417, 488–490
   dispatch on the flag and call
   `dl_backend_enc_request_idr(be, now)`
   (`drone/src/dl_backend_enc.c:232–252`), which:
   - Checks `(now_ms - last_idr_ms) < min_idr_interval_ms`
     (default 500 ms) and drops the request if so.
   - Otherwise HTTP-GETs `/request/idr` on the local encoder
     (majestic/waybeam) and arms the throttle.

### PixelPilot IDR path today

1. `PixelPilot_rk/src/gstrtpreceiver.cpp:466–505`
   `maybe_track_rtp_sequence()` watches incoming RTP packet
   sequence numbers; any `diff > 1` indicates a missing packet.
2. With a 500 ms cooldown,
   `maybe_request_idr_for_rtp_gap()` (lines 446–464) triggers
   `request_idr_bursts()` (lines 625–690).
3. `send_idr_token_to_ip()` (lines 591–612) opens an AF_INET
   SOCK_DGRAM socket and `sendto()`s a 3-byte random ASCII
   token plus newline (6 bytes including header) to the last-hop
   IP on UDP port **11223**.
4. Burst: 3 packets, 100 ms apart. Repeated 3 times,
   100 ms apart (stream) or 150 ms (record).
5. A separate decode-stall path (`gstrtpreceiver.cpp:737–739`)
   fires the same bursts after 700 ms with no decoded frames,
   on a 700 ms cooldown.
6. `maybe_mark_idr_received()` (lines 568–589) watches decoded
   NAL units (H.264 type 5, H.265 types 16–21) and clears the
   pending flag.

**Last-hop IP** is derived from the RTP packet's source address
in `maybe_update_last_hop_from_buffer()`. On a wfb-ng tunnel, the
last-hop address is the drone's tunnel address — PixelPilot does
not need explicit drone-IP configuration.

### dl-applier event loop

`drone/src/dl_applier.c:291–298` runs a single-threaded `poll(2)`
over four file descriptors:

- `listen_fd` — the DLK1 decision UDP socket.
- `tick_fd` — periodic tick timerfd.
- `gap_fd` — phase-2 gap timerfd.
- `hello_timer_fd` — DLHE handshake timer.

Adding a fifth fd (`idr_udp_fd`) is a one-line `pfds[]` extension
plus a branch in the dispatch ladder. No threading required.

## Architecture

### Drone-side: new UDP listener fold-in

A new module `drone/src/dl_idr_listen.c/h` exposes:

```c
typedef struct dl_idr_listen dl_idr_listen_t;

dl_idr_listen_t *dl_idr_listen_open(const char *bind_addr, uint16_t port);
int  dl_idr_listen_fd(const dl_idr_listen_t *l);
size_t dl_idr_listen_drain(dl_idr_listen_t *l);  /* read until EAGAIN; returns datagram count */
void dl_idr_listen_close(dl_idr_listen_t *l);
```

- `dl_idr_listen_open()` binds an `AF_INET` `SOCK_DGRAM` socket
  to `bind_addr:port` with `SO_REUSEADDR` and non-blocking. If
  `port == 0`, returns NULL and the caller skips registering the
  fd.
- `dl_idr_listen_fd()` returns the bound fd for poll
  registration.
- `dl_idr_listen_drain()` reads datagrams into a discard buffer
  until `recvfrom()` returns `EAGAIN`, and returns how many
  datagrams were consumed (for logging only). It does **not**
  call into the encoder backend itself — the main loop owns
  that side-effect.

`dl_applier.c` changes:

- Open the listener after `listen_fd` setup, conditional on
  `cfg->idr_listen_port != 0`.
- Extend `pfds[]` from 4 to 5 slots.
- After `poll()` returns, check `pfds[4].revents & POLLIN`:
  - Call `dl_idr_listen_drain()` to consume all queued datagrams
    (a burst can be 3 packets, all arriving within a few ms).
  - Call `dl_backend_enc_request_idr(be, now)` exactly once per
    poll wake. The throttle inside the backend silently drops
    redundant calls within `min_idr_interval_ms`, so collapsing
    a burst into one call is the natural behavior.
- Log received bursts via `dl_event_log` at debug level (count,
  source addr), and the HTTP outcome at info level (existing
  behavior of the backend).

### Drone-side: OSD counter

The drone applier's top OSD line (`dl_osd_write_status`,
`drone/src/dl_osd.c:68`) currently renders:

```
MCS5 12M (8,14)d2 TX18 R-50 | &B T&T W&W CPU&C
```

Add a running IDR-request counter as `I<n>` immediately before the
`|` separator:

```
MCS5 12M (8,14)d2 TX18 R-50 I7 | &B T&T W&W CPU&C
```

Semantics: increment once per poll-wake in which the IDR listener
had at least one datagram pending, regardless of throttle outcome.
This matches "logical PixelPilot IDR requests" — a 3-packet burst
counts as 1, and throttled-collapsed bursts also count as 1 each.
Monotonically increasing since process start; no reset.

Implementation:

- `dl_osd_t` gains a `uint32_t idr_requests` field.
- `dl_osd.h` exposes `void dl_osd_bump_idr(dl_osd_t *o);` which
  increments the field. Cheap; safe to call from the main poll
  loop alongside `dl_backend_enc_request_idr()`.
- `dl_osd_write_status` formats `I%u` into the existing
  `snprintf` of `status_line`. Width grows by ~5 chars; the
  128-byte buffer has plenty of room.
- The counter is process-local (lives in `dl_osd_t`). No
  persistence across applier restarts — acceptable; matches how
  the rest of dl observability works.

No `tests/drone/test_osd.c` exists today; add one with a single
test that bumps the counter a known number of times, calls
`dl_osd_write_status` against a tempfile path, and greps for
`I<n>` in the rendered output. Keep it tight — this is a
display-only counter, not a correctness-critical surface.

### Drone-side: config

`conf/drone.conf.sample` and `drone/src/dl_config.c` gain two
keys:

- `idr_listen_port = 11223` — 0 disables the listener. Range
  1–65535. Default 11223 matches PixelPilot's hardcoded
  `kIdrUdpPort`.
- `idr_listen_addr = 0.0.0.0` — bind address. Default permissive.
  Operators with a single wfb-tunnel ingress can restrict to the
  tunnel interface address.

The existing `min_idr_interval_ms` (default 500) remains and
continues to govern encoder API rate. No new throttle.

### Drone-side: removals

`drone/src/dl_applier.c`:

- Lines 391–393, 415–417, 488–490: drop the
  `if (decision->flags & DL_FLAG_IDR_REQUEST)
   dl_backend_enc_request_idr(...)` blocks. The three sites are
  the direct tick path, the phase-2 gap timer, and the
  apply-pending path respectively; all three become dead after
  the wire bit goes away.

`drone/src/dl_wire.h`:

- Remove `DL_FLAG_IDR_REQUEST = 0x01`. The flags byte is
  preserved (other bits may exist); only the IDR bit and any
  helper macros around it are deleted.

`drone/src/dl_wire.c`: remove any encode/decode surfacing of the
flag (none expected beyond the bitwise pass-through, but
verified during implementation).

`drone/src/dl_inject.c`:

- Remove the `--idr` CLI long option (lines 24, 58, 115).
- Remove the `IDR` token from the dry-run printout (line 272).
  Other dry-run fields are unchanged.

`tests/drone/test_wire.c`:

- Drop the `DL_FLAG_IDR_REQUEST` setter/assertion (lines 9, 38).

`tests/drone/`:

- Add unit tests for `dl_idr_listen` (open / drain / EAGAIN
  loop / port=0 disabled path).

### GS-side: removals

`gs/dynamic_link/wire.py`:

- Remove `FLAG_IDR_REQUEST` constant and all references.
- Remove the IDR-flag encode path from the decision serializer.
- Other flag bits, if any, are unaffected.

`gs/dynamic_link/idr_burst.py`:

- **Delete the file.**

`gs/dynamic_link/policy.py`:

- Remove `idr` local in `TrailingLoop.tick()` and the
  `signals.residual_loss_w > 0` → `idr=True` block.
- Remove `idr_request` field from `Decision` (and any callers
  that read it).
- Remove `idr_request=idr` from the `Decision(...)` construction
  in `Policy.tick()`.

`gs/dynamic_link/service.py`:

- Remove `idr_burster` instantiation and the `trigger()` call.
- Remove the import.

`gs/dynamic_link/flight_log.py`:

- Remove the `idr_request` field from the per-tick JSONL
  record schema. Old log files retain the field; replay/report
  tools tolerate its absence (below).

`gs/tools/dl_replay.py:70`:

- Drop the `idr_request=bool(rec["idr_request"])` line. Old logs
  with the field replay fine — the field is just ignored.

`gs/tools/dl_report.py:189-193`:

- Drop the `summary["idr_requests"]` aggregation. Old report
  consumers (if any) lose the key; acceptable.

`conf/gs.yaml.sample`:

- Remove any `idr_burst:` block (`count`, `interval_ms`). The
  controller no longer reads them.

`tests/`:

- Delete `tests/test_idr_burst.py`.
- `tests/test_wire_contract.py`: drop the `idr_request=True`
  fixture case (line 85). The `idr_request=False` baseline case
  (line 58) is also dropped since the field no longer exists on
  `Decision`. Contract is regenerated; passes.
- `tests/test_phase2_e2e.py`: delete
  `test_gs_wire_idr_flag_triggers_idr_request` (lines 68+) and
  remove `idr_request=False` from the helper at line 36.
- `tests/test_flight_log.py`: drop `idr_request=False` from the
  fixture decision (line 258).
- `tests/test_dl_review.py` (lines 31, 38, 42),
  `tests/test_dl_replay.py` (line 27),
  `tests/test_phase3_e2e.py` (line 71),
  `tests/bench/per_knob_loss_bench.py` (line 134): remove
  `idr_request` from the dict/decision constructors.
- Remove IDR triggering from policy tests; assert that
  `residual_loss_w > 0` no longer produces an `idr_request`
  side-effect (since the field is gone, this is mostly
  deletions).

### Wire contract

The byte-for-byte contract between `wire.py` and `dl_wire.h`
stays anchored by `tests/test_wire_contract.py`, which diffs
Python's encoder output against `dl-inject --dry-run`. Removing
`FLAG_IDR_REQUEST` is a symmetric change on both sides; the test
regenerates fixtures and passes. The vendored `tx_cmd.h` is
unaffected.

`dl-inject`'s `--idr` long option is removed alongside the wire
bit (see `drone/src/dl_inject.c` changes above).

## Operational notes

- **PixelPilot's burst parameters are hardcoded.** If a future
  operator wants tighter or looser bursts, they must fork
  PixelPilot. dl's drone-side throttle (`min_idr_interval_ms`)
  defends the encoder regardless.
- **The encoder HTTP API is unchanged.** majestic and waybeam
  still see `GET /request/idr` exactly as before — the trigger
  source moved from "wfb-stats residual_loss on GS" to
  "PixelPilot RTP gap on GS, relayed through dl-applier UDP
  listener", but the encoder doesn't observe the difference.
- **README update.** The component diagram and "what
  dl-applier does" section need an edit to note the IDR listener
  port and the PixelPilot dependency for IDR functionality.
- **Migration.** A new drone binary running against an old GS
  (still sending `FLAG_IDR_REQUEST`) silently ignores the bit
  (it's no longer in the flag set). An old drone binary running
  against a new GS never receives IDR requests over the wire,
  but the new PixelPilot listener isn't running yet — operators
  must upgrade both sides together, or accept a window with no
  IDR. Acceptable for this project's phase.

## Testing

- **Drone C unit:** `tests/drone/test_idr_listen.c` covers open,
  bind, drain-until-EAGAIN, close, and port=0 disabled path.
- **Drone C integration:** extend `tests/drone/test_main.c` to
  fire UDP packets at the bound port and assert that
  `dl_backend_enc_request_idr` is called (via the existing mock
  encoder HTTP server in the e2e harness). Confirm three packets
  arriving within 5 ms produce exactly one HTTP call thanks to
  throttle.
- **Python E2E:** `tests/test_drone_e2e.py` spawns a real
  `dl-applier` against the mock encoder. Add a new test that
  `sendto()`s a 6-byte ASCII payload to the applier's IDR port
  and asserts the mock encoder received one
  `GET /request/idr`. Repeat the send three times at 20 ms
  spacing and assert still exactly one HTTP call.
- **Python wire contract:** `tests/test_wire_contract.py`
  regenerates and passes with no flag bit.
- **Removed tests:** drop `tests/test_idr_burst.py` and any
  policy-test cases that assert `idr_request` is set.

## Out of scope (deferred)

- Reporting IDR events back to GS via MAVLink STATUSTEXT.
  Considered and rejected for this change; PixelPilot's own log
  is the operator's UI.
- A non-PixelPilot fallback. Will be reconsidered only if a
  concrete need arises.
- Bind-interface restriction defaults. Default `0.0.0.0` keeps
  setup simple; operators with stricter requirements set
  `idr_listen_addr` explicitly.

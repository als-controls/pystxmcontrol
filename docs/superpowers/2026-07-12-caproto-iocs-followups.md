# caproto IOC layer -- follow-ups (2026-07-12)

Recorded during the final whole-branch review of `feature/caproto-iocs`. Terse, one item each.

- ~~Hardware fly branch restructure: `moveLine` is currently called per-DAQ inside the fly loop; the real hardware path must configure ALL DAQs first, then fly ONCE gathering all `getLines`. `E712Motor` also lacks the trajectory attributes the sim path assumes. Resolve during the beamline benchmark with David.~~ -- RESOLVED 2026-07-24 (shared DAQ service: fly loop arms all DAQs, flies once; see specs/2026-07-24-shared-daq-service-design.md)
- ~~Multi-E712 DAQ mapping: `plan_fleet` now raises on >1 E712Controller group (each currently absorbs ALL daq entries); design and implement per-controller DAQ ownership.~~ -- RESOLVED 2026-07-24 (DAQs are standalone CA services; plan_fleet no longer absorbs or raises)
- caproto threading client doesn't resolve put futures on `ErrorResponse` -- a rejected put (e.g. out-of-limits) can leave the caller blocked up to the full timeout (30s ophyd default) instead of failing fast. Consider an upstream caproto issue.
- `base.py` uses `print` instead of `logging`.
- `VELO` is only applied by polling, not immediately on write.
- `STOP` issued in the idle-to-moving transition window can be silently dropped.
- `config.py` takes `simulation` from the first primary motor per controller; mismatched `simulation` flags across motors on the same controller should warn, not silently pick one.
- Derived motor offset/units raise a hard `KeyError` if missing from config, instead of falling back to `build_motor`-style defaults.
- `conftest.py` mutates `os.environ` directly for test scoping; should restore/patch more defensively.
- Shared `run_slice_ioc` helper to dedupe the near-identical `main()` in every IOC module.
- `spawn_ioc` test helper ignores its `port` parameter.
- The `:AXIS` non-default-index write path is untested.
- Hardware DAQ `getPoint` blocking-IO audit: confirm it never blocks the asyncio loop on real hardware the way it's assumed to in sim.
- MCL driver loads a vendor `.so` in `__init__`, making sim unusable on hosts without that library installed; consider lazy-loading or an upstream PR.
- Upstream PR to David with the caproto IOC layer changes once stabilized.
- CSM / iocular integration for deploying these IOCs.
- Spec #3: Lightfall EPICS migration using this IOC layer.

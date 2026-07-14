# PyChrono Environment Setup

PyChrono is **not** a pip/`requirements.txt` dependency of this project. It is a
heavy native package that must come from the `projectchrono` conda channel, and
it is only needed to run the `pychrono_smoke` backend and the
`@pytest.mark.pychrono` tests. Everything else in this repo (heuristic backend,
mock Chrono backend, schema/registry tests, Streamlit app) is import-safe and
runs with a plain `pip install -r requirements.txt` environment.

## Verified working environment

- conda environment: `chrono`
- Python executable: `C:\Users\lahee\anaconda3\envs\chrono\python.exe`
- Python: `3.12.9`
- PyChrono package: `pychrono 9.0.1` (build `py312h8c5e8c1_6621`, channel `projectchrono`)
- Verified imports: `pychrono` (core) only. `pychrono.vehicle` / `pychrono.irrlicht` currently
  **fail to import** in this env -- see "pychrono.vehicle / pychrono.irrlicht import failure" below.
  (An earlier note in this file claimed both imported successfully; that could not be reproduced
  on 2026-07-14 and should be treated as stale until re-verified.)
- Verified smoke result: box-drop scenario completes with real contact detection via `system.GetNumContacts()`
  (uses `pychrono` core only, not `pychrono.vehicle`)

Latest verified smoke metrics:

```text
status=completed
initial_z=1.0
minimum_z=0.10048812734971739
final_z=0.10203217634322208
final_vz=0.03706795009166446
max_contact_count=4
first_contact_time_s=0.4150000000000003
contact_detection_source=system.GetNumContacts
contact_detected=True
wall_time_s=0.0464177999997446
max_speed_mps=4.120199999999982
```

Check the exact installed build at any time with:

```bash
conda list -n chrono pychrono
```

## Do not do this

- Do **not** add `pychrono` to `requirements.txt`.
- Do **not** `pip install pychrono`.
- Do **not** rely on direct `import pychrono` checks in Streamlit import paths.
- Do **not** create a fresh conda env for PyChrono without checking free disk space first.

PyChrono plus its native dependencies can be several GB. Conda's package cache
and env store default to the `C:` drive (`C:\Users\<user>\anaconda3\{envs,pkgs}`).
If `C:` is low on space, reuse the existing `chrono` env or point
`pkgs_dirs`/`envs_dirs` at a drive with room before installing.

## Creating a new environment (only if `chrono` is unavailable)

```bash
conda create -n chrono-mvp -c projectchrono -c conda-forge python=3.12 pychrono streamlit pandas numpy matplotlib pytest pyyaml
conda activate chrono-mvp
```

If `C:` free space is limited, redirect conda's package cache to another drive
first, e.g.:

```bash
conda config --add pkgs_dirs E:\conda-pkgs
```

## Running the project against the verified environment

```bat
conda activate chrono
cd /d "C:\K_SRC\mobility_twin_mvp"

python -c "import pychrono; print(pychrono.__file__)"
python -m pytest -m pychrono -v
python -m pytest -q
python -m compileall src
python -m streamlit run app.py --browser.gatherUsageStats false
```

## Collision system note

`ChSystemNSC()`/`ChSystemSMC()` do **not** have a collision system attached by
default in this PyChrono build (`system.GetCollisionSystem()` returns `None`
until one is set). `src/chrono/smoke_scenario.py::_make_system` calls:

```python
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
```

Without this call, bodies with collision shapes can silently pass through each
other: no contacts are reported even though collision shapes and
`EnableCollision(True)` are set on the bodies.

## pychrono.vehicle / pychrono.irrlicht import failure (2026-07-14, unresolved)

`import pychrono.vehicle` and `import pychrono.irrlicht` both fail in the
`chrono` env with:

```text
ImportError: DLL load failed while importing _vehicle: DLL 초기화 루틴을 찾을 수 없습니다.
```

(same error for `_irrlicht`). `import pychrono` (core) always succeeds, but see
"Native loading is unpredictably slow" below -- "fast" is not reliable.
Ruled out:

- **Not a PATH issue.** Explicitly adding `Library/bin` via
  `os.add_dll_directory(...)` before the import does not fix it.
- **Not missing/corrupt files.** `Chrono_vehicle.dll`, `Chrono_irrlicht.dll`,
  `Irrlicht.dll`, etc. are present in `Library/bin` at plausible sizes, and
  `_vehicle.pyd`/`_irrlicht.pyd` are present in `site-packages/pychrono` at
  plausible sizes (not 0 bytes / truncated).

The error is "DLL init routine could not be executed" (not "module not
found"), meaning the DLL loads but its own init code fails -- consistent
with a real install/build defect in this specific `pychrono` build, not an
environment configuration problem this project can work around. A
`pychrono` reinstall (`conda install -c projectchrono --force-reinstall
pychrono`) has not been attempted yet (that's a disk-heavy operation --
confirm before running it, and check free `C:` space first per "Do not do
this" above).

**Important instability**: a fresh `import pychrono.vehicle` in a brand-new
process fails cleanly in ~4s. But when attempted in a process that already
built other Chrono objects (e.g. inside a pytest session, after earlier
tests already created `ChSystemNSC`/rover bodies), the same import has been
observed to **hang for 60s+** instead of failing cleanly. Treat any
in-process `import pychrono.vehicle` check as unsafe in a long-lived
process; check it in a fresh subprocess with a hard timeout instead (see
`tests/test_scm_pilot.py::_pychrono_vehicle_importable_in_subprocess` for
the pattern). `src/experiments/scm_pilot`'s SCM path (which needs
`pychrono.vehicle` for `SCMTerrain`) is blocked on this.

## Native loading is unpredictably slow -- not just pychrono.vehicle (2026-07-14)

This turned out to be broader than the vehicle/irrlicht import failure above.
Building a plain `ChSystemNSC` + Bullet collision + rigid terrain + rover
scenario (`src/experiments/rigid_transfer_pilot`, `pychrono` core only, no
`pychrono.vehicle` anywhere) was timed 3 times in a row in fresh processes:

```text
run 1: hung past 60s (killed)
run 2: 3.8s
run 3: 2.3s
```

So this is not specific to the vehicle/irrlicht modules -- it looks like
native DLL-loading instability in this `pychrono` install/environment in
general, roughly 1-in-3-to-4 chance of a bad hang on any given process start
that touches Chrono. The actual simulation stepping itself is fast once a
scenario is built (600 steps in ~0.07s) -- the unpredictable part is purely
the first native-library touch in a process.

Practical consequences:

- Any pytest that builds a real Chrono scenario must not do so in-process
  inside the main test run -- isolate it in a subprocess with a hard timeout
  and skip on timeout, same pattern as the `pychrono.vehicle` check above.
  See `tests/test_rigid_transfer_pilot.py::_run_pilot_condition_in_subprocess`.
- `scripts/run_rigid_transfer_pilot.py` and `scripts/run_scm_pilot.py` are
  plain single-process CLI scripts with no such protection -- if a run
  appears to hang, that is expected behavior given the above, not
  necessarily a bug. Killing and re-running is a reasonable first response.
- Root cause not identified (candidates: antivirus/Windows Defender
  real-time scanning of freshly-touched `.pyd`/`.dll` files, some other
  first-touch disk/OS contention). Not investigated further -- would need
  tooling (Process Monitor) beyond what's practical here.

## Stronger evidence it's filesystem/AV scanning, not pychrono itself (2026-07-15)

`tests/test_pychrono_smoke.py::test_pychrono_availability_is_import_safe` --
which calls `get_pychrono_availability()`, a function that by design **never
imports pychrono** (only `importlib.util.find_spec(...)` and
`importlib.metadata.version(...)`, see `src/chrono/availability.py`'s own
docstring) -- reproducibly hung past 45s at the exact same point in test
order across three separate `pytest` runs. `--collect-only` (which imports
every test module but runs nothing) completed in 1.36s with no hang, ruling
out a collection-time/import problem in this project's own code.

This means the hang isn't specific to actually loading a native `.pyd`/`.dll`
-- even a bare filesystem lookup for whether `pychrono`/`pychrono.vehicle`
exist can stall. That points squarely at something intercepting filesystem
access to the `chrono` conda env's `site-packages/pychrono` directory (a
real-time antivirus scanner is the most likely candidate on Windows) rather
than anything about the Chrono binaries or this project's code.

**If you want to actually try to fix this** (not attempted -- changes
system security settings, needs your own judgment call): add a Windows
Defender (or whatever AV is active) exclusion for
`C:\Users\<user>\anaconda3\envs\chrono\` and re-test. If that resolves the
hangs, it confirms AV scanning as the cause. Until then, treat every
in-process pychrono-adjacent call (imports, `find_spec`, scenario building)
as capable of hanging unpredictably, isolate it in a subprocess with a hard
timeout wherever it runs inside automation, and expect to kill/retry manual
CLI runs (`scripts/run_rigid_transfer_pilot.py`, `scripts/run_scm_pilot.py`).

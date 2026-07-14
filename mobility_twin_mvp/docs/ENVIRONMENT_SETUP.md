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
- Verified imports: `pychrono`, `pychrono.vehicle`, `pychrono.irrlicht`
- Verified smoke result: box-drop scenario completes with real contact detection via `system.GetNumContacts()`

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

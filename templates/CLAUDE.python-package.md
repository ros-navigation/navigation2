# <PKG_NAME> — assistant guide

Python (`ament_python`) or Python-heavy package.

## Layout
- `<pkg>/` — Python module(s)
- `test/` — pytest / `ament_*` tests
- `setup.py` + `setup.cfg` (ament_python) **or** `CMakeLists.txt` with `ament_python_install_package` (mixed)

## Lint
Subject to **ament_flake8**, **ament_pep257**, **ament_mypy**, **codespell**, **isort** — all run via `pre-commit`. Don't hand-format; let pre-commit rewrite.

mypy strict-mode overrides for ROS type stubs live in `tools/pyproject.toml` `[tool.mypy.overrides]`. If a new third-party import is missing stubs, add it there rather than scattering `# type: ignore`.

## How to add / change
- Public API change → bump version in `package.xml`, document in migration guide.
- New launch arg → also update `nav2_bringup/params/nav2_params.yaml` if it surfaces a server param.

## Tests
- `colcon test --packages-select <pkg>`
- pytest binary: `python -m pytest <pkg>/test/`

## Docs to update
- `docs.nav2.org` → tutorial pages for any public Python API surface change (especially `nav2_simple_commander`).

## Pitfalls
- Launch files (`.launch.py`) are linted under mypy when wired in `package.xml`. Untyped ROS message imports usually need an explicit `# type: ignore[attr-defined]` or an entry in pyproject.toml overrides.

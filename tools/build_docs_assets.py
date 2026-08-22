"""Run every experiment and copy its figures into the documentation tree.

The ``results/`` directory is gitignored -- it is regenerated output. The
documentation site, however, has to render on GitHub Pages without anyone
running a simulation first, so the figures the articles refer to are copied
into ``docs/assets/figures/`` and committed.

Usage::

    .venv/bin/python tools/build_docs_assets.py          # run experiments, then copy
    .venv/bin/python tools/build_docs_assets.py --copy-only

``--copy-only`` skips the (slow) experiment run and just refreshes the copies
from whatever is already in ``results/``.
"""

from __future__ import annotations

import argparse
import pathlib
import runpy
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
FIGURES = ROOT / "docs" / "assets" / "figures"


def run_experiments() -> None:
    scripts = sorted((ROOT / "experiments").glob("exp*.py"))
    for script in scripts:
        print(f"--- running {script.name} " + "-" * (54 - len(script.name)))
        runpy.run_path(str(script), run_name="__main__")


def copy_figures() -> int:
    FIGURES.mkdir(parents=True, exist_ok=True)
    pngs = sorted(RESULTS.glob("*.png"))
    if not pngs:
        print(f"no figures in {RESULTS}; run the experiments first", file=sys.stderr)
        return 0
    for png in pngs:
        shutil.copy2(png, FIGURES / png.name)
        print(f"copied {png.name}")
    return len(pngs)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--copy-only", action="store_true", help="do not re-run the experiments"
    )
    args = parser.parse_args()

    if not args.copy_only:
        run_experiments()
    n = copy_figures()
    print(f"\n{n} figure(s) in {FIGURES.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

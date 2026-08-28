from __future__ import annotations

from pathlib import Path
import sys
import argparse

# robust gegen IDE-Importprobleme
ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from acars_viewer.config.RadioConfig import RadioConfig
from acars_viewer.config.AcarsConfig import AcarsConfig
from acars_viewer.app.AcarsViewerApp import AcarsViewerApp

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ACARS Viewer")
    p.add_argument("--dll", required=True, help="Pfad zu rtlsdr.dll")
    p.add_argument("--freq", type=int, default=131_725_000)
    p.add_argument("--sr", type=int, default=1_024_000)
    p.add_argument("--ppm", type=int, default=0)
    p.add_argument("--plot", action="store_true")
    p.add_argument("--squelch", type=float, default=-45.0)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    radio_cfg = RadioConfig(
        dll_path=args.dll,
        center_freq_hz=args.freq,
        sample_rate_hz=args.sr,
        ppm=args.ppm,
    )
    acars_cfg = AcarsConfig(squelch_db=args.squelch)
    app = AcarsViewerApp(radio_cfg, acars_cfg, show_plot=args.plot)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
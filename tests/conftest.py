"""Make the package importable when running pytest from a source checkout.

An editable install (``pip install -e .``) also works and is what CI uses;
this fallback keeps ``pytest`` working straight after ``git clone``.
"""

import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "slow: regression against the published result; rebuilds the full grid",
    )

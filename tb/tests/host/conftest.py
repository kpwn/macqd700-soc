"""Ensure ``tools/`` is on sys.path so ``import m68kctl`` works.

Pytest loads this file automatically before collecting tests.
"""

import sys
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[3] / 'tools'
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

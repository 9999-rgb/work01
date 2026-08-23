"""Security contracts for the standalone task-history page."""

from __future__ import annotations

import re
import shutil
import subprocess
import unittest
from pathlib import Path


HISTORY_PATH = Path(__file__).resolve().parents[1] / "history.html"


def _inline_script() -> str:
    source = HISTORY_PATH.read_text(encoding="utf-8")
    matches = re.findall(r"<script>(.*?)</script>", source, flags=re.DOTALL)
    if not matches:
        raise AssertionError("history.html does not contain an inline script")
    return matches[-1]


class HistorySecurityContractTest(unittest.TestCase):
    def test_untrusted_history_fields_never_use_html_injection_sinks(self) -> None:
        source = _inline_script()
        for sink in (
            ".innerHTML",
            ".outerHTML",
            "insertAdjacentHTML",
            "document.write",
        ):
            self.assertNotIn(sink, source)
        self.assertIn("clearRenderedHistory()", source)
        self.assertIn("logoutBtn", source)


@unittest.skipUnless(shutil.which("node"), "Node.js is required for Web tests")
class HistoryJavascriptSyntaxTest(unittest.TestCase):
    def test_inline_javascript_parses(self) -> None:
        result = subprocess.run(
            ["node", "--check", "-"],
            input=_inline_script(),
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(0, result.returncode, msg=result.stderr)

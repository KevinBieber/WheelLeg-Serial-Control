"""PC 上位机入口。"""

from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication

from .config_loader import load_config
from .ui_main import MainWindow


def main() -> int:
    cfg = load_config()
    app = QApplication(sys.argv)
    win = MainWindow(cfg)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())

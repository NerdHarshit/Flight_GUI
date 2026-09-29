"""
CommandButton — A QPushButton that reflects per-command send/ack/fail state.

State colours (border flash):
  idle    → normal QSS purple
  pending → yellow border pulse
  sent    → yellow background tint (in-flight)
  acked   → bright green background tint (2 s then revert)
  failed  → red background tint (3 s then revert)
"""
from PyQt6.QtWidgets import QPushButton
from PyQt6.QtCore import pyqtSignal


# Background colours per state (as inline style overrides)
_STATE_STYLES = {
    "idle": "",   # blank → falls through to QSS
    "pending": (
        "background-color: rgba(200,180,0,0.25);"
        "border: 2px solid #FFD700;"
    ),
    "sent": (
        "background-color: rgba(200,160,0,0.35);"
        "border: 2px solid #FFC200;"
    ),
    "acked": (
        "background-color: rgba(0,200,100,0.35);"
        "border: 2px solid #00FF88;"
        "color: #00FF88;"
    ),
    "failed": (
        "background-color: rgba(220,40,40,0.35);"
        "border: 2px solid #FF4444;"
        "color: #FF6666;"
    ),
}

_BASE_STYLE = (
    "QPushButton {{ "
    "border-radius: 8px; padding: 7px 12px; font-weight: bold; "
    "font-family: 'Inter','Roboto',sans-serif; font-size: 12px; "
    "color: white; {extra} }}"
    "QPushButton:hover {{ filter: brightness(1.15); }}"
    "QPushButton:disabled {{ background-color: #2a2d35; color: #666; border: 1px solid #444; }}"
)


class CommandButton(QPushButton):
    """QPushButton that visually reflects command send/ack/fail lifecycle."""

    def __init__(self, label: str, cmd_type: int, warning: bool = False, parent=None):
        super().__init__(label, parent)
        self.cmd_type = cmd_type
        self.warning = warning
        self._state = "idle"
        self._apply_style()

    def set_state(self, state: str):
        """Update visual state. state is one of: idle/pending/sent/acked/failed."""
        if state == self._state:
            return
        self._state = state
        self._apply_style()

    def _apply_style(self):
        extra = _STATE_STYLES.get(self._state, "")
        if self._state == "idle":
            # Use the base purple from QSS, with warning tint if applicable
            if self.warning:
                extra = (
                    "background-color: qlineargradient("
                    "x1:0,y1:0,x2:1,y2:1,stop:0 #7a1000,stop:1 #3a0000);"
                    "border: 1px solid rgba(255,80,80,0.4);"
                )
            else:
                extra = (
                    "background-color: qlineargradient("
                    "x1:0,y1:0,x2:1,y2:1,stop:0 #8a2be2,stop:1 #4b0082);"
                    "border: 1px solid rgba(187,134,252,0.3);"
                )
        self.setStyleSheet(_BASE_STYLE.format(extra=extra))

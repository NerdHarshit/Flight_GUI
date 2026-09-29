"""
Debug Console Widget — Scrollable log panel showing raw packets
and GUI-raised warnings interleaved in a single scrollback.
"""
from PyQt6.QtWidgets import QFrame, QVBoxLayout, QLabel, QTextEdit
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QTextCursor, QFont


class DebugConsole(QFrame):
    """Scrollable debug console with colored, interleaved log entries."""

    def __init__(self, title="Debug"):
        super().__init__()
        self.setObjectName("DebugPanel")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(2)

        # Header
        header = QLabel(f"{title} >")
        header.setStyleSheet(
            "font-size: 12px; font-weight: bold; color: #BB86FC; "
            "border: none; background: transparent; padding: 0;"
        )
        layout.addWidget(header)

        # Log text area
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setFont(QFont("Consolas", 9))
        self.text_edit.setStyleSheet(
            "QTextEdit { background-color: rgba(5, 5, 10, 0.9); "
            "color: #888; border: none; border-radius: 4px; padding: 4px; }"
        )
        self.text_edit.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.text_edit.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        layout.addWidget(self.text_edit)

        self._entry_count = 0

    def append_entry(self, entry):
        """Append a LogEntry to the console."""
        color = entry.color
        prefix = entry.prefix
        text = entry.text

        # Truncate long packet lines for display
        if entry.source == "packet" and len(text) > 120:
            text = text[:117] + "..."

        html = f'<span style="color:{color}; font-size:9px;">{prefix} {text}</span>'
        self.text_edit.append(html)
        self._entry_count += 1

        # Auto-scroll to bottom
        cursor = self.text_edit.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self.text_edit.setTextCursor(cursor)

        # Trim old entries if too many (performance)
        if self._entry_count > 500:
            self._trim_old_entries()

    def append_entries(self, entries):
        """Append multiple LogEntry objects at once."""
        for entry in entries:
            self.append_entry(entry)

    def _trim_old_entries(self):
        """Remove old entries to keep the console performant."""
        cursor = self.text_edit.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.Start)
        for _ in range(100):
            cursor.movePosition(QTextCursor.MoveOperation.Down, QTextCursor.MoveMode.KeepAnchor)
        cursor.removeSelectedText()
        self._entry_count -= 100

    def clear_log(self):
        """Clear all log entries."""
        self.text_edit.clear()
        self._entry_count = 0

"""
Confirmation Dialog — Modal dialog for destructive/irreversible actions.
Supports standard and warning (high-consequence) styling.
"""
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
from PyQt6.QtCore import Qt


class ConfirmationDialog(QDialog):
    """Modal confirmation dialog for dangerous or irreversible actions."""

    def __init__(self, parent, title, message, warning=False):
        """
        Args:
            parent: Parent widget
            title: Dialog title
            message: Warning/instruction text
            warning: If True, uses red/danger styling (for high-consequence actions)
        """
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setModal(True)
        self.setMinimumWidth(360)
        self.setMaximumWidth(480)

        layout = QVBoxLayout(self)
        layout.setSpacing(16)
        layout.setContentsMargins(24, 20, 24, 20)

        # Warning icon + title
        icon_text = "⚠️" if warning else "❓"
        title_label = QLabel(f"{icon_text}  {title}")
        title_color = "#FF4444" if warning else "#BB86FC"
        title_label.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {title_color}; "
            f"border: none; background: transparent;"
        )
        layout.addWidget(title_label)

        # Message
        msg_label = QLabel(message)
        msg_label.setWordWrap(True)
        msg_label.setStyleSheet(
            "font-size: 13px; color: #c5c6c7; border: none; background: transparent;"
        )
        layout.addWidget(msg_label)

        # Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(12)

        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.setMinimumHeight(36)
        self.btn_cancel.setStyleSheet(
            "QPushButton { background-color: #2a2d35; color: #aaa; "
            "border-radius: 6px; padding: 8px 20px; font-weight: bold; "
            "border: 1px solid #555; }"
            "QPushButton:hover { background-color: #3a3d45; }"
        )
        self.btn_cancel.clicked.connect(self.reject)

        self.btn_confirm = QPushButton("Confirm")
        self.btn_confirm.setMinimumHeight(36)
        if warning:
            self.btn_confirm.setStyleSheet(
                "QPushButton { background-color: qlineargradient("
                "x1:0,y1:0,x2:1,y2:1,stop:0 #CC2200,stop:1 #881100); "
                "color: white; border-radius: 6px; padding: 8px 20px; "
                "font-weight: bold; border: 1px solid rgba(255,100,100,0.3); }"
                "QPushButton:hover { background-color: #EE3300; }"
            )
        else:
            self.btn_confirm.setStyleSheet(
                "QPushButton { background-color: qlineargradient("
                "x1:0,y1:0,x2:1,y2:1,stop:0 #8a2be2,stop:1 #4b0082); "
                "color: white; border-radius: 6px; padding: 8px 20px; "
                "font-weight: bold; border: 1px solid rgba(187,134,252,0.3); }"
                "QPushButton:hover { background-color: #9b4de3; }"
            )
        self.btn_confirm.clicked.connect(self.accept)

        btn_layout.addWidget(self.btn_cancel)
        btn_layout.addWidget(self.btn_confirm)
        layout.addLayout(btn_layout)

        # Dialog styling
        bg_color = "rgba(20, 10, 10, 0.95)" if warning else "rgba(15, 15, 25, 0.95)"
        border_color = "rgba(255, 80, 80, 0.4)" if warning else "rgba(187, 134, 252, 0.3)"
        self.setStyleSheet(
            f"QDialog {{ background-color: {bg_color}; "
            f"border: 1px solid {border_color}; border-radius: 12px; }}"
        )

    @staticmethod
    def confirm(parent, title, message, warning=False):
        """Show confirmation dialog and return True if confirmed."""
        dialog = ConfirmationDialog(parent, title, message, warning)
        return dialog.exec() == QDialog.DialogCode.Accepted

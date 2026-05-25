"""
Network Manager — TCP telemetry sharing server for multi-laptop support.
Primary laptop hosts server; other laptops connect as clients.
"""
from PyQt6.QtCore import QObject, QThread, pyqtSignal, QTimer
import socket
import json
import threading
from time import time


class ClientHandler(threading.Thread):
    """Handles a single connected client."""
    def __init__(self, conn, addr, server):
        super().__init__(daemon=True)
        self.conn = conn
        self.addr = addr
        self.server = server
        self.running = True
        self.name_str = f"{addr[0]}:{addr[1]}"

    def run(self):
        try:
            while self.running:
                # Keep connection alive, listen for client messages
                self.conn.settimeout(5.0)
                try:
                    data = self.conn.recv(1024)
                    if not data:
                        break
                except socket.timeout:
                    continue
                except Exception:
                    break
        finally:
            self.conn.close()
            self.server._remove_client(self)

    def send_telemetry(self, packet_json: str):
        try:
            msg = (packet_json + "\n").encode("utf-8")
            self.conn.sendall(msg)
        except Exception:
            self.running = False

    def stop(self):
        self.running = False
        try:
            self.conn.close()
        except Exception:
            pass


class TelemetryServer(QThread):
    """TCP server that broadcasts telemetry to connected clients."""

    client_connected = pyqtSignal(str)
    client_disconnected = pyqtSignal(str)
    server_error = pyqtSignal(str)
    client_count_changed = pyqtSignal(int)

    def __init__(self, port=5555):
        super().__init__()
        self.port = port
        self.running = False
        self.clients = []
        self.lock = threading.Lock()
        self.server_socket = None

    def run(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(("0.0.0.0", self.port))
            self.server_socket.listen(10)
            self.server_socket.settimeout(2.0)
            self.running = True

            while self.running:
                try:
                    conn, addr = self.server_socket.accept()
                    handler = ClientHandler(conn, addr, self)
                    with self.lock:
                        self.clients.append(handler)
                    handler.start()
                    self.client_connected.emit(handler.name_str)
                    self.client_count_changed.emit(len(self.clients))
                except socket.timeout:
                    continue
                except Exception:
                    if self.running:
                        continue
                    break
        except Exception as e:
            self.server_error.emit(str(e))
        finally:
            self._shutdown()

    def broadcast(self, packet: dict):
        """Send telemetry packet to all connected clients."""
        try:
            # Filter out non-serializable keys
            clean = {k: v for k, v in packet.items() if not k.startswith("_")}
            packet_json = json.dumps(clean)
        except Exception:
            return

        with self.lock:
            for client in list(self.clients):
                client.send_telemetry(packet_json)

    def _remove_client(self, handler):
        with self.lock:
            if handler in self.clients:
                self.clients.remove(handler)
        self.client_disconnected.emit(handler.name_str)
        self.client_count_changed.emit(len(self.clients))

    def get_client_names(self):
        with self.lock:
            return [c.name_str for c in self.clients]

    def get_client_count(self):
        with self.lock:
            return len(self.clients)

    def stop_server(self):
        self.running = False
        with self.lock:
            for c in self.clients:
                c.stop()
            self.clients.clear()
        self._shutdown()

    def _shutdown(self):
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.server_socket = None


class NetworkManager(QObject):
    """High-level network manager for telemetry sharing."""

    status_changed = pyqtSignal(str)
    clients_updated = pyqtSignal(list)

    def __init__(self, port=5555):
        super().__init__()
        self.server = TelemetryServer(port)
        self.server.client_connected.connect(self._on_client_change)
        self.server.client_disconnected.connect(self._on_client_change)
        self.server.server_error.connect(lambda e: self.status_changed.emit(f"Server error: {e}"))
        self.is_running = False

    def start_server(self):
        if not self.is_running:
            self.server.start()
            self.is_running = True
            self.status_changed.emit("Server started")

    def stop_server(self):
        if self.is_running:
            self.server.stop_server()
            self.is_running = False
            self.status_changed.emit("Server stopped")

    def broadcast(self, packet: dict):
        if self.is_running:
            self.server.broadcast(packet)

    def _on_client_change(self, name):
        self.clients_updated.emit(self.server.get_client_names())
        self.status_changed.emit(f"Clients: {self.server.get_client_count()}")

    def get_client_list(self):
        return self.server.get_client_names()

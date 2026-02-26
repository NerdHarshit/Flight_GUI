class CalculationsEngine:

    def __init__(self):
        self.last_timestamp = None

        self.last_ax = 0.0
        self.last_ay = 0.0
        self.last_az = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    def update(self, packet):

        timestamp = packet["timestamp"]  # milliseconds

        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            self.last_ax = packet["Ax"]
            self.last_ay = packet["Ay"]
            self.last_az = packet["Az"]
            return self._output()

        dt = (timestamp - self.last_timestamp) / 1000.0

        if dt <= 0 or dt > 1:
            return self._output()

        ax = packet["Ax"]
        ay = packet["Ay"]
        az = packet["Az"]

        # Trapezoidal integration
        self.vx += 0.5 * (self.last_ax + ax) * dt
        self.vy += 0.5 * (self.last_ay + ay) * dt
        self.vz += 0.5 * (self.last_az + az) * dt

        self.last_ax = ax
        self.last_ay = ay
        self.last_az = az
        self.last_timestamp = timestamp

        return self._output()

    def _output(self):
        return {
            "Vx": self.vx,
            "Vy": self.vy,
            "Vz": self.vz
        }
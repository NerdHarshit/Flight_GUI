class PacketParser:

    @staticmethod
    def parse(line: str):
        try:
            parts = line.split(",")

            if len(parts) != 14:
                return None

            return {
                "timestamp": float(parts[0]),  # milliseconds
                "Ax": float(parts[1]),
                "Ay": float(parts[2]),
                "Az": float(parts[3]),
                "H_baro": float(parts[4]),
                "Latitude": float(parts[5]),
                "Longitude": float(parts[6]),
                "H_gps": float(parts[7]),
                "Gx": float(parts[8]),
                "Gy": float(parts[9]),
                "Gz": float(parts[10]),
                "FSM": int(parts[11]),
                "Signal": int(parts[12]),
                "Counter": int(parts[13]),
            }

        except:
            return None
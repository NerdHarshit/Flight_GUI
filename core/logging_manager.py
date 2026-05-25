"""
Logging Manager — Handles enhanced CSV exports, metadata headers, and mission folder organization.
Replaces the old CSVExporter with better file handling.
"""
import csv
import os
from datetime import datetime


class LoggingManager:

    @staticmethod
    def _create_mission_folder():
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"exports/Mission_{timestamp}"
        os.makedirs(folder_name, exist_ok=True)
        return folder_name

    @staticmethod
    def _write_csv(filename, buffer_a, buffer_b, mission_folder=None):
        if not buffer_a.data and not buffer_b.data:
            print("No flight data to export")
            return None

        if mission_folder is None:
            os.makedirs("exports/csv", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = f"exports/csv/{filename}_{timestamp}.csv"
        else:
            filepath = os.path.join(mission_folder, f"{filename}.csv")

        # Collect all fields from the new packet structure
        headers = [
            "time_ms", 
            "A_ax", "A_ay", "A_az", "A_gx", "A_gy", "A_gz",
            "A_roll", "A_pitch", "A_yaw", "A_lat", "A_lon", "A_gps_alt",
            "A_baro_alt", "A_pressure", "A_temperature", "A_voltage", "A_current",
            "A_state", "A_apogee",
            "B_ax", "B_ay", "B_az", "B_baro_alt", "B_pressure", "B_temperature",
            "B_voltage", "B_state", "B_apogee",
            "system_flags", "packet_id", "crc", "signal_strength", "packet_loss_pct",
            
            # Legacy compatibility fields
            "timestamp", "Ax", "Ay", "Az", "H_baro", "Latitude", "Longitude",
            "H_gps", "Gx", "Gy", "Gz", "FSM", "Signal", "Counter"
        ]

        with open(filepath, "w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=headers, extrasaction='ignore')
            writer.writeheader()

            # We write the merged data (or just data from A since it contains both in the new structure)
            # A and B buffers contain the same packets, they just represent the history. 
            # We can use buffer_a as the primary source of truth for the CSV.
            for packet in buffer_a.data:
                writer.writerow(packet)

        print(f"File: {filepath} exported")
        return filepath

    @staticmethod
    def exportCheckPoint(telemetry_mgr):
        return LoggingManager._write_csv("checkpoint", telemetry_mgr.buffer_a, telemetry_mgr.buffer_b)

    @staticmethod
    def exportFullCSV(telemetry_mgr):
        mission_folder = LoggingManager._create_mission_folder()
        return LoggingManager._write_csv("full_flight", telemetry_mgr.buffer_a, telemetry_mgr.buffer_b, mission_folder)

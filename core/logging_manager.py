"""
Logging Manager — Handles CSV exports, metadata headers, and mission folder organization for dual-controller avionics and CanSat telemetry.
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
    def _write_avionics_csv(filename, buffer_a, mission_folder=None):
        if not buffer_a or not buffer_a.data:
            return None

        if mission_folder is None:
            os.makedirs("exports/csv", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = f"exports/csv/{filename}_avionics_{timestamp}.csv"
        else:
            filepath = os.path.join(mission_folder, f"{filename}_avionics.csv")

        headers = [
            "time_ms", "team_id", "A_baro_alt", "max_alt",
            "A_ax", "A_ay", "A_az", "accelMag",
            "A_gx", "A_gy", "A_gz",
            "A_temperature", "A_pressure",
            "A_pitch", "A_roll", "A_yaw",
            "A_voltage", "A_lat", "A_lon", "A_gps_alt",
            "gps_sats", "gps_stale", "A_state",
            "A_launched", "A_apogee", "A_separated", "A_landed",
            "B_alive", "B_ax", "B_ay", "B_az", "B_baro_alt", "B_state",
            "B_temperature", "B_pressure", "B_voltage", "B_pitch", "B_roll", "B_yaw",
            "battLow", "rssi", "snr", "packet_loss"
        ]

        # Dynamically append any unlisted non-private fields
        for packet in buffer_a.data:
            for k in packet.keys():
                if not k.startswith("_") and k not in headers and k != "flags":
                    headers.append(k)

        with open(filepath, "w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=headers, extrasaction='ignore')
            writer.writeheader()
            for packet in buffer_a.data:
                writer.writerow(packet)

        print(f"Avionics flight data exported to: {filepath}")
        return filepath

    @staticmethod
    def _write_cansat_csv(filename, buffer_cansat, mission_folder=None):
        if not buffer_cansat or not buffer_cansat.data:
            return None

        if mission_folder is None:
            os.makedirs("exports/csv", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = f"exports/csv/{filename}_cansat_{timestamp}.csv"
        else:
            filepath = os.path.join(mission_folder, f"{filename}_cansat.csv")

        headers = [
            "team_id", "time_ms", "state",
            "ax", "ay", "az",
            "h_raw", "h_filtered",
            "gx", "gy", "gz",
            "magX", "magY", "magZ",
            "lat", "lon", "alt", "gps_sats",
            "COG", "headingErr", "distToTarget",
            "batteryVoltage", "rssi", "snr", "packet_loss"
        ]

        # Dynamically append any unlisted non-private fields
        for packet in buffer_cansat.data:
            for k in packet.keys():
                if not k.startswith("_") and k not in headers:
                    headers.append(k)

        with open(filepath, "w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=headers, extrasaction='ignore')
            writer.writeheader()
            for packet in buffer_cansat.data:
                writer.writerow(packet)

        print(f"CanSat telemetry data exported to: {filepath}")
        return filepath

    @staticmethod
    def exportCheckPoint(telemetry_mgr):
        av_path = LoggingManager._write_avionics_csv("checkpoint", getattr(telemetry_mgr, 'buffer_a', None))
        can_path = LoggingManager._write_cansat_csv("checkpoint", getattr(telemetry_mgr, 'buffer_cansat', None))
        return av_path or can_path

    @staticmethod
    def exportFullCSV(telemetry_mgr):
        mission_folder = LoggingManager._create_mission_folder()
        av_path = LoggingManager._write_avionics_csv("full_flight", getattr(telemetry_mgr, 'buffer_a', None), mission_folder)
        can_path = LoggingManager._write_cansat_csv("full_flight", getattr(telemetry_mgr, 'buffer_cansat', None), mission_folder)
        return av_path or can_path

import csv
import os
from datetime import datetime

class CSVExporter:

    #this one will save @current instant csv
    @staticmethod
    def exportCheckPoint(buffer):

        if not buffer.data:
            print("no flight data to export")
            return None
        
        os.makedirs("exports/csv",exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"exports/csv/flight_{timestamp}.csv"

        headers = [
            "timestamp",
                "Ax",
                "Ay",
                "Az",
                "H_baro",
                "Latitude",
                "Longitude",
                "H_gps",
                "Gx",
                "Gy",
                "Gz",
                "FSM",
                "Signal",
                "Counter",
        ]

        with open(filename,"w",newline="")as file:
            writer = csv.DictWriter(file,fieldnames=headers)

            writer.writeheader()

            for packet in buffer.data:
                writer.writerow(packet)

        print("file:{filename} exported")
        return filename
    

    #this one saves full flight csv 
    @staticmethod
    def exportFullCSV(buffer):
        if not buffer.data:
            print("No data to export in buffer")
            return None
        
        os.makedirs("exports/Full_Flight_csvs",exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"exports/Full_Flight_csvs/Full_flight_{timestamp}.csv"

        headers = [
            "timestamp",
                "Ax",
                "Ay",
                "Az",
                "H_baro",
                "Latitude",
                "Longitude",
                "H_gps",
                "Gx",
                "Gy",
                "Gz",
                "FSM",
                "Signal",
                "Counter",
        ]

        with open(filename,"w",newline="")as file:
            writer = csv.DictWriter(file,fieldnames=headers)

            writer.writeheader()

            for packet in buffer.data:
                writer.writerow(packet)

        print(f"file:{filename} exported")
        return filename
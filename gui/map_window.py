from PyQt6.QtWidgets import QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl
import os

class MapWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live Map Tracking")
        self.setGeometry(200, 200, 800, 600)
        
        self.browser = QWebEngineView()
        self.setCentralWidget(self.browser)
        
        self._load_map_html()
        
    def _load_map_html(self):
        html_content = """<!DOCTYPE html>
<html>
<head>
    <title>Live Map</title>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <style>
        body { margin: 0; padding: 0; }
        #map { height: 100vh; width: 100vw; }
    </style>
</head>
<body>
    <div id="map"></div>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script>
        var map = L.map('map').setView([0, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        }).addTo(map);

        var rocketMarker = L.circleMarker([0, 0], {
            color: 'red',
            fillColor: '#f03',
            fillOpacity: 1,
            radius: 8
        }).addTo(map);

        var trajectory = L.polyline([], {color: 'purple', weight: 4}).addTo(map);

        var firstUpdate = true;
        function updatePosition(lat, lon, alt) {
            var newLatLng = new L.LatLng(lat, lon);
            rocketMarker.setLatLng(newLatLng);
            trajectory.addLatLng(newLatLng);
            if (firstUpdate) {
                map.setView(newLatLng, 16);
                firstUpdate = false;
            }
            rocketMarker.bindPopup("Alt: " + alt.toFixed(1) + "m");
        }
    </script>
</body>
</html>
"""
        self.browser.setHtml(html_content, QUrl("http://localhost"))

    def update_location(self, lat, lon, alt):
        # Execute JS to update map
        js = f"updatePosition({lat}, {lon}, {alt});"
        self.browser.page().runJavaScript(js)

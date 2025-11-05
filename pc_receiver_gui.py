#!/usr/bin/env python3
"""
PC Receiver GUI for HC-12 data from Raspberry Pi Mining Helmet
- Cross-platform (Windows/Linux)
- Requires: pyserial, PyQt5 (or PySide6 with small tweaks)

Install:
  Linux:  python3 -m pip install pyqt5 pyserial
  Windows: py -m pip install pyqt5 pyserial

Run:
  python pc_receiver_gui.py

Build portable exe (optional on Windows):
  pyinstaller --noconsole --onefile pc_receiver_gui.py

This GUI auto-discovers serial ports, connects at 9600 8N1 (default),
parses both structured text lines and JSON alert messages sent by main.py,
shows live status and logs, and supports CSV export.
"""

import json
import sys
import csv
import os
import time
import threading
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List, Dict

import serial
import serial.tools.list_ports

from PyQt5 import QtCore, QtGui, QtWidgets


DEFAULT_BAUD = 9600
LINE_ENCODING = "utf-8"


@dataclass
class ParsedReading:
    raw: str
    timestamp: float
    status: Optional[str] = None
    gps: Optional[Dict[str, float]] = None
    temperature: Optional[float] = None
    humidity: Optional[float] = None
    methane: Optional[float] = None
    co: Optional[float] = None
    lpg: Optional[float] = None
    smoke: Optional[float] = None
    air_quality: Optional[float] = None
    danger: Optional[str] = None
    reasons: Optional[List[str]] = field(default_factory=list)
    time_field: Optional[str] = None
    is_json_alert: bool = False
    alert_type: Optional[str] = None


class SerialReader(QtCore.QObject):
    line_received = QtCore.pyqtSignal(str)
    port_error = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal(str)
    disconnected = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._ser: Optional[serial.Serial] = None
        self._current_port = None
        self._baud = DEFAULT_BAUD
        self._synchronized = False

    def list_ports(self) -> List[str]:
        """List available serial ports."""
        ports = []
        try:
            for p in serial.tools.list_ports.comports():
                ports.append(p.device)
        except Exception:
            pass
        # Add common Linux ports if they exist
        for cand in ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyS0"]:
            if cand not in ports and os.path.exists(cand):
                ports.append(cand)
        return ports

    def connect(self, port: str, baud: int = DEFAULT_BAUD):
        """Start connection to serial port."""
        self._baud = baud
        self._current_port = port
        self._stop.clear()
        self._synchronized = False
        if self._thread and self._thread.is_alive():
            self.disconnect()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        """Stop serial connection."""
        self._stop.set()
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.disconnected.emit()

    def _run_loop(self):
        """Main serial reading loop."""
        try:
            self._ser = serial.Serial(self._current_port, self._baud, timeout=0.2)
            self.connected.emit(self._current_port)
        except Exception as e:
            self.port_error.emit(f"Open failed on {self._current_port}: {e}")
            return

        buffer = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(1024)
                if chunk:
                    buffer.extend(chunk)
                    while b"\n" in buffer:
                        line, _, rest = buffer.partition(b"\n")
                        buffer = bytearray(rest)
                        try:
                            text = line.decode(LINE_ENCODING, errors="replace").strip()
                        except Exception:
                            text = line.decode("latin1", errors="replace").strip()
                        
                        if not text:
                            continue
                        
                        # Synchronization logic
                        if not self._synchronized:
                            if self._is_valid_line_start(text):
                                self._synchronized = True
                                self.line_received.emit(text)
                        else:
                            self.line_received.emit(text)
                else:
                    time.sleep(0.01)
            except serial.SerialException as e:
                self.port_error.emit(f"Serial error: {e}")
                break
            except Exception as e:
                # Log but continue on transient errors
                time.sleep(0.1)

        # Cleanup
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self.disconnected.emit()

    def _is_valid_line_start(self, text: str) -> bool:
        """Check if line appears to be valid telemetry data."""
        t = text.strip()
        # CSV format: A,19.0760,72.8777,26.0,55.0,120,12,60,40,85
        # or D,... (DROWSY)
        if t and t[0] in ('A', 'D', 'a', 'd'):
            parts = t.split(',')
            if len(parts) >= 10:  # status + 9 data fields
                return True
        
        # JSON format
        if t.startswith('{'):
            return True
        
        # Key-value formats
        if any(t.startswith(prefix) for prefix in ("AWAKE", "DROWSY", "STATUS:", "ALERT:")):
            return True
        
        return False


class LogModel(QtCore.QAbstractTableModel):
    """Table model for displaying parsed telemetry data."""
    HEADERS = [
        "Received", "Type", "Status/Danger", "GPS", "Temp", "Hum",
        "CH4", "CO", "LPG", "Smoke", "AQI", "Reasons", "Raw"
    ]

    def __init__(self):
        super().__init__()
        self._rows: List[ParsedReading] = []

    def rowCount(self, parent=None):
        return len(self._rows)

    def columnCount(self, parent=None):
        return len(self.HEADERS)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid():
            return None
        r = self._rows[index.row()]
        c = index.column()
        
        if role == QtCore.Qt.DisplayRole:
            ts = datetime.fromtimestamp(r.timestamp).strftime("%H:%M:%S")
            if c == 0:
                return ts
            if c == 1:
                return "JSON-ALERT" if r.is_json_alert else "LINE"
            if c == 2:
                if r.is_json_alert:
                    return r.alert_type or "-"
                return f"{r.status or '-'} / {r.danger or '-'}"
            if c == 3:
                if r.gps:
                    return f"({r.gps.get('lat', 0):.6f},{r.gps.get('lon', 0):.6f})"
                return "-"
            if c == 4:
                return "-" if r.temperature is None else f"{r.temperature:.1f}"
            if c == 5:
                return "-" if r.humidity is None else f"{r.humidity:.1f}"
            if c == 6:
                return "-" if r.methane is None else f"{r.methane:.1f}"
            if c == 7:
                return "-" if r.co is None else f"{r.co:.1f}"
            if c == 8:
                return "-" if r.lpg is None else f"{r.lpg:.1f}"
            if c == 9:
                return "-" if r.smoke is None else f"{r.smoke:.1f}"
            if c == 10:
                return "-" if r.air_quality is None else f"{r.air_quality:.1f}"
            if c == 11:
                return ",".join(r.reasons) if r.reasons else "-"
            if c == 12:
                return r.raw
        
        if role == QtCore.Qt.ForegroundRole:
            if r.is_json_alert:
                return QtGui.QBrush(QtGui.QColor("#d35400"))
            if r.danger == "CRITICAL":
                return QtGui.QBrush(QtGui.QColor("#c0392b"))
            if r.danger == "WARNING":
                return QtGui.QBrush(QtGui.QColor("#f39c12"))
        
        return None

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if role == QtCore.Qt.DisplayRole and orientation == QtCore.Qt.Horizontal:
            return self.HEADERS[section]
        return None

    def add_row(self, r: ParsedReading):
        """Add a new row to the table."""
        self.beginInsertRows(QtCore.QModelIndex(), len(self._rows), len(self._rows))
        self._rows.append(r)
        self.endInsertRows()

    def to_csv(self, path: str):
        """Export all data to CSV file."""
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(self.HEADERS)
            for r in self._rows:
                row = [
                    datetime.fromtimestamp(r.timestamp).strftime("%Y-%m-%d %H:%M:%S"),
                    "JSON-ALERT" if r.is_json_alert else "LINE",
                    (r.alert_type or "-") if r.is_json_alert else f"{r.status or '-'} / {r.danger or '-'}",
                    f"({r.gps.get('lat', 0):.6f},{r.gps.get('lon', 0):.6f})" if r.gps else "-",
                    "-" if r.temperature is None else f"{r.temperature:.1f}",
                    "-" if r.humidity is None else f"{r.humidity:.1f}",
                    "-" if r.methane is None else f"{r.methane:.1f}",
                    "-" if r.co is None else f"{r.co:.1f}",
                    "-" if r.lpg is None else f"{r.lpg:.1f}",
                    "-" if r.smoke is None else f"{r.smoke:.1f}",
                    "-" if r.air_quality is None else f"{r.air_quality:.1f}",
                    ",".join(r.reasons) if r.reasons else "-",
                    r.raw,
                ]
                w.writerow(row)


def parse_line(text: str) -> ParsedReading:
    """Parse a line of telemetry data (CSV or JSON format)."""
    ts = time.time()

    def _to_float(val):
        try:
            if isinstance(val, (int, float)):
                return float(val)
            s = str(val).strip()
            # Strip common unit suffixes
            for suf in ["°C", "C", "%", "ppm", "PPM", "mg/m3", "mg/m^3"]:
                if s.upper().endswith(suf.upper()):
                    s = s[: -len(suf)].strip()
            return float(s)
        except Exception:
            return None

    # Try JSON first
    try:
        obj = json.loads(text)
        if isinstance(obj, dict):
            if obj.get("type") == "ALERT":
                r = ParsedReading(raw=text, timestamp=ts, is_json_alert=True)
                r.alert_type = obj.get("alert") or obj.get("message")
            else:
                r = ParsedReading(raw=text, timestamp=ts, is_json_alert=False)

            # GPS
            gps = obj.get("gps") or {}
            if isinstance(gps, dict) and ("lat" in gps or "lon" in gps):
                r.gps = {"lat": _to_float(gps.get("lat")) or 0.0, "lon": _to_float(gps.get("lon")) or 0.0}
            elif "lat" in obj and "lon" in obj:
                lt = _to_float(obj.get("lat"))
                ln = _to_float(obj.get("lon"))
                if lt is not None and ln is not None:
                    r.gps = {"lat": lt, "lon": ln}

            # Sensors
            sensors = obj.get("sensors") or obj
            if isinstance(sensors, dict):
                r.temperature = _to_float(sensors.get("temperature") or sensors.get("temp"))
                r.humidity = _to_float(sensors.get("humidity") or sensors.get("hum") or sensors.get("rh"))
                r.methane = _to_float(sensors.get("methane") or sensors.get("ch4"))
                r.co = _to_float(sensors.get("co") or sensors.get("carbon_monoxide"))
                r.lpg = _to_float(sensors.get("lpg"))
                r.smoke = _to_float(sensors.get("smoke") or sensors.get("mq2") or sensors.get("mq135"))
                aq = sensors.get("air_quality") or sensors.get("aqi") or sensors.get("airquality")
                r.air_quality = _to_float(aq)

            # Status/danger
            r.status = obj.get("status") or obj.get("state") or r.status
            if "danger" in obj:
                r.danger = obj.get("danger")
            reasons = obj.get("reasons")
            if isinstance(reasons, list):
                r.reasons = [str(x) for x in reasons]
            elif isinstance(reasons, str) and reasons and reasons.upper() != "NONE":
                r.reasons = [s.strip() for s in reasons.replace(";", ",").split(",") if s.strip()]

            return r
    except Exception:
        pass

    # Parse compact CSV format: A,19.0760,72.8777,26.0,55.0,120,12,60,40,85
    r = ParsedReading(raw=text, timestamp=ts)
    try:
        parts = [p.strip() for p in text.split(',')]
        if len(parts) >= 10:
            # Decode status from first character
            status_char = parts[0].upper()
            if status_char == 'A':
                r.status = "AWAKE"
            elif status_char == 'D':
                r.status = "DROWSY"
            else:
                r.status = parts[0]
            
            # Parse GPS
            lat = _to_float(parts[1])
            lon = _to_float(parts[2])
            if lat is not None and lon is not None:
                r.gps = {"lat": lat, "lon": lon}
            
            # Parse sensor values
            r.temperature = _to_float(parts[3])
            r.humidity = _to_float(parts[4])
            r.methane = _to_float(parts[5])
            r.co = _to_float(parts[6])
            r.lpg = _to_float(parts[7])
            r.smoke = _to_float(parts[8])
            r.air_quality = _to_float(parts[9])
            
            # Determine danger level based on sensor readings
            if r.methane and r.methane > 200:
                r.danger = "CRITICAL"
                r.reasons.append("High CH4")
            elif r.co and r.co > 50:
                r.danger = "CRITICAL"
                r.reasons.append("High CO")
            elif r.methane and r.methane > 150:
                r.danger = "WARNING"
                r.reasons.append("Elevated CH4")
            elif r.co and r.co > 30:
                r.danger = "WARNING"
                r.reasons.append("Elevated CO")
            elif r.status == "DROWSY":
                r.danger = "WARNING"
                r.reasons.append("Worker Drowsy")
            else:
                r.danger = "NORMAL"
            
            return r
    except Exception:
        pass

    # If CSV parsing failed, try key-value format
    try:
        tokens = [p.strip() for p in text.replace(',', ' ').split() if p.strip()]
        if tokens:
            first = tokens[0].upper()
            if first in ("AWAKE", "DROWSY"):
                r.status = first
            
            for token in tokens:
                if ':' in token or '=' in token:
                    sep = ':' if ':' in token else '='
                    key, val = token.split(sep, 1)
                    k = key.strip().upper()
                    v = val.strip()
                    
                    if k in ("TEMP", "TEMPERATURE"):
                        r.temperature = _to_float(v)
                    elif k in ("HUM", "HUMIDITY"):
                        r.humidity = _to_float(v)
                    elif k in ("METHANE", "CH4"):
                        r.methane = _to_float(v)
                    elif k == "CO":
                        r.co = _to_float(v)
                    elif k == "LPG":
                        r.lpg = _to_float(v)
                    elif k in ("SMOKE", "MQ2"):
                        r.smoke = _to_float(v)
                    elif k in ("AIR_QUALITY", "AQI"):
                        r.air_quality = _to_float(v)
    except Exception:
        pass

    return r


class MainWindow(QtWidgets.QMainWindow):
    """Main application window."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HC-12 Receiver - Mining Helmet Monitor")
        self.resize(1200, 700)

        self.reader = SerialReader()
        self.model = LogModel()

        self._build_ui()
        self._connect_signals()
        self.refresh_ports()

        # Port refresh timer
        self.port_timer = QtCore.QTimer(self)
        self.port_timer.setInterval(5000)
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start()

        self._message_count = 0

    def _build_ui(self):
        """Build the user interface."""
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # Control bar
        control_layout = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton("Refresh Ports")
        self.baud_edit = QtWidgets.QLineEdit(str(DEFAULT_BAUD))
        self.baud_edit.setFixedWidth(80)
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        
        control_layout.addWidget(QtWidgets.QLabel("Port:"))
        control_layout.addWidget(self.port_combo)
        control_layout.addWidget(self.refresh_btn)
        control_layout.addWidget(QtWidgets.QLabel("Baud:"))
        control_layout.addWidget(self.baud_edit)
        control_layout.addStretch(1)
        control_layout.addWidget(self.connect_btn)
        control_layout.addWidget(self.disconnect_btn)
        layout.addLayout(control_layout)

        # Status bar
        status_layout = QtWidgets.QHBoxLayout()
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #c0392b;")
        self.last_rx_label = QtWidgets.QLabel("Last RX: -")
        self.count_label = QtWidgets.QLabel("Messages: 0")
        self.export_btn = QtWidgets.QPushButton("Export CSV")
        
        status_layout.addWidget(self.status_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.last_rx_label)
        status_layout.addSpacing(20)
        status_layout.addWidget(self.count_label)
        status_layout.addStretch(1)
        status_layout.addWidget(self.export_btn)
        layout.addLayout(status_layout)

        # Table view
        self.table = QtWidgets.QTableView()
        self.table.setModel(self.model)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        layout.addWidget(self.table)

        # Quick gauges
        gauges = QtWidgets.QGroupBox("Latest Reading")
        g_layout = QtWidgets.QGridLayout(gauges)
        self.lbl_status = QtWidgets.QLabel("-")
        self.lbl_danger = QtWidgets.QLabel("-")
        self.lbl_gps = QtWidgets.QLabel("-")
        self.lbl_temp = QtWidgets.QLabel("-")
        self.lbl_hum = QtWidgets.QLabel("-")
        self.lbl_ch4 = QtWidgets.QLabel("-")
        self.lbl_co = QtWidgets.QLabel("-")
        self.lbl_lpg = QtWidgets.QLabel("-")
        self.lbl_smoke = QtWidgets.QLabel("-")
        self.lbl_aqi = QtWidgets.QLabel("-")
        
        labels = [
            ("Status", self.lbl_status), ("Danger", self.lbl_danger), ("GPS", self.lbl_gps),
            ("Temp", self.lbl_temp), ("Hum", self.lbl_hum), ("CH4", self.lbl_ch4),
            ("CO", self.lbl_co), ("LPG", self.lbl_lpg), ("Smoke", self.lbl_smoke), ("AQI", self.lbl_aqi),
        ]
        for i, (name, widget) in enumerate(labels):
            g_layout.addWidget(QtWidgets.QLabel(name+":"), i // 5, (i % 5) * 2)
            g_layout.addWidget(widget, i // 5, (i % 5) * 2 + 1)
        layout.addWidget(gauges)

    def _connect_signals(self):
        """Connect UI signals to slots."""
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.export_btn.clicked.connect(self.on_export)

        self.reader.line_received.connect(self.on_line)
        self.reader.port_error.connect(self.on_port_error)
        self.reader.connected.connect(self.on_connected)
        self.reader.disconnected.connect(self.on_disconnected)

    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        current = self.port_combo.currentText()
        ports = self.reader.list_ports()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        self.port_combo.blockSignals(False)
        
        if current and current in ports:
            idx = ports.index(current)
            self.port_combo.setCurrentIndex(idx)
        elif ports:
            self.port_combo.setCurrentIndex(0)

    def on_connect(self):
        """Handle connect button click."""
        port = self.port_combo.currentText()
        if not port:
            QtWidgets.QMessageBox.warning(self, "No Port", "No serial port selected.")
            return
        try:
            baud = int(self.baud_edit.text().strip())
        except Exception:
            baud = DEFAULT_BAUD
            self.baud_edit.setText(str(baud))
        
        self.reader.connect(port, baud)
        self.status_label.setText(f"Connecting to {port}...")
        self.status_label.setStyleSheet("color: #2980b9;")
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)

    def on_disconnect(self):
        """Handle disconnect button click."""
        self.reader.disconnect()

    def on_connected(self, port: str):
        """Handle successful connection."""
        self.status_label.setText(f"Connected: {port}")
        self.status_label.setStyleSheet("color: #27ae60;")

    def on_disconnected(self):
        """Handle disconnection."""
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: #c0392b;")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    def on_port_error(self, msg: str):
        """Handle port errors."""
        self.statusBar().showMessage(msg, 5000)
        self.status_label.setText(f"Error: {msg}")
        self.status_label.setStyleSheet("color: #c0392b;")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    def on_line(self, text: str):
        """Handle received line of data."""
        r = parse_line(text)
        self.model.add_row(r)
        self._message_count += 1
        self.count_label.setText(f"Messages: {self._message_count}")
        self.last_rx_label.setText(f"Last RX: {datetime.now().strftime('%H:%M:%S')}")
        
        # Auto-scroll
        idx = self.model.index(self.model.rowCount()-1, 0)
        self.table.scrollTo(idx, QtWidgets.QAbstractItemView.PositionAtBottom)
        
        self._update_gauges(r)

    def _update_gauges(self, r: ParsedReading):
        """Update the gauge display with latest values."""
        if not hasattr(self, "_last_values"):
            self._last_values = {}

        if r.is_json_alert:
            if r.alert_type:
                self.lbl_status.setText(r.alert_type)
                self.lbl_status.setStyleSheet("color:#d35400;font-weight:bold;")
        else:
            if r.status:
                self.lbl_status.setText(r.status)
                self.lbl_status.setStyleSheet("")
            if r.danger:
                self.lbl_danger.setText(r.danger)
                if r.danger == "CRITICAL":
                    self.lbl_danger.setStyleSheet("color:#c0392b;font-weight:bold;")
                elif r.danger == "WARNING":
                    self.lbl_danger.setStyleSheet("color:#f39c12;font-weight:bold;")
                else:
                    self.lbl_danger.setStyleSheet("")

        if r.gps:
            lat, lon = r.gps.get('lat', 0.0), r.gps.get('lon', 0.0)
            self.lbl_gps.setText(f"({lat:.6f},{lon:.6f})")

        if r.temperature is not None:
            self.lbl_temp.setText(f"{r.temperature:.1f} °C")
        if r.humidity is not None:
            self.lbl_hum.setText(f"{r.humidity:.1f} %")
        if r.methane is not None:
            self.lbl_ch4.setText(f"{r.methane:.1f} ppm")
        if r.co is not None:
            self.lbl_co.setText(f"{r.co:.1f} ppm")
        if r.lpg is not None:
            self.lbl_lpg.setText(f"{r.lpg:.1f} ppm")
        if r.smoke is not None:
            self.lbl_smoke.setText(f"{r.smoke:.1f} ppm")
        if r.air_quality is not None:
            self.lbl_aqi.setText(f"{r.air_quality:.1f}")

    def on_export(self):
        """Export data to CSV file."""
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, 
            "Export CSV", 
            f"hc12_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", 
            "CSV Files (*.csv)"
        )
        if not path:
            return
        try:
            self.model.to_csv(path)
            QtWidgets.QMessageBox.information(self, "Export", f"Saved to: {path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Export Failed", str(e))

    def closeEvent(self, event):
        """Handle window close event."""
        self.reader.disconnect()
        self.port_timer.stop()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
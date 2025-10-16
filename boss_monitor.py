import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import time
from datetime import datetime

# =============================
# CONFIGURATION
# =============================

HC12_PORT = '/dev/ttyUSB0'  # Change as needed for your laptop
HC12_BAUDRATE = 9600

# =============================
# GUI CLASS
# =============================

class BossMonitorGUI:
    def __init__(self, master):
        self.master = master
        master.title("Worker Safety Monitor")
        master.geometry("600x400")
        master.configure(bg="#222831")

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TLabel', background="#222831", foreground="#eeeeee", font=("Segoe UI", 12))
        style.configure('TFrame', background="#222831")
        style.configure('TButton', font=("Segoe UI", 11))

        self.status_var = tk.StringVar(value="Waiting for data...")
        self.gps_var = tk.StringVar(value="-")
        self.temp_var = tk.StringVar(value="-")
        self.hum_var = tk.StringVar(value="-")
        self.time_var = tk.StringVar(value="-")

        self.log = []

        main_frame = ttk.Frame(master)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)

        # Status
        ttk.Label(main_frame, text="Worker Status:").grid(row=0, column=0, sticky="w")
        self.status_label = ttk.Label(main_frame, textvariable=self.status_var, font=("Segoe UI", 14, "bold"))
        self.status_label.grid(row=0, column=1, sticky="w")

        # GPS
        ttk.Label(main_frame, text="GPS Location:").grid(row=1, column=0, sticky="w")
        self.gps_label = ttk.Label(main_frame, textvariable=self.gps_var)
        self.gps_label.grid(row=1, column=1, sticky="w")

        # Temperature
        ttk.Label(main_frame, text="Temperature (°C):").grid(row=2, column=0, sticky="w")
        self.temp_label = ttk.Label(main_frame, textvariable=self.temp_var)
        self.temp_label.grid(row=2, column=1, sticky="w")

        # Humidity
        ttk.Label(main_frame, text="Humidity (%):").grid(row=3, column=0, sticky="w")
        self.hum_label = ttk.Label(main_frame, textvariable=self.hum_var)
        self.hum_label.grid(row=3, column=1, sticky="w")

        # Time
        ttk.Label(main_frame, text="Last Alert Time:").grid(row=4, column=0, sticky="w")
        self.time_label = ttk.Label(main_frame, textvariable=self.time_var)
        self.time_label.grid(row=4, column=1, sticky="w")

        # Log
        ttk.Label(main_frame, text="Recent Alerts:").grid(row=5, column=0, sticky="nw", pady=(10,0))
        self.log_listbox = tk.Listbox(main_frame, height=8, width=60, bg="#393e46", fg="#eeeeee", font=("Segoe UI", 10))
        self.log_listbox.grid(row=5, column=1, sticky="w", pady=(10,0))

        # HC-12 Thread
        self.serial_thread = threading.Thread(target=self.read_hc12, daemon=True)
        self.serial_thread.start()

    def read_hc12(self):
        try:
            ser = serial.Serial(HC12_PORT, HC12_BAUDRATE, timeout=1)
            print(f"Connected to HC-12 on {HC12_PORT}")
        except Exception as e:
            self.status_var.set(f"HC-12 Error: {e}")
            return

        while True:
            try:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    self.process_message(line)
            except Exception as e:
                self.status_var.set(f"Read error: {e}")
                time.sleep(2)

    def process_message(self, msg):
        # Example message: DROWSY,GPS:(12.345678,98.765432),TEMP:25.1,HUM:60.2,TIME:12:34:56
        parts = msg.split(',')
        status = parts[0] if parts else "Unknown"
        gps = "-"
        temp = "-"
        hum = "-"
        t = datetime.now().strftime('%H:%M:%S')
        for p in parts:
            if p.startswith("GPS:"):
                gps = p[4:]
            elif p.startswith("TEMP:"):
                temp = p[5:]
            elif p.startswith("HUM:"):
                hum = p[4:]
            elif p.startswith("TIME:"):
                t = p[5:]
        self.status_var.set(status)
        self.gps_var.set(gps)
        self.temp_var.set(temp)
        self.hum_var.set(hum)
        self.time_var.set(t)
        log_entry = f"[{t}] {status} | {gps} | Temp: {temp} | Hum: {hum}"
        self.log.append(log_entry)
        self.log_listbox.insert(0, log_entry)
        if len(self.log) > 100:
            self.log_listbox.delete(100, tk.END)

# =============================
# MAIN
# =============================

def main():
    root = tk.Tk()
    app = BossMonitorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()

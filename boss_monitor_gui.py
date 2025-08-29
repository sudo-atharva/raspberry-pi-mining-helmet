import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime
import math

# =============================
# CONFIGURATION
# =============================

# Default port - will be auto-detected
HC12_PORT = None
HC12_BAUDRATE = 9600

# =============================
# MODERN GUI CLASS
# =============================

class ModernBossMonitorGUI:
    def __init__(self, master):
        self.master = master
        master.title("Worker Safety Monitor - Command Center")
        master.geometry("1200x800")
        master.configure(bg="#0f1419")
        master.minsize(1000, 700)
        
        # Configure grid weights for responsive layout
        master.grid_columnconfigure(0, weight=1)
        master.grid_rowconfigure(0, weight=1)
        
        # Modern color scheme
        self.colors = {
            'bg_dark': '#0f1419',
            'bg_card': '#1a1f2e',
            'bg_accent': '#2d3748',
            'primary': '#4299e1',
            'success': '#48bb78',
            'warning': '#ed8936',
            'danger': '#f56565',
            'text_primary': '#f7fafc',
            'text_secondary': '#a0aec0',
            'border': '#2d3748'
        }
        
        # Serial connection
        self.serial_connection = None
        self.is_connected = False
        
        # Configure modern styling
        self.setup_styles()
        
        # Variables
        self.status_var = tk.StringVar(value="Waiting for data...")
        self.gps_var = tk.StringVar(value="-")
        self.temp_var = tk.StringVar(value="-")
        self.hum_var = tk.StringVar(value="-")
        self.time_var = tk.StringVar(value="-")
        self.connection_status = tk.StringVar(value="Disconnected")
        self.connection_color = tk.StringVar(value=self.colors['danger'])
        
        self.log = []
        self.alert_count = 0
        
        # Create main container
        self.create_main_layout()
        
        # Auto-detect ports
        self.available_ports = self.get_available_ports()
        
        # Try to connect automatically
        self.auto_connect()
        
        # Update connection status periodically
        self.update_connection_status()

    def get_available_ports(self):
        """Get list of available serial ports"""
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            print(f"Available ports: {ports}")
            return ports
        except Exception as e:
            print(f"Error getting ports: {e}")
            return []

    def auto_connect(self):
        """Try to connect to HC-12 automatically"""
        if not self.available_ports:
            self.show_port_selection_dialog()
            return
            
        # Try common HC-12 ports
        common_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1', 'COM1', 'COM2', 'COM3']
        
        for port in common_ports:
            if port in self.available_ports:
                if self.try_connect(port):
                    return
        
        # If no common ports work, show port selection
        self.show_port_selection_dialog()

    def try_connect(self, port):
        """Try to connect to a specific port"""
        try:
            test_ser = serial.Serial(port, HC12_BAUDRATE, timeout=1)
            test_ser.close()
            HC12_PORT = port
            print(f"Auto-connected to {port}")
            return True
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
            return False

    def show_port_selection_dialog(self):
        """Show dialog to select port manually"""
        if not self.available_ports:
            messagebox.showerror("No Ports Available", 
                               "No serial ports found. Please connect your HC-12 module and restart the application.")
            return
            
        # Create port selection dialog
        dialog = tk.Toplevel(self.master)
        dialog.title("Select HC-12 Port")
        dialog.geometry("400x300")
        dialog.configure(bg=self.colors['bg_dark'])
        dialog.transient(self.master)
        dialog.grab_set()
        
        # Center the dialog
        dialog.geometry("+%d+%d" % (self.master.winfo_rootx() + 100, self.master.winfo_rooty() + 100))
        
        # Content
        tk.Label(dialog, 
                text="Select HC-12 Port:", 
                font=("Segoe UI", 14, "bold"),
                bg=self.colors['bg_dark'],
                fg=self.colors['text_primary']).pack(pady=20)
        
        # Port listbox
        port_frame = tk.Frame(dialog, bg=self.colors['bg_dark'])
        port_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        port_listbox = tk.Listbox(port_frame, 
                                 bg=self.colors['bg_card'],
                                 fg=self.colors['text_primary'],
                                 font=("Segoe UI", 12),
                                 selectmode=tk.SINGLE)
        port_listbox.pack(fill=tk.BOTH, expand=True)
        
        # Populate with available ports
        for port in self.available_ports:
            port_listbox.insert(tk.END, port)
        
        if self.available_ports:
            port_listbox.selection_set(0)
        
        # Buttons
        button_frame = tk.Frame(dialog, bg=self.colors['bg_dark'])
        button_frame.pack(fill=tk.X, padx=20, pady=20)
        
        def connect_selected():
            selection = port_listbox.curselection()
            if selection:
                selected_port = port_listbox.get(selection[0])
                global HC12_PORT
                HC12_PORT = selected_port
                print(f"Selected port: {HC12_PORT}")
                dialog.destroy()
                # Start connection thread
                self.start_connection_thread()
            else:
                messagebox.showwarning("No Port Selected", "Please select a port from the list.")
        
        def refresh_ports():
            dialog.destroy()
            self.available_ports = self.get_available_ports()
            self.show_port_selection_dialog()
        
        tk.Button(button_frame, 
                 text="Connect", 
                 command=connect_selected,
                 bg=self.colors['primary'],
                 fg=self.colors['text_primary'],
                 font=("Segoe UI", 11, "bold"),
                 relief='flat',
                 padx=20,
                 pady=10).pack(side=tk.LEFT, padx=(0, 10))
        
        tk.Button(button_frame, 
                 text="Refresh", 
                 command=refresh_ports,
                 bg=self.colors['bg_accent'],
                 fg=self.colors['text_primary'],
                 font=("Segoe UI", 11),
                 relief='flat',
                 padx=20,
                 pady=10).pack(side=tk.LEFT, padx=(0, 10))
        
        tk.Button(button_frame, 
                 text="Cancel", 
                 command=dialog.destroy,
                 bg=self.colors['bg_accent'],
                 fg=self.colors['text_primary'],
                 font=("Segoe UI", 11),
                 relief='flat',
                 padx=20,
                 pady=10).pack(side=tk.RIGHT)

    def start_connection_thread(self):
        """Start the HC-12 connection thread"""
        if not hasattr(self, 'serial_thread') or not self.serial_thread.is_alive():
            self.serial_thread = threading.Thread(target=self.read_hc12, daemon=True)
            self.serial_thread.start()

    def setup_styles(self):
        """Configure modern ttk styles"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Configure styles for different widgets
        style.configure('Card.TFrame', background=self.colors['bg_card'], relief='flat')
        style.configure('Header.TLabel', 
                      background=self.colors['bg_card'], 
                      foreground=self.colors['text_primary'], 
                      font=("Segoe UI", 16, "bold"))
        style.configure('SubHeader.TLabel', 
                      background=self.colors['bg_card'], 
                      foreground=self.colors['text_secondary'], 
                      font=("Segoe UI", 12))
        style.configure('Value.TLabel', 
                      background=self.colors['bg_card'], 
                      foreground=self.colors['text_primary'], 
                      font=("Segoe UI", 14, "bold"))
        style.configure('Status.TLabel', 
                      background=self.colors['bg_card'], 
                      foreground=self.colors['primary'], 
                      font=("Segoe UI", 18, "bold"))
        style.configure('Modern.TButton', 
                      font=("Segoe UI", 11, "bold"),
                      padding=(20, 10))

    def create_main_layout(self):
        """Create the main application layout"""
        # Main container
        main_container = tk.Frame(self.master, bg=self.colors['bg_dark'])
        main_container.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
        main_container.grid_columnconfigure(0, weight=1)
        main_container.grid_rowconfigure(1, weight=1)
        
        # Header section
        self.create_header(main_container)
        
        # Content section
        content_frame = tk.Frame(main_container, bg=self.colors['bg_dark'])
        content_frame.grid(row=1, column=0, sticky="nsew", pady=(20, 0))
        content_frame.grid_columnconfigure(0, weight=1)
        content_frame.grid_columnconfigure(1, weight=1)
        content_frame.grid_rowconfigure(0, weight=1)
        
        # Left panel - Status cards
        self.create_status_panel(content_frame)
        
        # Right panel - Log and controls
        self.create_log_panel(content_frame)

    def create_header(self, parent):
        """Create the header section with title and connection status"""
        header_frame = tk.Frame(parent, bg=self.colors['bg_card'], relief='flat', bd=0)
        header_frame.grid(row=0, column=0, sticky="ew", pady=(0, 20))
        header_frame.grid_columnconfigure(1, weight=1)
        
        # Title
        title_label = tk.Label(header_frame, 
                              text="🛡️ WORKER SAFETY MONITOR", 
                              font=("Segoe UI", 24, "bold"),
                              bg=self.colors['bg_card'],
                              fg=self.colors['text_primary'])
        title_label.grid(row=0, column=0, sticky="w", padx=20, pady=20)
        
        # Connection status
        status_frame = tk.Frame(header_frame, bg=self.colors['bg_card'])
        status_frame.grid(row=0, column=1, sticky="e", padx=20, pady=20)
        
        tk.Label(status_frame, 
                text="Connection:", 
                font=("Segoe UI", 12),
                bg=self.colors['bg_card'],
                fg=self.colors['text_secondary']).pack(side=tk.LEFT, padx=(0, 10))
        
        self.connection_indicator = tk.Label(status_frame, 
                                           text="●", 
                                           font=("Segoe UI", 16),
                                           bg=self.colors['bg_card'],
                                           fg=self.colors['danger'])
        self.connection_indicator.pack(side=tk.LEFT, padx=(0, 10))
        
        self.connection_label = tk.Label(status_frame, 
                                       textvariable=self.connection_status,
                                       font=("Segoe UI", 12, "bold"),
                                       bg=self.colors['bg_card'],
                                       fg=self.colors['text_primary'])
        self.connection_label.pack(side=tk.LEFT)
        
        # Port info
        self.port_label = tk.Label(status_frame, 
                                  text="", 
                                  font=("Segoe UI", 10),
                                  bg=self.colors['bg_card'],
                                  fg=self.colors['text_secondary'])
        self.port_label.pack(side=tk.LEFT, padx=(20, 0))

    def create_status_panel(self, parent):
        """Create the left panel with status cards"""
        status_frame = tk.Frame(parent, bg=self.colors['bg_dark'])
        status_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        status_frame.grid_columnconfigure(0, weight=1)
        
        # Main status card
        self.create_status_card(status_frame, "Worker Status", self.status_var, "Status.TLabel", row=0)
        
        # GPS card
        self.create_metric_card(status_frame, "📍 GPS Location", self.gps_var, row=1)
        
        # Temperature card
        self.create_metric_card(status_frame, "🌡️ Temperature", self.temp_var, row=2, unit="°C")
        
        # Humidity card
        self.create_metric_card(status_frame, "💧 Humidity", self.hum_var, row=3, unit="%")
        
        # Time card
        self.create_metric_card(status_frame, "⏰ Last Alert", self.time_var, row=4)
        
        # Alert counter card
        self.alert_counter_var = tk.StringVar(value="0")
        self.create_metric_card(status_frame, "🚨 Total Alerts", self.alert_counter_var, row=5)

    def create_status_card(self, parent, title, variable, style, row):
        """Create a main status card"""
        card = tk.Frame(parent, bg=self.colors['bg_card'], relief='flat', bd=0)
        card.grid(row=row, column=0, sticky="ew", pady=(0, 15))
        card.grid_columnconfigure(1, weight=1)
        
        # Title
        tk.Label(card, 
                text=title, 
                font=("Segoe UI", 14, "bold"),
                bg=self.colors['bg_card'],
                fg=self.colors['text_secondary']).grid(row=0, column=0, sticky="w", padx=20, pady=(20, 10))
        
        # Value
        value_label = tk.Label(card, 
                              textvariable=variable,
                              font=("Segoe UI", 18, "bold"),
                              bg=self.colors['bg_card'],
                              fg=self.colors['primary'])
        value_label.grid(row=0, column=1, sticky="e", padx=20, pady=(20, 10))
        
        # Bottom border
        tk.Frame(card, height=3, bg=self.colors['primary']).grid(row=1, column=0, columnspan=2, sticky="ew", padx=20)

    def create_metric_card(self, parent, title, variable, row, unit=""):
        """Create a metric card"""
        card = tk.Frame(parent, bg=self.colors['bg_card'], relief='flat', bd=0)
        card.grid(row=row, column=0, sticky="ew", pady=(0, 15))
        card.grid_columnconfigure(1, weight=1)
        
        # Title
        tk.Label(card, 
                text=title, 
                font=("Segoe UI", 12),
                bg=self.colors['bg_card'],
                fg=self.colors['text_secondary']).grid(row=0, column=0, sticky="w", padx=20, pady=(15, 5))
        
        # Value with unit
        value_text = f"{variable.get()}{unit}" if unit else variable.get()
        value_label = tk.Label(card, 
                              textvariable=variable,
                              font=("Segoe UI", 16, "bold"),
                              bg=self.colors['bg_card'],
                              fg=self.colors['text_primary'])
        value_label.grid(row=0, column=1, sticky="e", padx=20, pady=(15, 5))
        
        # Unit label
        if unit:
            unit_label = tk.Label(card, 
                                 text=unit, 
                                 font=("Segoe UI", 10),
                                 bg=self.colors['bg_card'],
                                 fg=self.colors['text_secondary'])
            unit_label.grid(row=1, column=1, sticky="e", padx=20, pady=(0, 15))

    def create_log_panel(self, parent):
        """Create the right panel with log and controls"""
        log_frame = tk.Frame(parent, bg=self.colors['bg_dark'])
        log_frame.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
        log_frame.grid_columnconfigure(0, weight=1)
        log_frame.grid_rowconfigure(1, weight=1)
        
        # Log header
        log_header = tk.Frame(log_frame, bg=self.colors['bg_card'], relief='flat', bd=0)
        log_header.grid(row=0, column=0, sticky="ew", pady=(0, 15))
        log_header.grid_columnconfigure(0, weight=1)
        
        tk.Label(log_header, 
                text="📋 Recent Alerts", 
                font=("Segoe UI", 16, "bold"),
                bg=self.colors['bg_card'],
                fg=self.colors['text_primary']).grid(row=0, column=0, sticky="w", padx=20, pady=20)
        
        # Clear button
        clear_btn = tk.Button(log_header, 
                             text="Clear Log", 
                             font=("Segoe UI", 10, "bold"),
                             bg=self.colors['bg_accent'],
                             fg=self.colors['text_primary'],
                             relief='flat',
                             bd=0,
                             padx=15,
                             pady=5,
                             command=self.clear_log)
        clear_btn.grid(row=0, column=1, sticky="e", padx=20, pady=20)
        
        # Log container
        log_container = tk.Frame(log_frame, bg=self.colors['bg_card'], relief='flat', bd=0)
        log_container.grid(row=1, column=0, sticky="nsew")
        log_container.grid_columnconfigure(0, weight=1)
        log_container.grid_rowconfigure(0, weight=1)
        
        # Scrollbar
        scrollbar = tk.Scrollbar(log_container)
        scrollbar.grid(row=0, column=1, sticky="ns")
        
        # Log listbox
        self.log_listbox = tk.Listbox(log_container, 
                                     height=20, 
                                     width=50,
                                     bg=self.colors['bg_card'],
                                     fg=self.colors['text_primary'],
                                     font=("Consolas", 10),
                                     relief='flat',
                                     bd=0,
                                     selectmode=tk.NONE,
                                     yscrollcommand=scrollbar.set)
        self.log_listbox.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)
        
        scrollbar.config(command=self.log_listbox.yview)
        
        # Configure listbox colors
        self.log_listbox.configure(selectbackground=self.colors['primary'],
                                  selectforeground=self.colors['text_primary'])

    def clear_log(self):
        """Clear the log display"""
        self.log_listbox.delete(0, tk.END)
        self.alert_count = 0
        self.alert_counter_var.set("0")

    def update_connection_status(self):
        """Update connection status display"""
        if self.is_connected:
            self.connection_indicator.configure(fg=self.colors['success'])
            self.connection_label.configure(fg=self.colors['success'])
            self.connection_status.set("Connected")
            if hasattr(self, 'port_label') and HC12_PORT:
                self.port_label.configure(text=f"Port: {HC12_PORT}")
        else:
            self.connection_indicator.configure(fg=self.colors['danger'])
            self.connection_label.configure(fg=self.colors['danger'])
            self.connection_status.set("Disconnected")
            if hasattr(self, 'port_label'):
                self.port_label.configure(text="")
        
        # Update every 2 seconds
        self.master.after(2000, self.update_connection_status)

    def read_hc12(self):
        """Read data from HC-12 module"""
        global HC12_PORT
        
        if not HC12_PORT:
            self.status_var.set("No port selected")
            return
            
        try:
            self.serial_connection = serial.Serial(HC12_PORT, HC12_BAUDRATE, timeout=1)
            print(f"Connected to HC-12 on {HC12_PORT}")
            self.is_connected = True
            self.connection_status.set("Connected")
        except Exception as e:
            error_msg = f"HC-12 Error: {e}"
            self.status_var.set(error_msg)
            self.is_connected = False
            self.connection_status.set("Disconnected")
            print(error_msg)
            return

        while self.is_connected:
            try:
                line = self.serial_connection.readline().decode('utf-8', errors='replace').strip()
                if line:
                    self.process_message(line)
            except Exception as e:
                error_msg = f"Read error: {e}"
                self.status_var.set(error_msg)
                self.is_connected = False
                self.connection_status.set("Disconnected")
                print(error_msg)
                break
        
        # Close connection if loop exits
        if self.serial_connection:
            try:
                self.serial_connection.close()
            except:
                pass

    def process_message(self, msg):
        """Process incoming messages and update GUI"""
        # Example message: DROWSY,GPS:(12.345678,98.765432),TEMP:25.1,HUM:60.2,TIME:12:34:56
        import csv
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
        
        # Update GUI variables
        self.status_var.set(status)
        self.gps_var.set(gps)
        self.temp_var.set(temp)
        self.hum_var.set(hum)
        self.time_var.set(t)
        
        # Update alert counter
        self.alert_count += 1
        self.alert_counter_var.set(str(self.alert_count))
        
        # Create log entry with timestamp and formatting
        log_entry = f"[{t}] {status} | GPS: {gps} | Temp: {temp}°C | Hum: {hum}%"
        self.log_listbox.insert(0, log_entry)
        
        # Limit log entries
        if self.log_listbox.size() > 100:
            self.log_listbox.delete(100, tk.END)
        
        # Write to CSV
        try:
            with open("alerts_log.csv", "a", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([t, status, gps, temp, hum])
        except Exception as e:
            print(f"CSV log error: {e}")

# =============================
# MAIN
# =============================

def main():
    root = tk.Tk()
    app = ModernBossMonitorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()

import tkinter as tk
from tkinter import ttk, messagebox
from serial.serialutil import SerialException
import serial
import serial.tools.list_ports
import re
import threading
import time
from datetime import datetime
import math
from collections import deque
import csv
try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    MATPLOTLIB_AVAILABLE = True
except Exception:
    MATPLOTLIB_AVAILABLE = False


def parse_hc12_message(msg: str) -> dict:
    """Parse a single HC-12 message line into a dict. This mirrors the sender format in main.py.

    Returns dict with keys: status,gps,temp,hum,methane,co,lpg,smoke,air_quality,danger,reasons,time
    """
    try:
        raw = msg.strip()
        if not raw:
            return {}

        # Extract leading status (before the first comma)
        m = re.match(r"\s*([^,]+)\s*,?\s*(.*)$", raw)
        if m:
            status = m.group(1).strip()
            rest = m.group(2) or ""
        else:
            status = raw
            rest = ""

        # Extract REASONS (may contain commas) and TIME if present
        reasons = None
        time_val = None
        reasons_match = re.search(r'REASONS:(.*),TIME:([0-9:]{5,8})', rest)
        if reasons_match:
            reasons = reasons_match.group(1).strip()
            time_val = reasons_match.group(2).strip()
            rest = rest[:reasons_match.start()] + rest[reasons_match.end():]
        else:
            time_match = re.search(r'TIME:([0-9:]{5,8})', rest)
            if time_match:
                time_val = time_match.group(1).strip()
                rest = rest.replace(time_match.group(0), '')

        # Extract simple KEY:VALUE pairs (values can be parentheses containing commas such as GPS)
        pairs = {}
        for pm in re.finditer(r"([A-Z_]+):(\([^\)]*\)|[^,]+)", rest):
            key = pm.group(1).strip()
            val = pm.group(2).strip()
            pairs[key] = val

        gps = pairs.get('GPS', '-')
        if gps.startswith('(') and gps.endswith(')'):
            gps = gps[1:-1]

        parsed = {
            'status': status,
            'gps': gps,
            'temp': pairs.get('TEMP', '-'),
            'hum': pairs.get('HUM', '-'),
            'methane': pairs.get('METHANE', '-'),
            'co': pairs.get('CO', '-'),
            'lpg': pairs.get('LPG', '-'),
            'smoke': pairs.get('SMOKE', '-'),
            'air_quality': pairs.get('AIR_QUALITY', '-'),
            'danger': pairs.get('DANGER', 'SAFE'),
            'reasons': reasons if reasons is not None else pairs.get('REASONS', '-'),
            'time': time_val if time_val is not None else pairs.get('TIME', datetime.now().strftime('%H:%M:%S'))
        }
        return parsed
    except Exception:
        return {}

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
        
        # Current state variables
        self._status = "Unknown"
        self._gps = "-"
        self._temp = "-"
        self._hum = "-"
        self._time = "-"
        self._methane = "-"
        self._co = "-"
        self._lpg = "-"
        self._smoke = "-"
        self._air_quality = "-"
        self._danger = "SAFE"
        self._reasons = "-"

        # GUI Variables
        self.status_var = tk.StringVar(value="Waiting for data...")
        self.gps_var = tk.StringVar(value="-")
        self.temp_var = tk.StringVar(value="-")
        self.hum_var = tk.StringVar(value="-")
        self.time_var = tk.StringVar(value="-")
        self.methane_var = tk.StringVar(value="-")
        self.co_var = tk.StringVar(value="-")
        self.lpg_var = tk.StringVar(value="-")
        self.smoke_var = tk.StringVar(value="-")
        self.air_quality_var = tk.StringVar(value="-")
        self.danger_var = tk.StringVar(value="SAFE")
        self.connection_status = tk.StringVar(value="Disconnected")
        self.connection_color = tk.StringVar(value=self.colors['danger'])
        
        # Variables for data history and plotting
        self.log = []
        self.alert_count = 0
        self.danger_alerts = []
        self.temp_history = deque(maxlen=300)
        self.hum_history = deque(maxlen=300)
        self.index_history = deque(maxlen=300)
        self.sample_index = 0
        
        # Matplotlib objects initialization
        if MATPLOTLIB_AVAILABLE:
            self.ax_temp = None
            self.ax_hum = None
            self.temp_line = None
            self.hum_line = None
            self.canvas = None
        
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
            # List all ports including details
            all_ports = list(serial.tools.list_ports.comports())
            ports = []
            for port in all_ports:
                # Print detailed port information for debugging
                print(f"Found port: {port.device}")
                print(f"   Description: {port.description}")
                print(f"   Hardware ID: {port.hwid}")
                ports.append(port.device)
            
            if not ports:
                print("No serial ports found. Please connect your HC-12 module.")
            else:
                print(f"Available ports: {ports}")
            return ports
        except Exception as e:
            print(f"Error getting ports: {e}")
            return []

    def check_port_permissions(self, port):
        """Check if we have permission to access the port"""
        import os
        import stat
        import platform
        
        try:
            # Check if port exists
            if not os.path.exists(port):
                print(f"Port {port} does not exist")
                return False
                
            # Get port stats
            st = os.stat(port)
            
            # Check if current user has read/write permission
            has_rw = bool(st.st_mode & stat.S_IRUSR) and bool(st.st_mode & stat.S_IWUSR)
            
            # Check group permissions on Arch Linux vs other distros
            import grp
            import pwd
            username = pwd.getpwuid(os.getuid())[0]
            groups = [g.gr_name for g in grp.getgrall() if username in g.gr_mem]
            
            # Check for both 'dialout' and 'uucp' groups (Arch uses uucp for serial)
            if platform.system() == 'Linux':
                if 'uucp' in groups or 'dialout' in groups:
                    print(f"User {username} is in required group (uucp or dialout)")
                    return True
            
            # Check for direct permissions
            if has_rw:
                print(f"User has direct read/write permission to {port}")
                return True
            
            # If we get here, show appropriate message
            if platform.system() == 'Linux':
                print(f"Permission denied for {port}. Add user to 'uucp' group: 'sudo usermod -a -G uucp $USER'")
            else:
                print(f"Permission denied for {port}. Please check your user permissions.")
            return False
            
        except Exception as e:
            print(f"Error checking permissions: {e}")
            return True  # Assume we can try anyway if there's an error checking permissions

    def auto_connect(self):
        """Try to connect to HC-12 automatically"""
        if not self.available_ports:
            self.show_port_selection_dialog()
            return
            
        # Try common HC-12 ports
        common_ports = [
            '/dev/ttyUSB0',  # Common USB-Serial adapter
            '/dev/ttyUSB1',
            '/dev/ttyACM0',  # Arduino-style USB
            '/dev/ttyACM1',
            '/dev/ttyS0',    # Hardware serial ports
            '/dev/ttyS1',
            'COM1', 'COM2', 'COM3'  # Windows ports
        ]
        
        # First try USB devices
        usb_ports = [port for port in self.available_ports if 'USB' in port.upper()]
        for port in usb_ports:
            print(f"Trying USB port: {port}")
            if self.check_port_permissions(port) and self.try_connect(port):
                self.start_connection_thread()
                return
                
        # Then try common ports
        for port in common_ports:
            if port in self.available_ports:
                print(f"Trying common port: {port}")
                if self.check_port_permissions(port) and self.try_connect(port):
                    self.start_connection_thread()
                    return
        
        # If no ports work, show port selection
        self.show_port_selection_dialog()

    def try_connect(self, port):
        """Try to connect to a specific port"""
        try:
            # First check if port exists
            if not port in [p.device for p in serial.tools.list_ports.comports()]:
                print(f"Port {port} not found in system")
                return False
                
            # Try to open the port
            test_ser = serial.Serial(port=port, baudrate=HC12_BAUDRATE, timeout=1)
            
            # Try to write and read from port to verify it's working
            try:
                test_ser.write(b'AT\r\n')
                time.sleep(0.1)
                response = test_ser.read(test_ser.in_waiting or 1)
                print(f"Port response: {response}")
            except Exception as e:
                print(f"Port write/read test failed: {e}")
                # Continue anyway as some HC-12 modules might not respond to AT commands
            
            test_ser.close()
            global HC12_PORT
            HC12_PORT = port
            print(f"Successfully connected to {port}")
            
            # Update status label preview
            try:
                self.port_label.configure(text=f"Port: {HC12_PORT}")
            except Exception:
                pass
            return True
            
        except SerialException as se:
            print(f"Serial error on {port}: {se}")
            if "Permission denied" in str(se):
                print("Permission denied. Try running with sudo or add user to dialout group")
            elif "Device or resource busy" in str(se):
                print("Port is busy. Make sure no other program is using it")
            return False
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
            return False

    def show_port_selection_dialog(self):
        """Show dialog to select port manually"""
        # Refresh available ports
        self.available_ports = self.get_available_ports()
        
        if not self.available_ports:
            response = messagebox.askretrycancel(
                "No Ports Available", 
                "No serial ports found. Please:\n\n" +
                "1. Check if the HC-12 module is connected\n" +
                "2. Check if you have permissions to access the port\n" +
                "3. Try disconnecting and reconnecting the module\n\n" +
                "Would you like to retry scanning for ports?")
            if response:
                self.show_port_selection_dialog()
            return
            
        # Create port selection dialog
        dialog = tk.Toplevel(self.master)
        dialog.title("Select HC-12 Port")
        dialog.geometry("500x400")
        dialog.configure(bg=self.colors['bg_dark'])
        dialog.transient(self.master)
        dialog.grab_set()
        
        # Center the dialog
        dialog.geometry("+%d+%d" % (
            self.master.winfo_rootx() + (self.master.winfo_width() - 500) // 2,
            self.master.winfo_rooty() + (self.master.winfo_height() - 400) // 2
        ))
        
        # Instructions
        tk.Label(dialog, 
                text="Select HC-12 Module Port", 
                font=("Segoe UI", 16, "bold"),
                bg=self.colors['bg_dark'],
                fg=self.colors['text_primary']).pack(pady=(20, 5))
                
        tk.Label(dialog,
                text="The HC-12 module typically appears as a USB-to-Serial device",
                font=("Segoe UI", 10),
                bg=self.colors['bg_dark'],
                fg=self.colors['text_secondary']).pack(pady=(0, 20))
        
        # Port listbox with scrollbar
        port_frame = tk.Frame(dialog, bg=self.colors['bg_dark'])
        port_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        # Add scrollbar
        scrollbar = tk.Scrollbar(port_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Port listbox with port details
        port_listbox = tk.Listbox(port_frame, 
                                 bg=self.colors['bg_card'],
                                 fg=self.colors['text_primary'],
                                 font=("Segoe UI", 11),
                                 selectmode=tk.SINGLE,
                                 yscrollcommand=scrollbar.set)
        port_listbox.pack(fill=tk.BOTH, expand=True)
        
        scrollbar.config(command=port_listbox.yview)
        
        # Populate with available ports and their descriptions
        all_ports = list(serial.tools.list_ports.comports())
        for port in all_ports:
            description = f"{port.device} - {port.description}"
            if "USB" in port.description.upper():
                description += " (Recommended)"
            port_listbox.insert(tk.END, description)
        
        if all_ports:
            port_listbox.selection_set(0)
            
        # Status label
        self.port_status_label = tk.Label(dialog,
                                        text="",
                                        font=("Segoe UI", 10),
                                        bg=self.colors['bg_dark'],
                                        fg=self.colors['text_secondary'])
        self.port_status_label.pack(pady=5)
        
        # Buttons
        button_frame = tk.Frame(dialog, bg=self.colors['bg_dark'])
        button_frame.pack(fill=tk.X, padx=20, pady=20)
        
        def connect_selected():
            selection = port_listbox.curselection()
            if selection:
                # Extract port name from the description
                selected_text = port_listbox.get(selection[0])
                selected_port = selected_text.split(" - ")[0].strip()
                
                # Try to connect
                if self.try_connect(selected_port):
                    global HC12_PORT
                    HC12_PORT = selected_port
                    print(f"Successfully connected to {HC12_PORT}")
                    dialog.destroy()
                    # Start connection thread
                    self.start_connection_thread()
                else:
                    self.port_status_label.configure(
                        text=f"Failed to connect to {selected_port}. Please try another port.",
                        fg=self.colors['danger'])
            else:
                self.port_status_label.configure(
                    text="Please select a port from the list.",
                    fg=self.colors['warning'])
        
        def refresh_ports():
            self.port_status_label.configure(text="Scanning for ports...", fg=self.colors['primary'])
            dialog.update()
            dialog.after(100, lambda: dialog.destroy() and self.show_port_selection_dialog())
        
        # Connect button
        tk.Button(button_frame, 
                 text="Connect", 
                 command=connect_selected,
                 bg=self.colors['primary'],
                 fg=self.colors['text_primary'],
                 font=("Segoe UI", 11, "bold"),
                 relief='flat',
                 padx=30,
                 pady=10).pack(side=tk.LEFT, padx=(0, 10))
        
        # Refresh button
        tk.Button(button_frame, 
                 text="Refresh Ports", 
                 command=refresh_ports,
                 bg=self.colors['bg_accent'],
                 fg=self.colors['text_primary'],
                 font=("Segoe UI", 11),
                 relief='flat',
                 padx=20,
                 pady=10).pack(side=tk.LEFT, padx=(0, 10))
        
        # Cancel button
        tk.Button(button_frame, 
                 text="Cancel", 
                 command=dialog.destroy,
                 bg=self.colors['bg_accent'],
                 fg=self.colors['text_primary'],
                 font=("Segoe UI", 11),
                 relief='flat',
                 padx=20,
                 pady=10).pack(side=tk.RIGHT)
        
        # Help text
        help_text = ("Note: If you don't see your device, try:\n" +
                    "1. Unplugging and replugging the HC-12 module\n" +
                    "2. Clicking 'Refresh Ports'\n" +
                    "3. Check system permissions for USB devices")
        
        tk.Label(dialog,
                text=help_text,
                font=("Segoe UI", 9),
                bg=self.colors['bg_dark'],
                fg=self.colors['text_secondary'],
                justify=tk.LEFT).pack(padx=20, pady=(0, 10))

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
        
        # Charts below the log (if matplotlib available)
        if MATPLOTLIB_AVAILABLE:
            self.create_charts(content_frame)

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
        
        # Gas sensor cards
        self.create_metric_card(status_frame, "⛽ Methane (ppm)", self.methane_var, row=6)
        self.create_metric_card(status_frame, "💨 CO (ppm)", self.co_var, row=7)
        self.create_metric_card(status_frame, "🔥 LPG (ppm)", self.lpg_var, row=8)
        self.create_metric_card(status_frame, "💭 Smoke (ppm)", self.smoke_var, row=9)
        
        # Air quality and danger cards
        self.create_metric_card(status_frame, "🌬️ Air Quality", self.air_quality_var, row=10, unit="%")
        self.create_danger_card(status_frame, "⚠️ Danger Level", self.danger_var, row=11)

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
        if title == "Worker Status":
            self.status_value_label = value_label
        
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

    def create_danger_card(self, parent, title, variable, row):
        """Create a danger level card with color coding"""
        card = tk.Frame(parent, bg=self.colors['bg_card'], relief='flat', bd=0)
        card.grid(row=row, column=0, sticky="ew", pady=(0, 15))
        card.grid_columnconfigure(1, weight=1)
        
        # Title
        tk.Label(card, 
                text=title, 
                font=("Segoe UI", 12),
                bg=self.colors['bg_card'],
                fg=self.colors['text_secondary']).grid(row=0, column=0, sticky="w", padx=20, pady=(15, 5))
        
        # Value with dynamic color based on danger level
        self.danger_value_label = tk.Label(card, 
                                          textvariable=variable,
                                          font=("Segoe UI", 16, "bold"),
                                          bg=self.colors['bg_card'],
                                          fg=self.colors['success'])
        self.danger_value_label.grid(row=0, column=1, sticky="e", padx=20, pady=(15, 5))
        
        # Update danger color based on value
        self.update_danger_color()
    
    def update_danger_color(self):
        """Update danger level color based on current value"""
        try:
            if hasattr(self, 'danger_value_label'):
                danger_level = self.danger_var.get()
                if danger_level == 'CRITICAL':
                    self.danger_value_label.configure(fg=self.colors['danger'])
                elif danger_level == 'WARNING':
                    self.danger_value_label.configure(fg=self.colors['warning'])
                else:
                    self.danger_value_label.configure(fg=self.colors['success'])
        except Exception:
            pass

    def create_log_panel(self, parent):
        """Create the right panel with log and controls"""
        log_frame = tk.Frame(parent, bg=self.colors['bg_dark'])
        log_frame.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
        log_frame.grid_columnconfigure(0, weight=1)
        log_frame.grid_rowconfigure(1, weight=1)
        log_frame.grid_rowconfigure(2, weight=0)
        
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

        self.log_frame = log_frame

    def create_charts(self, parent):
        """Create matplotlib charts for temperature and humidity"""
        charts_container = tk.Frame(parent, bg=self.colors['bg_dark'])
        charts_container.grid(row=1, column=1, sticky="ew", padx=(10, 0))
        
        fig = Figure(figsize=(5.5, 2.5), dpi=100)
        self.ax_temp = fig.add_subplot(211)
        self.ax_hum = fig.add_subplot(212)
        
        self.ax_temp.set_title('Temperature (°C)', color=self.colors['text_primary'], fontsize=10)
        self.ax_hum.set_title('Humidity (%)', color=self.colors['text_primary'], fontsize=10)
        
        for ax in (self.ax_temp, self.ax_hum):
            ax.grid(True, alpha=0.2)
            ax.tick_params(colors=self.colors['text_secondary'])
            ax.spines['bottom'].set_color(self.colors['text_secondary'])
            ax.spines['top'].set_color(self.colors['text_secondary'])
            ax.spines['left'].set_color(self.colors['text_secondary'])
            ax.spines['right'].set_color(self.colors['text_secondary'])
        
        self.temp_line, = self.ax_temp.plot([], [], color='#f6ad55')
        self.hum_line, = self.ax_hum.plot([], [], color='#63b3ed')
        
        self.canvas = FigureCanvasTkAgg(fig, master=charts_container)
        self.canvas.get_tk_widget().pack(fill=tk.X, expand=False)

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
            # Try to open serial port
            self.serial_connection = serial.Serial(
                port=HC12_PORT,
                baudrate=HC12_BAUDRATE,
                timeout=1,
                write_timeout=1
            )
            
            # Set DTR and RTS to prepare for communication
            self.serial_connection.dtr = False
            self.serial_connection.rts = False
            time.sleep(0.1)  # Wait for signals to settle
            
            print(f"Connected to HC-12 on {HC12_PORT}")
            self.is_connected = True
            self.connection_status.set("Connected")
            
        except SerialException as se:
            error_msg = f"HC-12 Error: {se}"
            if "Permission denied" in str(se):
                error_msg += "\nTry running with sudo or add user to dialout group"
            elif "Device or resource busy" in str(se):
                error_msg += "\nPort is busy. Make sure no other program is using it"
            self.status_var.set(error_msg)
            self.is_connected = False
            self.connection_status.set("Disconnected")
            print(error_msg)
            return
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
        """Parse incoming message (from main.py) and schedule UI update on the Tk thread.

        Expected message format (one line):
        STATUS,GPS:(lat,lon),TEMP:xx.x,HUM:xx.x,METHANE:xx.x,CO:xx.x,LPG:xx.x,SMOKE:xx.x,AIR_QUALITY:xx.x,DANGER:LEVEL,REASONS:xxx,TIME:HH:MM:SS
        """
        try:
            raw = msg.strip()
            if not raw:
                return

            # Extract leading status (before the first comma)
            m = re.match(r"\s*([^,]+)\s*,?\s*(.*)$", raw)
            if m:
                status = m.group(1).strip()
                rest = m.group(2) or ""
            else:
                status = raw
                rest = ""

            # First, extract REASONS (may contain commas) and TIME if present, so REASONS doesn't get split by commas
            reasons = None
            time_val = None
            reasons_match = re.search(r'REASONS:(.*),TIME:([0-9:]{5,8})', rest)
            if reasons_match:
                reasons = reasons_match.group(1).strip()
                time_val = reasons_match.group(2).strip()
                # remove the matched REASONS...,TIME:... from rest so subsequent parsing isn't confused by commas
                rest = rest[:reasons_match.start()] + rest[reasons_match.end():]
            else:
                # fallback: look for TIME alone
                time_match = re.search(r'TIME:([0-9:]{5,8})', rest)
                if time_match:
                    time_val = time_match.group(1).strip()
                    rest = rest.replace(time_match.group(0), '')

            # Find KEY:VALUE pairs. Value may be in parentheses (e.g. GPS:(lat,lon)) which can contain commas.
            pairs = dict()
            for pm in re.finditer(r"([A-Z_]+):(\([^\)]*\)|[^,]+)", rest):
                key = pm.group(1).strip()
                val = pm.group(2).strip()
                pairs[key] = val

            # Normalize and strip parentheses for GPS
            gps = pairs.get('GPS', '-')
            if gps.startswith('(') and gps.endswith(')'):
                gps = gps[1:-1]

            parsed = {
                'status': status,
                'gps': gps,
                'temp': pairs.get('TEMP', '-'),
                'hum': pairs.get('HUM', '-'),
                'methane': pairs.get('METHANE', '-'),
                'co': pairs.get('CO', '-'),
                'lpg': pairs.get('LPG', '-'),
                'smoke': pairs.get('SMOKE', '-'),
                'air_quality': pairs.get('AIR_QUALITY', '-'),
                'danger': pairs.get('DANGER', 'SAFE'),
                'reasons': reasons if reasons is not None else pairs.get('REASONS', '-'),
                'time': time_val if time_val is not None else pairs.get('TIME', datetime.now().strftime('%H:%M:%S'))
            }
            # Debug: print parsed message (helpful when data not showing correctly)
            print(f"Parsed HC-12 message: {parsed}")

            # Schedule UI update on main thread
            try:
                self.master.after(0, lambda p=parsed: self._update_ui_with_parsed(p))
            except Exception:
                # Fallback: if master not available, update inline
                self._update_ui_with_parsed(parsed)

        except Exception as e:
            print(f"process_message parse error: {e} -- raw: {repr(msg)}")

    def _update_ui_with_parsed(self, parsed):
        """Apply parsed data to GUI and internal state. Runs on Tk main thread."""
        # Update internal state
        self._status = parsed.get('status', self._status)
        self._gps = parsed.get('gps', self._gps)
        self._temp = parsed.get('temp', self._temp)
        self._hum = parsed.get('hum', self._hum)
        self._methane = parsed.get('methane', self._methane)
        self._co = parsed.get('co', self._co)
        self._lpg = parsed.get('lpg', self._lpg)
        self._smoke = parsed.get('smoke', self._smoke)
        self._air_quality = parsed.get('air_quality', self._air_quality)
        self._danger = parsed.get('danger', self._danger)
        self._reasons = parsed.get('reasons', self._reasons)
        self._time = parsed.get('time', self._time)

        # Update GUI variables
        self.status_var.set(self._status)
        self.gps_var.set(self._gps)
        self.temp_var.set(self._temp)
        self.hum_var.set(self._hum)
        self.methane_var.set(self._methane)
        self.co_var.set(self._co)
        self.lpg_var.set(self._lpg)
        self.smoke_var.set(self._smoke)
        self.air_quality_var.set(self._air_quality)
        self.danger_var.set(self._danger)
        self.time_var.set(self._time)

        # Update danger/status colors
        self.update_danger_color()
        if hasattr(self, 'status_value_label'):
            s = (self._status or "").upper()
            if 'DROWSY' in s or 'NO_FACE' in s or 'ALERT' in s:
                self.status_value_label.configure(fg=self.colors['danger'])
            elif 'AWAKE' in s or 'FACE' in s:
                self.status_value_label.configure(fg=self.colors['success'])
            else:
                self.status_value_label.configure(fg=self.colors['primary'])

        # Update charts data (if matplotlib available and initialized)
        if MATPLOTLIB_AVAILABLE and self.temp_line is not None and self.hum_line is not None:
            try:
                temp_f = float(str(self._temp).replace('°C','').strip()) if self._temp not in ('-', '') else None
            except Exception:
                temp_f = None
            try:
                hum_f = float(str(self._hum).replace('%','').strip()) if self._hum not in ('-', '') else None
            except Exception:
                hum_f = None

            self.sample_index += 1
            self.index_history.append(self.sample_index)
            self.temp_history.append(temp_f if temp_f is not None else float('nan'))
            self.hum_history.append(hum_f if hum_f is not None else float('nan'))

            x = list(self.index_history)
            self.temp_line.set_data(x, list(self.temp_history))
            self.hum_line.set_data(x, list(self.hum_history))
            if x:
                xmin = max(0, x[-1] - 300)
                xmax = x[-1] + 1
                self.ax_temp.set_xlim(xmin, xmax)
                self.ax_hum.set_xlim(xmin, xmax)
            self.ax_temp.relim(); self.ax_temp.autoscale_view(scaley=True)
            self.ax_hum.relim(); self.ax_hum.autoscale_view(scaley=True)
            try:
                self.canvas.draw_idle()
            except Exception:
                pass
    
    def create_danger_alert(self, danger_level, reasons, methane, co, lpg, smoke):
        """Create and display danger alerts"""
        try:
            # Create alert message
            alert_msg = f"🚨 {danger_level} ALERT: "
            if 'HIGH_METHANE' in reasons:
                alert_msg += f"Methane: {methane} ppm (EXPLOSIVE RISK!) "
            if 'HIGH_CO' in reasons:
                alert_msg += f"CO: {co} ppm (POISONOUS!) "
            if 'HIGH_LPG' in reasons:
                alert_msg += f"LPG: {lpg} ppm (EXPLOSIVE RISK!) "
            if 'SMOKE_DETECTED' in reasons:
                alert_msg += f"Smoke: {smoke} ppm (FIRE RISK!) "
            
            # Add to danger alerts list
            timestamp = datetime.now().strftime('%H:%M:%S')
            self.danger_alerts.append(f"[{timestamp}] {alert_msg}")
            
            # Keep only last 50 alerts
            if len(self.danger_alerts) > 50:
                self.danger_alerts.pop(0)
            
            # Show popup for critical alerts
            if danger_level == "CRITICAL":
                self.show_critical_alert_popup(alert_msg)
            
            # Log to console
            print(f"⚠️ {alert_msg}")
            
        except Exception as e:
            print(f"Error creating danger alert: {e}")
    
    def show_critical_alert_popup(self, alert_msg):
        """Show critical alert popup with sound"""
        try:
            # Create popup window
            popup = tk.Toplevel(self.master)
            popup.title("🚨 CRITICAL ALERT")
            popup.geometry("500x300")
            popup.configure(bg=self.colors['danger'])
            popup.attributes('-topmost', True)
            
            # Center popup
            popup.geometry("+%d+%d" % (self.master.winfo_rootx() + 200, self.master.winfo_rooty() + 200))
            
            # Alert content
            tk.Label(popup, 
                    text="🚨 CRITICAL ALERT 🚨", 
                    font=("Segoe UI", 20, "bold"),
                    bg=self.colors['danger'],
                    fg="white").pack(pady=20)
            
            tk.Label(popup, 
                    text=alert_msg, 
                    font=("Segoe UI", 14),
                    bg=self.colors['danger'],
                    fg="white",
                    wraplength=450).pack(pady=20)
            
            tk.Label(popup, 
                    text="IMMEDIATE ACTION REQUIRED!", 
                    font=("Segoe UI", 16, "bold"),
                    bg=self.colors['danger'],
                    fg="yellow").pack(pady=20)
            
            # Acknowledge button
            tk.Button(popup, 
                     text="ACKNOWLEDGE", 
                     command=popup.destroy,
                     font=("Segoe UI", 14, "bold"),
                     bg="white",
                     fg=self.colors['danger'],
                     relief='flat',
                     padx=30,
                     pady=10).pack(pady=20)
            
            # Auto-close after 30 seconds
            popup.after(30000, popup.destroy)
            
            # Update alert counter
            self.alert_count += 1
            self.alert_counter_var.set(str(self.alert_count))
            
        except Exception as e:
            print(f"Error showing critical alert popup: {e}")
        finally:
            self.update_status_and_charts()

    def update_status_and_charts(self):
        """Update status colors and chart data"""
        # Update status color using current internal state
        try:
            if hasattr(self, 'status_value_label'):
                s = (self._status or "").upper()
                if 'DROWSY' in s or 'NO_FACE' in s or 'ALERT' in s:
                    self.status_value_label.configure(fg=self.colors['danger'])
                elif 'AWAKE' in s or 'FACE' in s:
                    self.status_value_label.configure(fg=self.colors['success'])
                else:
                    self.status_value_label.configure(fg=self.colors['primary'])
        except Exception:
            pass

        # Update charts data using current temp/hum
        if MATPLOTLIB_AVAILABLE and self.temp_line is not None and self.hum_line is not None:
            try:
                temp_f = float(str(self._temp).replace('°C','').strip()) if self._temp not in ('-', '') else None
            except Exception:
                temp_f = None
            try:
                hum_f = float(str(self._hum).replace('%','').strip()) if self._hum not in ('-', '') else None
            except Exception:
                hum_f = None

            self.sample_index += 1
            self.index_history.append(self.sample_index)
            self.temp_history.append(temp_f if temp_f is not None else float('nan'))
            self.hum_history.append(hum_f if hum_f is not None else float('nan'))

            try:
                x = list(self.index_history)
                self.temp_line.set_data(x, list(self.temp_history))
                self.hum_line.set_data(x, list(self.hum_history))
                if x:
                    xmin = max(0, x[-1] - 300)
                    xmax = x[-1] + 1
                    self.ax_temp.set_xlim(xmin, xmax)
                    self.ax_hum.set_xlim(xmin, xmax)
                self.ax_temp.relim()
                self.ax_temp.autoscale_view(scaley=True)
                self.ax_hum.relim()
                self.ax_hum.autoscale_view(scaley=True)
                try:
                    self.canvas.draw_idle()
                except Exception:
                    pass
            except Exception:
                pass

            # Create log entry with current internal state and write to GUI and CSV
            try:
                log_entry = f"[{self._time}] {self._status} | GPS: {self._gps} | Temp: {self._temp} | Hum: {self._hum}"
                self.log_listbox.insert(0, log_entry)
                if self.log_listbox.size() > 100:
                    self.log_listbox.delete(100, tk.END)
                    
                try:
                    with open("alerts_log.csv", "a", newline="") as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow([self._time, self._status, self._gps, self._temp, self._hum])
                except Exception as e:
                    print(f"CSV log error: {e}")
            except Exception:
                pass

# =============================
# MAIN
# =============================

def main():
    import sys
    root = tk.Tk()
    app = ModernBossMonitorGUI(root)

    # If user passed --simulate, feed sample messages to GUI for testing
    if '--simulate' in sys.argv:
        def simulate_messages():
            samples = [
                "AWAKE,GPS:(12.345678,98.765432),TEMP:25.3,HUM:45.2,METHANE:10.0,CO:0.1,LPG:5.0,SMOKE:0.0,AIR_QUALITY:98.5,DANGER:SAFE,REASONS:NONE,TIME:12:34:56",
                "DROWSY,GPS:(12.345900,98.765600),TEMP:30.2,HUM:70.1,METHANE:1200.0,CO:60.2,LPG:1500.0,SMOKE:10.0,AIR_QUALITY:20.0,DANGER:CRITICAL,REASONS:HIGH_METHANE,HIGH_CO,TIME:12:35:10",
                "ALERT,GPS:(-33.865143,151.209900),TEMP:22.1,HUM:40.0,METHANE:5.0,CO:0.0,LPG:2.0,SMOKE:0.0,AIR_QUALITY:99.0,DANGER:WARNING,REASONS:SMOKE_DETECTED,TIME:12:35:30"
            ]
            i = 0
            def send_next():
                nonlocal i
                app.process_message(samples[i % len(samples)])
                i += 1
                root.after(1500, send_next)
            root.after(1000, send_next)
        simulate_messages()

    root.mainloop()

if __name__ == "__main__":
    main()

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading

class RainSensorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MH-RD Rain Sensor Monitor")
        self.root.geometry("420x280")
        self.root.resizable(False, False)

        self.ser = None
        self.running = True

        self._create_widgets()
        self._refresh_ports()

        # Background thread to continuously read serial data
        self.rx_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.rx_thread.start()

    def _create_widgets(self):
        # 1. Connection Header
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding=10)
        conn_frame.pack(fill="x", padx=15, pady=8)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w", padx=4)
        self.port_combobox = ttk.Combobox(conn_frame, width=16, state="readonly")
        self.port_combobox.grid(row=0, column=1, padx=4)

        ttk.Button(conn_frame, text="Refresh", command=self._refresh_ports, width=8).grid(row=0, column=2, padx=4)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection, width=10)
        self.connect_btn.grid(row=0, column=3, padx=4)

        # 2. Rain Sensor Telemetry Display
        monitor_frame = ttk.LabelFrame(self.root, text="Live Readings", padding=15)
        monitor_frame.pack(fill="both", expand=True, padx=15, pady=8)

        self.moisture_label = ttk.Label(monitor_frame, text="Moisture Level: 0.0 %", font=("Helvetica", 11, "bold"))
        self.moisture_label.pack(anchor="w", pady=4)

        self.progress_bar = ttk.Progressbar(monitor_frame, orient="horizontal", length=360, mode="determinate", maximum=100)
        self.progress_bar.pack(pady=8)

        self.status_label = ttk.Label(monitor_frame, text="Condition: DRY", font=("Helvetica", 12, "bold"), foreground="green")
        self.status_label.pack(anchor="w", pady=4)

        # 3. Bottom Status Bar
        self.status_var = tk.StringVar(value="Disconnected")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w")
        status_bar.pack(side="bottom", fill="x")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combobox["values"] = ports
        if ports:
            self.port_combobox.current(0)

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.connect_btn.config(text="Connect")
            self.status_var.set("Disconnected")
            self.status_label.config(text="Condition: DRY", foreground="green")
            self.progress_bar["value"] = 0
            self.moisture_label.config(text="Moisture Level: 0.0 %")
        else:
            port = self.port_combobox.get()
            if not port:
                messagebox.showerror("Error", "No COM port selected!")
                return
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.connect_btn.config(text="Disconnect")
                self.status_var.set(f"Connected to {port} @ 115200 baud")
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    def _read_serial(self):
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line.startswith("RAIN:"):
                        payload = line[5:].split(",")
                        wetness = float(payload[0])
                        is_raining = int(payload[1]) == 1

                        self.root.after(0, self._update_gui, wetness, is_raining)
                except Exception:
                    pass

    def _update_gui(self, wetness, is_raining):
        self.progress_bar["value"] = wetness
        self.moisture_label.config(text=f"Moisture Level: {wetness:.1f} %")
        if is_raining:
            self.status_label.config(text="Condition: RAIN DETECTED!", foreground="red")
        else:
            self.status_label.config(text="Condition: DRY", foreground="green")

if __name__ == "__main__":
    root = tk.Tk()
    app = RainSensorApp(root)
    root.mainloop()
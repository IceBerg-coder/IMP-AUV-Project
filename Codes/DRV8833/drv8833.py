import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

class MotorControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("N20 Motor Controller (DRV8833 + STM32)")
        self.root.geometry("420x360")
        self.root.resizable(False, False)

        self.ser = None

        self._create_widgets()
        self._refresh_ports()

    def _create_widgets(self):
        # 1. Connection Panel
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding=10)
        conn_frame.pack(fill="x", padx=15, pady=8)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w", padx=5)
        self.port_combobox = ttk.Combobox(conn_frame, width=18, state="readonly")
        self.port_combobox.grid(row=0, column=1, padx=5)

        self.refresh_btn = ttk.Button(conn_frame, text="Refresh", command=self._refresh_ports, width=8)
        self.refresh_btn.grid(row=0, column=2, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection, width=10)
        self.connect_btn.grid(row=0, column=3, padx=5)

        # 2. Motor Speed Slider (-100% to +100%)
        control_frame = ttk.LabelFrame(self.root, text="Motor Throttle", padding=15)
        control_frame.pack(fill="x", padx=15, pady=8)

        self.slider_label = ttk.Label(control_frame, text="Throttle: 0 % (Stopped)", font=("Helvetica", 11, "bold"))
        self.slider_label.pack(pady=5)

        self.speed_slider = ttk.Scale(
            control_frame,
            from_=-100,
            to=100,
            orient="horizontal",
            command=self._on_slider_change,
            length=350
        )
        self.speed_slider.set(0)
        self.speed_slider.pack(pady=8)

        # 3. Action Buttons
        btn_frame = ttk.Frame(self.root, padding=10)
        btn_frame.pack(fill="x", padx=15)

        self.coast_btn = ttk.Button(btn_frame, text="Coast (Stop)", command=self._cmd_stop)
        self.coast_btn.pack(side="left", expand=True, fill="x", padx=4)

        self.brake_btn = ttk.Button(btn_frame, text="Active Brake", command=self._cmd_brake)
        self.brake_btn.pack(side="left", expand=True, fill="x", padx=4)

        # 4. Status Bar
        self.status_var = tk.StringVar(value="Disconnected")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w")
        status_bar.pack(side="bottom", fill="x")

    def _refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox["values"] = ports
        if ports:
            self.port_combobox.current(0)
        else:
            self.port_combobox.set("")

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.connect_btn.config(text="Connect")
            self.status_var.set("Disconnected")
        else:
            port = self.port_combobox.get()
            if not port:
                messagebox.showerror("Error", "No serial port selected!")
                return
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.connect_btn.config(text="Disconnect")
                self.status_var.set(f"Connected to {port} @ 115200 baud")
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    def _send_command(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode("utf-8"))
            except Exception as e:
                self.status_var.set(f"Tx Error: {e}")

    def _on_slider_change(self, value):
        val = int(float(value))
        if val > 0:
            self.slider_label.config(text=f"Throttle: +{val} % (Forward)")
        elif val < 0:
            self.slider_label.config(text=f"Throttle: {val} % (Reverse)")
        else:
            self.slider_label.config(text="Throttle: 0 % (Stopped)")

        duty = val / 100.0
        self._send_command(f"SPEED:{duty:.2f}")

    def _cmd_stop(self):
        self.speed_slider.set(0)
        self._on_slider_change(0)
        self._send_command("STOP")

    def _cmd_brake(self):
        self.speed_slider.set(0)
        self.slider_label.config(text="Throttle: BRAKE APPLIED")
        self._send_command("BRAKE")

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControllerApp(root)
    root.mainloop()
import tkinter as tk
import serial
import time

PORT = "COM3"          # Linux: "/dev/ttyACM0", macOS: "/dev/cu.usbmodemXXXX"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)          # give the board time to reset after the port opens

root = tk.Tk()
root.title("MG90S Servo Control")

def send(servo_id, angle):
    ser.write(f"{servo_id} {int(angle)}\n".encode())

def make_servo_panel(parent, servo_id):
    frame = tk.LabelFrame(parent, text=f"Servo {servo_id}", padx=10, pady=10)
    frame.pack(padx=10, pady=5, fill="x")

    value_label = tk.Label(frame, text="90°", font=("Arial", 14))
    value_label.pack()

    def on_change(v):
        angle = int(float(v))
        value_label.config(text=f"{angle}°")
        send(servo_id, angle)

    slider = tk.Scale(frame, from_=0, to=180, orient="horizontal",
                      length=350, command=on_change)
    slider.set(90)
    slider.pack()

    btns = tk.Frame(frame)
    btns.pack(pady=5)
    for label, angle in [("0°", 0), ("90°", 90), ("180°", 180)]:
        tk.Button(btns, text=label, width=6,
                  command=lambda a=angle: slider.set(a)).pack(side="left", padx=3)

    return slider

make_servo_panel(root, 1)
make_servo_panel(root, 2)

def on_close():
    ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
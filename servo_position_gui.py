import ctypes
from ctypes import wintypes
import os
import subprocess
import time
import tkinter as tk
from tkinter import messagebox


SERVO_CHANNELS = [12, 13, 14, 10, 8]
INITIAL_ANGLES = [90, 90, -90, 0, 0]
ANGLE_LIMITS = [(0, 180), (0, 180), (-150, 30), (-90, 90), (-90, 90)]
ANGLE_STEP = 5
BAUD_RATE = 115200
DEFAULT_PORT = "COM7"
MODE_COMMAND = os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32", "mode.com")

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = wintypes.HANDLE(-1).value
SETDTR = 5
CLRDTR = 6


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wintypes.DWORD),
        ("BaudRate", wintypes.DWORD),
        ("flags", wintypes.DWORD),
        ("wReserved", wintypes.WORD),
        ("XonLim", wintypes.WORD),
        ("XoffLim", wintypes.WORD),
        ("ByteSize", ctypes.c_ubyte),
        ("Parity", ctypes.c_ubyte),
        ("StopBits", ctypes.c_ubyte),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", wintypes.WORD),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wintypes.DWORD),
        ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
        ("ReadTotalTimeoutConstant", wintypes.DWORD),
        ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
        ("WriteTotalTimeoutConstant", wintypes.DWORD),
    ]


kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)


def _check_bool(result, func, args):
    if not result:
        raise ctypes.WinError(ctypes.get_last_error())
    return args


for _func_name in [
    "BuildCommDCBW",
    "SetCommState",
    "SetCommTimeouts",
    "SetupComm",
    "PurgeComm",
    "EscapeCommFunction",
    "CloseHandle",
]:
    getattr(kernel32, _func_name).errcheck = _check_bool


class WindowsSerialPort:
    def __init__(self, port: str, baud_rate: int = BAUD_RATE) -> None:
        self.port = port.upper()
        self.baud_rate = baud_rate
        self.handle = None

    def open(self) -> None:
        subprocess.run(
            [MODE_COMMAND, self.port, f"BAUD={self.baud_rate}", "PARITY=n", "DATA=8", "STOP=1"],
            check=True,
            capture_output=True,
            text=True,
        )

        device_path = rf"\\.\{self.port}"
        self.handle = kernel32.CreateFileW(
            ctypes.c_wchar_p(device_path),
            GENERIC_READ | GENERIC_WRITE,
            0,
            None,
            OPEN_EXISTING,
            0,
            None,
        )
        if self.handle == INVALID_HANDLE_VALUE:
            raise ctypes.WinError(ctypes.get_last_error())

        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        kernel32.BuildCommDCBW(f"baud={self.baud_rate} parity=n data=8 stop=1", ctypes.byref(dcb))
        kernel32.SetCommState(self.handle, ctypes.byref(dcb))
        kernel32.SetupComm(self.handle, 4096, 4096)

        timeouts = COMMTIMEOUTS()
        timeouts.ReadIntervalTimeout = 50
        timeouts.ReadTotalTimeoutMultiplier = 0
        timeouts.ReadTotalTimeoutConstant = 50
        timeouts.WriteTotalTimeoutMultiplier = 0
        timeouts.WriteTotalTimeoutConstant = 50
        kernel32.SetCommTimeouts(self.handle, ctypes.byref(timeouts))
        kernel32.PurgeComm(self.handle, 0x0004 | 0x0008 | 0x0002 | 0x0001)

    def reset_arduino(self) -> None:
        kernel32.EscapeCommFunction(self.handle, CLRDTR)
        time.sleep(0.1)
        kernel32.EscapeCommFunction(self.handle, SETDTR)
        time.sleep(1.8)
        kernel32.PurgeComm(self.handle, 0x0004 | 0x0008 | 0x0002 | 0x0001)

    def close(self) -> None:
        if self.handle not in (None, INVALID_HANDLE_VALUE):
            kernel32.CloseHandle(self.handle)
        self.handle = None

    def write_line(self, text: str) -> None:
        data = (text + "\n").encode("ascii")
        written = wintypes.DWORD()
        ok = kernel32.WriteFile(self.handle, data, len(data), ctypes.byref(written), None)
        if not ok:
            raise ctypes.WinError(ctypes.get_last_error())

    def read_line(self, timeout_seconds: float = 2.0) -> str:
        deadline = time.monotonic() + timeout_seconds
        buffer = bytearray()
        while time.monotonic() < deadline:
            chunk = ctypes.create_string_buffer(1)
            read = wintypes.DWORD()
            ok = kernel32.ReadFile(self.handle, chunk, 1, ctypes.byref(read), None)
            if not ok:
                raise ctypes.WinError(ctypes.get_last_error())
            if read.value == 0:
                continue
            if chunk.raw == b"\n":
                return buffer.decode("ascii", errors="replace").strip()
            if chunk.raw != b"\r":
                buffer.extend(chunk.raw)
        raise TimeoutError("Timed out waiting for serial response.")


class PositionalServoBridgeClient:
    def __init__(self, port: str = DEFAULT_PORT) -> None:
        self.serial = WindowsSerialPort(port)

    def connect(self) -> None:
        self.serial.open()
        self.serial.reset_arduino()

        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            try:
                line = self.serial.read_line(timeout_seconds=0.5)
            except TimeoutError:
                break
            if line == "READY":
                return

        self.get_all_states()

    def disconnect(self) -> None:
        self.serial.close()

    def get_all_states(self) -> dict[int, dict[str, int]]:
        self.serial.write_line("GETALL")
        result: dict[int, dict[str, int]] = {}

        while True:
            line = self.serial.read_line()
            if line == "DONE":
                return result
            if not line.startswith("STATE "):
                raise RuntimeError(f"Unexpected response: {line}")

            _, index_text, channel_text, angle_text, pulse_text = line.split()
            result[int(index_text)] = {
                "channel": int(channel_text),
                "angle": int(angle_text),
                "pulse": int(pulse_text),
            }

    def set_angle(self, servo_index: int, angle: int) -> dict[str, int]:
        self.serial.write_line(f"SET {servo_index} {angle}")
        line = self.serial.read_line()
        if not line.startswith("OK "):
            raise RuntimeError(f"Unexpected response: {line}")
        _, index_text, channel_text, angle_text, pulse_text = line.split()
        return {
            "index": int(index_text),
            "channel": int(channel_text),
            "angle": int(angle_text),
            "pulse": int(pulse_text),
        }

    def home_all(self) -> None:
        self.serial.write_line("HOME")
        line = self.serial.read_line()
        if line != "OK HOME":
            raise RuntimeError(f"Unexpected response: {line}")


class ServoPositionApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PCA9685 Positional Servo Controller")
        self.client = PositionalServoBridgeClient()
        self.servo_state: dict[int, dict[str, tk.StringVar | int]] = {}

        self.status_var = tk.StringVar(value="Disconnected")
        self._build_ui()
        self._connect_and_load()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        frame = tk.Frame(self.root, padx=12, pady=12)
        frame.pack(fill="both", expand=True)

        tk.Label(
            frame,
            text="Independent 5 degree control for 5 positional servos on PCA9685",
            anchor="w",
            justify="left",
        ).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 12))

        tk.Label(frame, text="Servo", width=10).grid(row=1, column=0, sticky="w")
        tk.Label(frame, text="PCA Ch", width=10).grid(row=1, column=1, sticky="w")
        tk.Label(frame, text="Angle", width=12).grid(row=1, column=2, sticky="w")
        tk.Label(frame, text="Pulse", width=10).grid(row=1, column=3, sticky="w")

        for servo_index, channel in enumerate(SERVO_CHANNELS):
            angle_var = tk.StringVar(value=f"{INITIAL_ANGLES[servo_index]} deg")
            pulse_var = tk.StringVar(value="--")
            min_angle, max_angle = ANGLE_LIMITS[servo_index]
            self.servo_state[servo_index] = {
                "channel": channel,
                "angle": INITIAL_ANGLES[servo_index],
                "min_angle": min_angle,
                "max_angle": max_angle,
                "angle_var": angle_var,
                "pulse_var": pulse_var,
            }

            row_index = servo_index + 2
            tk.Label(frame, text=f"Servo {servo_index + 1}", width=10).grid(row=row_index, column=0, sticky="w", pady=4)
            tk.Label(frame, text=str(channel), width=10).grid(row=row_index, column=1, sticky="w")
            tk.Label(frame, textvariable=angle_var, width=12).grid(row=row_index, column=2, sticky="w")
            tk.Label(frame, textvariable=pulse_var, width=10).grid(row=row_index, column=3, sticky="w")
            tk.Button(
                frame,
                text="-5 deg",
                width=8,
                command=lambda idx=servo_index: self.adjust_servo(idx, -ANGLE_STEP),
            ).grid(row=row_index, column=4, padx=4)
            tk.Button(
                frame,
                text="+5 deg",
                width=8,
                command=lambda idx=servo_index: self.adjust_servo(idx, ANGLE_STEP),
            ).grid(row=row_index, column=5, padx=4)

        button_row = len(SERVO_CHANNELS) + 2
        tk.Button(frame, text="Home All", command=self.home_all).grid(
            row=button_row, column=0, columnspan=2, sticky="w", pady=(12, 0)
        )
        tk.Button(frame, text="Reconnect", command=self._connect_and_load).grid(
            row=button_row, column=2, columnspan=2, sticky="w", pady=(12, 0)
        )
        tk.Label(frame, textvariable=self.status_var, anchor="w").grid(
            row=button_row + 1, column=0, columnspan=6, sticky="w", pady=(12, 0)
        )

    def _connect_and_load(self) -> None:
        try:
            self.status_var.set("Connecting to bridge...")
            self.client.disconnect()
            self.client.connect()
            self._reload_states()
            self.status_var.set(f"Connected on {self.client.serial.port}")
        except Exception as exc:
            self.status_var.set("Connection failed")
            messagebox.showerror("Serial bridge error", str(exc))

    def _reload_states(self) -> None:
        states = self.client.get_all_states()
        for servo_index in range(len(SERVO_CHANNELS)):
            state = states.get(servo_index)
            if state is None:
                continue
            self._update_servo_display(servo_index, state["angle"], state["pulse"])

    def _update_servo_display(self, servo_index: int, angle: int, pulse: int) -> None:
        servo_info = self.servo_state[servo_index]
        servo_info["angle"] = angle
        servo_info["angle_var"].set(f"{angle} deg")
        servo_info["pulse_var"].set(str(pulse))

    def adjust_servo(self, servo_index: int, delta: int) -> None:
        servo_info = self.servo_state[servo_index]
        angle = int(servo_info["angle"])
        min_angle = int(servo_info["min_angle"])
        max_angle = int(servo_info["max_angle"])
        new_angle = max(min_angle, min(max_angle, angle + delta))

        try:
            response = self.client.set_angle(servo_index, new_angle)
            self._update_servo_display(servo_index, response["angle"], response["pulse"])
            self.status_var.set(f"Set servo {servo_index + 1} to {response['angle']} deg")
        except Exception as exc:
            self.status_var.set(f"Failed to set servo {servo_index + 1}")
            messagebox.showerror("Set failed", str(exc))

    def home_all(self) -> None:
        try:
            self.client.home_all()
            self._reload_states()
            self.status_var.set("Returned all servos to initial positions")
        except Exception as exc:
            self.status_var.set("Failed to home servos")
            messagebox.showerror("Home failed", str(exc))

    def on_close(self) -> None:
        self.client.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ServoPositionApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()

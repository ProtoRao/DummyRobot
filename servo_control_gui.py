import ctypes
from ctypes import wintypes
import os
import subprocess
import time
import tkinter as tk
from tkinter import messagebox


PCA_CHANNELS = [8, 9, 10, 12, 13, 14]
ANGLE_STEP = 5
ANGLE_MIN = 0
ANGLE_STOP = 90
ANGLE_MAX = 180
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


class ServoBridgeClient:
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

        # Some boards won't emit the startup banner reliably; a working GETALL
        # response is enough to consider the bridge connected.
        self.get_all_states()

    def disconnect(self) -> None:
        self.serial.close()

    def get_all_states(self) -> dict[int, tuple[int, int]]:
        self.serial.write_line("GETALL")
        result: dict[int, tuple[int, int]] = {}

        while True:
            line = self.serial.read_line()
            if line == "DONE":
                return result
            if not line.startswith("STATE "):
                raise RuntimeError(f"Unexpected response: {line}")

            _, channel_text, angle_text, pulse_text = line.split()
            result[int(channel_text)] = (int(angle_text), int(pulse_text))

    def set_angle(self, channel: int, angle: int) -> tuple[int, int]:
        self.serial.write_line(f"SET {channel} {angle}")
        line = self.serial.read_line()
        if not line.startswith("OK "):
            raise RuntimeError(f"Unexpected response: {line}")
        _, channel_text, angle_text, pulse_text = line.split()
        return int(angle_text), int(pulse_text)

    def stop_channel(self, channel: int) -> tuple[int, int]:
        self.serial.write_line(f"STOP {channel}")
        line = self.serial.read_line()
        if not line.startswith("OK "):
            raise RuntimeError(f"Unexpected response: {line}")
        _, channel_text, angle_text, pulse_text = line.split()
        return int(angle_text), int(pulse_text)


class ServoControlApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PCA9685 Continuous Servo Controller")
        self.client = ServoBridgeClient()
        self.channel_state: dict[int, dict[str, tk.StringVar | int]] = {}

        self.status_var = tk.StringVar(value="Disconnected")
        self._build_ui()
        self._connect_and_load()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        frame = tk.Frame(self.root, padx=12, pady=12)
        frame.pack(fill="both", expand=True)

        header = tk.Label(
            frame,
            text="Continuous servo control: 0 = full reverse, 90 = stop, 180 = full forward",
            anchor="w",
            justify="left",
        )
        header.grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 12))

        tk.Label(frame, text="Channel", width=10).grid(row=1, column=0, sticky="w")
        tk.Label(frame, text="Command", width=14).grid(row=1, column=1, sticky="w")
        tk.Label(frame, text="Pulse", width=10).grid(row=1, column=2, sticky="w")

        row_index = 2
        for channel in PCA_CHANNELS:
            angle_var = tk.StringVar(value="--")
            pulse_var = tk.StringVar(value="--")
            self.channel_state[channel] = {
                "angle": 90,
                "angle_var": angle_var,
                "pulse_var": pulse_var,
            }

            tk.Label(frame, text=str(channel), width=10).grid(row=row_index, column=0, sticky="w", pady=4)
            tk.Label(frame, textvariable=angle_var, width=14).grid(row=row_index, column=1, sticky="w")
            tk.Label(frame, textvariable=pulse_var, width=10).grid(row=row_index, column=2, sticky="w")
            tk.Button(
                frame,
                text="-",
                width=6,
                command=lambda ch=channel: self.adjust_channel(ch, -ANGLE_STEP),
            ).grid(row=row_index, column=3, padx=4)
            tk.Button(
                frame,
                text="+",
                width=6,
                command=lambda ch=channel: self.adjust_channel(ch, ANGLE_STEP),
            ).grid(row=row_index, column=4, padx=4)
            tk.Button(
                frame,
                text="Stop",
                width=8,
                command=lambda ch=channel: self.stop_channel(ch),
            ).grid(row=row_index, column=5, padx=4)

            row_index += 1

        tk.Button(frame, text="Stop All", command=self.stop_all).grid(
            row=row_index, column=0, columnspan=2, sticky="w", pady=(12, 0)
        )
        tk.Button(frame, text="Reconnect", command=self._connect_and_load).grid(
            row=row_index, column=2, columnspan=2, sticky="w", pady=(12, 0)
        )
        tk.Label(frame, textvariable=self.status_var, anchor="w").grid(
            row=row_index + 1, column=0, columnspan=6, sticky="w", pady=(12, 0)
        )

    def _connect_and_load(self) -> None:
        try:
            self.status_var.set("Connecting to bridge...")
            self.client.disconnect()
            self.client.connect()
            states = self.client.get_all_states()
            for channel in PCA_CHANNELS:
                angle, pulse = states.get(channel, (ANGLE_STOP, 307))
                self._update_channel_display(channel, angle, pulse)
            self.status_var.set(f"Connected on {self.client.serial.port}")
        except Exception as exc:
            self.status_var.set("Connection failed")
            messagebox.showerror("Serial bridge error", str(exc))

    def _update_channel_display(self, channel: int, angle: int, pulse: int) -> None:
        channel_info = self.channel_state[channel]
        channel_info["angle"] = angle
        channel_info["angle_var"].set(f"{angle} deg")
        channel_info["pulse_var"].set(str(pulse))

    def adjust_channel(self, channel: int, delta: int) -> None:
        channel_info = self.channel_state[channel]
        angle = int(channel_info["angle"])
        new_angle = max(ANGLE_MIN, min(ANGLE_MAX, angle + delta))

        try:
            confirmed_angle, pulse = self.client.set_angle(channel, new_angle)
            self._update_channel_display(channel, confirmed_angle, pulse)
            self.status_var.set(f"Set channel {channel} to {confirmed_angle}")
        except Exception as exc:
            self.status_var.set(f"Failed to set channel {channel}")
            messagebox.showerror("Set failed", str(exc))

    def stop_channel(self, channel: int) -> None:
        try:
            confirmed_angle, pulse = self.client.stop_channel(channel)
            self._update_channel_display(channel, confirmed_angle, pulse)
            self.status_var.set(f"Stopped channel {channel}")
        except Exception as exc:
            self.status_var.set(f"Failed to stop channel {channel}")
            messagebox.showerror("Stop failed", str(exc))

    def stop_all(self) -> None:
        for channel in PCA_CHANNELS:
            self.stop_channel(channel)

    def on_close(self) -> None:
        self.client.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ServoControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Small Tk GUI for manual Armtrack gimbal control."""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from gimbal_protocol import (
    command_home, command_pitch_position, command_pitch_speed, command_pose,
    command_stop, command_track, command_yaw_position, command_yaw_speed,
    command_tick, describe_line,
)


class GimbalGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Armtrack Gimbal Control")
        self.root.minsize(700, 500)
        self.port = None
        self.rx_buffer = bytearray()
        self.tick_sequence = 0
        self.heartbeat_job = None
        self.port_var = tk.StringVar(value="COM34")
        self.yaw_speed_var = tk.IntVar(value=10)
        self.yaw_pos_var = tk.StringVar(value="130.0")
        self.pitch_pos_var = tk.StringVar(value="330.0")
        self.pitch_speed_var = tk.IntVar(value=5)
        self.pose_yaw_var = tk.StringVar(value="130.0")
        self.pose_pitch_var = tk.StringVar(value="330.0")
        self.track_yaw_var = tk.IntVar(value=10)
        self.track_pitch_var = tk.StringVar(value="330.0")
        self.custom_var = tk.StringVar(value="home")
        self.status_vars = {name: tk.StringVar(value="--") for name in ("yaw_cur", "yaw_tgt", "pitch_cur", "pitch_tgt", "homed", "home_fault", "state")}
        self._build()
        self.refresh_ports()

    def _build(self) -> None:
        connection = ttk.LabelFrame(self.root, text="Serial")
        connection.pack(fill="x", padx=10, pady=8)
        ttk.Label(connection, text="Port").grid(row=0, column=0, padx=5, pady=5)
        ttk.Entry(connection, textvariable=self.port_var, width=12).grid(row=0, column=1, padx=5)
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        ttk.Button(connection, text="Connect / Disconnect", command=self.toggle_connection).grid(row=0, column=3, padx=5)
        ttk.Button(connection, text="HOME", command=lambda: self.send(command_home())).grid(row=0, column=4, padx=5)
        ttk.Button(connection, text="STOP", command=lambda: self.send(command_stop())).grid(row=0, column=5, padx=5)

        controls = ttk.LabelFrame(self.root, text="Commands")
        controls.pack(fill="x", padx=10, pady=4)
        self._row(controls, 0, "Yaw speed rpm", self.yaw_speed_var, lambda: self._safe(lambda: command_yaw_speed(self.yaw_speed_var.get())))
        self._row(controls, 1, "Yaw position", self.yaw_pos_var, lambda: self._safe(lambda: command_yaw_position(self.yaw_pos_var.get())))
        self._row(controls, 2, "Pitch position", self.pitch_pos_var, lambda: self._safe(lambda: command_pitch_position(self.pitch_pos_var.get())))
        self._row(controls, 3, "Pitch speed rpm", self.pitch_speed_var, lambda: self._safe(lambda: command_pitch_speed(self.pitch_speed_var.get())))
        self._row(controls, 4, "Track yaw rpm", self.track_yaw_var, lambda: self._safe(lambda: command_track(self.track_yaw_var.get(), self.track_pitch_var.get())))
        ttk.Label(controls, text="Track pitch").grid(row=4, column=2, padx=5)
        ttk.Entry(controls, textvariable=self.track_pitch_var, width=12).grid(row=4, column=3)
        self._row(controls, 5, "Pose yaw", self.pose_yaw_var, lambda: self._safe(lambda: command_pose(self.pose_yaw_var.get(), self.pose_pitch_var.get())))
        ttk.Label(controls, text="Pose pitch").grid(row=5, column=2, padx=5)
        ttk.Entry(controls, textvariable=self.pose_pitch_var, width=12).grid(row=5, column=3)
        ttk.Entry(controls, textvariable=self.custom_var, width=28).grid(row=6, column=0, columnspan=3, padx=5, pady=6)
        ttk.Button(controls, text="Send custom", command=lambda: self.send(self.custom_var.get())).grid(row=6, column=3, padx=5)

        status = ttk.LabelFrame(self.root, text="Status")
        status.pack(fill="x", padx=10, pady=4)
        for index, name in enumerate(self.status_vars):
            ttk.Label(status, text=name).grid(row=0, column=index, padx=5)
            ttk.Label(status, textvariable=self.status_vars[name], width=12).grid(row=1, column=index, padx=5)
        self.log = tk.Text(self.root, height=12, state="disabled")
        self.log.pack(fill="both", expand=True, padx=10, pady=8)

    @staticmethod
    def _row(parent, row, label, variable, callback) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, padx=5, pady=4, sticky="w")
        ttk.Entry(parent, textvariable=variable, width=12).grid(row=row, column=1, padx=5)
        ttk.Button(parent, text="Send", command=callback).grid(row=row, column=3, padx=5)

    def _safe(self, builder) -> None:
        try:
            self.send(builder())
        except ValueError as exc:
            messagebox.showerror("Invalid command", str(exc))

    def log_line(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text.rstrip() + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def refresh_ports(self) -> None:
        if list_ports is None:
            return
        ports = [port.device for port in list_ports.comports()]
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self.log_line("ports: " + (", ".join(ports) if ports else "none"))

    def toggle_connection(self) -> None:
        if self.port is not None:
            self.port.close()
            self.port = None
            if self.heartbeat_job is not None:
                self.root.after_cancel(self.heartbeat_job)
                self.heartbeat_job = None
            self.log_line("disconnected")
            return
        if serial is None:
            messagebox.showerror("Missing dependency", "Install pyserial from requirements.txt")
            return
        try:
            self.port = serial.Serial(self.port_var.get(), 115200, timeout=0.05, write_timeout=0.5)
            self.log_line(f"connected {self.port_var.get()}")
            self._poll_rx()
            self._heartbeat()
        except (OSError, serial.SerialException) as exc:
            self.port = None
            messagebox.showerror("Serial error", str(exc))

    def send(self, command: str) -> None:
        if self.port is None:
            self.log_line("not connected: " + command)
            return
        self.port.write((command.strip() + "\r\n").encode("ascii"))
        self.port.flush()
        self.log_line("TX " + command.strip())

    def _poll_rx(self) -> None:
        if self.port is None:
            return
        try:
            self.rx_buffer.extend(self.port.read(self.port.in_waiting or 1))
        except (OSError, serial.SerialException):
            self.port = None
            return
        while b"\n" in self.rx_buffer or b"\r" in self.rx_buffer:
            indexes = [index for index in (self.rx_buffer.find(b"\r"), self.rx_buffer.find(b"\n")) if index >= 0]
            index = min(indexes)
            line = bytes(self.rx_buffer[:index]).decode("ascii", errors="replace")
            del self.rx_buffer[:index + 1]
            while self.rx_buffer[:1] in (b"\r", b"\n"):
                del self.rx_buffer[:1]
            if line:
                self.log_line("RX " + describe_line(line))
                for item in line.split(","):
                    if ":" in item:
                        key, value = item.split(":", 1)
                        if key in self.status_vars:
                            self.status_vars[key].set(value)
        self.root.after(50, self._poll_rx)

    def _heartbeat(self) -> None:
        if self.port is None:
            return
        self.tick_sequence += 1
        self.send(command_tick(self.tick_sequence))
        self.heartbeat_job = self.root.after(1000, self._heartbeat)


def main() -> None:
    root = tk.Tk()
    GimbalGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rover WiFi Debug GUI v0.1

Назначение:
    GUI-терминал для отладки ровера через цепочку:
        STM32 rover <-> UART <-> ESP32 <-> Wi-Fi/TCP <-> PC Python GUI

Архитектура:
    - Python GUI на ноутбуке поднимает TCP server.
    - ESP32 подключается к ноутбуку как TCP client.
    - Входящая телеметрия выводится в отдельное окно.
    - Команды отправляются через поле ввода или кнопки.

Важно:
    STOP отправляет ТОЛЬКО:
        drive 0 0
    STOP НЕ отправляет en 0.
"""

import socket
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from datetime import datetime
from pathlib import Path


APP_VERSION = "Rover WiFi Debug GUI v0.1"

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 3333

LOG_DIR = Path("logs")


class RoverTcpServer:
    """
    TCP server for ESP32 client.

    GUI thread must not call blocking socket operations directly.
    This class uses background threads and reports events through rx_queue.
    """

    def __init__(self, rx_queue: queue.Queue):
        self.rx_queue = rx_queue

        self.host = DEFAULT_HOST
        self.port = DEFAULT_PORT

        self.server_socket = None
        self.client_socket = None
        self.client_addr = None

        self.server_thread = None
        self.rx_thread = None

        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    def start(self, host: str, port: int):
        self.stop()

        self.host = host
        self.port = int(port)
        self.stop_event.clear()

        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()

    def stop(self):
        self.stop_event.set()

        with self.lock:
            if self.client_socket is not None:
                try:
                    self.client_socket.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    self.client_socket.close()
                except OSError:
                    pass
                self.client_socket = None
                self.client_addr = None

            if self.server_socket is not None:
                try:
                    self.server_socket.close()
                except OSError:
                    pass
                self.server_socket = None

    def is_connected(self) -> bool:
        with self.lock:
            return self.client_socket is not None

    def get_client_addr_text(self) -> str:
        with self.lock:
            if self.client_addr is None:
                return ""
            return f"{self.client_addr[0]}:{self.client_addr[1]}"

    def send_line(self, line: str) -> bool:
        """
        Send command line to ESP32/STM32.
        Always appends CRLF if missing.
        """
        if line is None:
            return False

        text = line.strip()
        if not text:
            return False

        payload = (text + "\n").encode("utf-8", errors="replace")

        with self.lock:
            sock = self.client_socket

        if sock is None:
            self.rx_queue.put(("status", "TX failed: no ESP32 connected"))
            return False

        try:
            sock.sendall(payload)
            return True
        except OSError as exc:
            self.rx_queue.put(("status", f"TX failed: {exc}"))
            self._drop_client()
            return False

    def _server_loop(self):
        self.rx_queue.put(("status", f"Starting TCP server on {self.host}:{self.port}"))

        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(1)
            srv.settimeout(0.5)

            with self.lock:
                self.server_socket = srv

            self.rx_queue.put(("status", f"Waiting for ESP32 on port {self.port}"))

            while not self.stop_event.is_set():
                try:
                    client, addr = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                with self.lock:
                    old_client = self.client_socket
                    self.client_socket = client
                    self.client_addr = addr

                if old_client is not None:
                    try:
                        old_client.close()
                    except OSError:
                        pass

                self.rx_queue.put(("connected", f"{addr[0]}:{addr[1]}"))
                self.rx_queue.put(("rx", f"\n[INFO] ESP32 connected from {addr[0]}:{addr[1]}\n"))

                self.rx_thread = threading.Thread(target=self._rx_loop, args=(client, addr), daemon=True)
                self.rx_thread.start()

        except OSError as exc:
            self.rx_queue.put(("status", f"Server error: {exc}"))
        finally:
            self._close_server_socket()
            self.rx_queue.put(("disconnected", "server stopped"))

    def _rx_loop(self, client: socket.socket, addr):
        while not self.stop_event.is_set():
            try:
                data = client.recv(4096)
            except OSError as exc:
                self.rx_queue.put(("rx", f"\n[INFO] RX socket closed: {exc}\n"))
                break

            if not data:
                self.rx_queue.put(("rx", "\n[INFO] ESP32 disconnected\n"))
                break

            text = data.decode("utf-8", errors="replace")
            self.rx_queue.put(("rx", text))

        with self.lock:
            if self.client_socket is client:
                try:
                    self.client_socket.close()
                except OSError:
                    pass
                self.client_socket = None
                self.client_addr = None

        self.rx_queue.put(("disconnected", "ESP32 disconnected"))

    def _drop_client(self):
        with self.lock:
            sock = self.client_socket
            self.client_socket = None
            self.client_addr = None

        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

        self.rx_queue.put(("disconnected", "ESP32 disconnected"))

    def _close_server_socket(self):
        with self.lock:
            sock = self.server_socket
            self.server_socket = None

        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


class RoverGui(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title(APP_VERSION)
        self.geometry("1100x720")
        self.minsize(850, 520)

        self.rx_queue = queue.Queue()
        self.server = RoverTcpServer(self.rx_queue)

        self.log_lines = []
        self.autoscroll_var = tk.BooleanVar(value=True)
        self.echo_tx_var = tk.BooleanVar(value=True)

        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        self.port_var = tk.StringVar(value=str(DEFAULT_PORT))
        self.status_var = tk.StringVar(value="STOPPED")
        self.client_var = tk.StringVar(value="-")

        self.command_var = tk.StringVar(value="")

        self._create_widgets()
        self._bind_events()

        self.after(50, self._poll_queue)

        # Start server automatically for convenience
        self._start_server()

    def _create_widgets(self):
        root = ttk.Frame(self, padding=8)
        root.pack(fill=tk.BOTH, expand=True)

        # Top connection frame
        conn = ttk.LabelFrame(root, text="Connection")
        conn.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(conn, text="Host:").pack(side=tk.LEFT, padx=(8, 4), pady=6)
        self.host_entry = ttk.Entry(conn, textvariable=self.host_var, width=14)
        self.host_entry.pack(side=tk.LEFT, padx=(0, 8), pady=6)

        ttk.Label(conn, text="Port:").pack(side=tk.LEFT, padx=(0, 4), pady=6)
        self.port_entry = ttk.Entry(conn, textvariable=self.port_var, width=7)
        self.port_entry.pack(side=tk.LEFT, padx=(0, 8), pady=6)

        self.start_btn = ttk.Button(conn, text="Start server", command=self._start_server)
        self.start_btn.pack(side=tk.LEFT, padx=(0, 4), pady=6)

        self.stop_server_btn = ttk.Button(conn, text="Stop server", command=self._stop_server)
        self.stop_server_btn.pack(side=tk.LEFT, padx=(0, 12), pady=6)

        ttk.Label(conn, text="Status:").pack(side=tk.LEFT, padx=(8, 4), pady=6)
        self.status_label = ttk.Label(conn, textvariable=self.status_var, width=28)
        self.status_label.pack(side=tk.LEFT, padx=(0, 8), pady=6)

        ttk.Label(conn, text="Client:").pack(side=tk.LEFT, padx=(8, 4), pady=6)
        self.client_label = ttk.Label(conn, textvariable=self.client_var)
        self.client_label.pack(side=tk.LEFT, padx=(0, 8), pady=6)

        # Telemetry log
        log_frame = ttk.LabelFrame(root, text="Telemetry / Log")
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.text = tk.Text(
            log_frame,
            wrap=tk.NONE,
            height=24,
            font=("Consolas", 10),
            undo=False,
        )
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        yscroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.text.yview)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.text.configure(yscrollcommand=yscroll.set)

        xscroll = ttk.Scrollbar(root, orient=tk.HORIZONTAL, command=self.text.xview)
        xscroll.pack(fill=tk.X)
        self.text.configure(xscrollcommand=xscroll.set)

        # Command input
        cmd_frame = ttk.LabelFrame(root, text="Command")
        cmd_frame.pack(fill=tk.X, pady=(8, 8))

        self.command_entry = ttk.Entry(cmd_frame, textvariable=self.command_var, font=("Consolas", 11))
        self.command_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 6), pady=8)

        self.send_btn = ttk.Button(cmd_frame, text="Send", command=self._send_current_command)
        self.send_btn.pack(side=tk.LEFT, padx=(0, 8), pady=8)

        # Buttons
        btn_frame = ttk.Frame(root)
        btn_frame.pack(fill=tk.X)

        self.stop_btn = tk.Button(
            btn_frame,
            text="STOP: drive 0 0",
            command=lambda: self._send_command("drive 0 0"),
            bg="#b00020",
            fg="white",
            activebackground="#d00030",
            activeforeground="white",
            font=("Segoe UI", 11, "bold"),
            height=2,
            width=20,
        )
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 8), pady=4)

        ttk.Button(btn_frame, text="en 0", command=lambda: self._send_command("en 0")).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(btn_frame, text="en 1", command=lambda: self._send_command("en 1")).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(btn_frame, text="drive 0 0", command=lambda: self._send_command("drive 0 0")).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(btn_frame, text="get all", command=lambda: self._send_command("get all")).pack(side=tk.LEFT, padx=4, pady=4)

        ttk.Separator(btn_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=4)

        ttk.Button(btn_frame, text="Clear log", command=self._clear_log).pack(side=tk.LEFT, padx=4, pady=4)
        ttk.Button(btn_frame, text="Save log", command=self._save_log_dialog).pack(side=tk.LEFT, padx=4, pady=4)

        ttk.Checkbutton(btn_frame, text="Autoscroll", variable=self.autoscroll_var).pack(side=tk.RIGHT, padx=4, pady=4)
        ttk.Checkbutton(btn_frame, text="Echo TX", variable=self.echo_tx_var).pack(side=tk.RIGHT, padx=4, pady=4)

    def _bind_events(self):
        self.command_entry.bind("<Return>", lambda event: self._send_current_command())
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _start_server(self):
        try:
            port = int(self.port_var.get().strip())
            if port <= 0 or port > 65535:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid port", "Port must be an integer from 1 to 65535.")
            return

        host = self.host_var.get().strip() or DEFAULT_HOST

        self.status_var.set("STARTING")
        self.client_var.set("-")
        self._append_log(f"[INFO] {APP_VERSION}\n")
        self._append_log(f"[INFO] Starting server {host}:{port}\n")
        self.server.start(host, port)

    def _stop_server(self):
        self.server.stop()
        self.status_var.set("STOPPED")
        self.client_var.set("-")
        self._append_log("[INFO] Server stopped by user\n")

    def _send_current_command(self):
        cmd = self.command_var.get()
        self._send_command(cmd)
        self.command_var.set("")
        self.command_entry.focus_set()

    def _send_command(self, cmd: str):
        cmd = (cmd or "").strip()
        if not cmd:
            return

        ok = self.server.send_line(cmd)

        if self.echo_tx_var.get():
            prefix = "[TX]" if ok else "[TX FAILED]"
            self._append_log(f"{prefix} {cmd}\n")

    def _poll_queue(self):
        processed = 0

        while processed < 200:
            try:
                kind, payload = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            processed += 1

            if kind == "rx":
                self._append_log(payload)
            elif kind == "status":
                self.status_var.set(payload)
                self._append_log(f"[INFO] {payload}\n")
            elif kind == "connected":
                self.status_var.set("CONNECTED")
                self.client_var.set(payload)
            elif kind == "disconnected":
                if self.status_var.get() != "STOPPED":
                    self.status_var.set("WAITING")
                self.client_var.set("-")

        self.after(50, self._poll_queue)

    def _append_log(self, text: str):
        if not text:
            return

        self.text.insert(tk.END, text)
        self.log_lines.append(text)

        # Limit GUI text buffer to avoid very slow operation during long tests.
        # Full session can still be saved if it stays in memory, but we also limit memory.
        max_chars = 3_000_000
        try:
            current_chars = int(self.text.index("end-1c").split(".")[0])
            # This is line count, not chars. Keep enough lines for live debug.
            max_lines = 30000
            if current_chars > max_lines:
                self.text.delete("1.0", "5000.0")
        except Exception:
            pass

        if len(self.log_lines) > 200000:
            self.log_lines = self.log_lines[-100000:]

        if self.autoscroll_var.get():
            self.text.see(tk.END)

    def _clear_log(self):
        self.text.delete("1.0", tk.END)
        self.log_lines.clear()
        self._append_log("[INFO] Log cleared\n")

    def _save_log_dialog(self):
        LOG_DIR.mkdir(exist_ok=True)
        default_name = datetime.now().strftime("rover_wifi_log_%Y%m%d_%H%M%S.txt")

        path = filedialog.asksaveasfilename(
            title="Save rover log",
            initialdir=str(LOG_DIR),
            initialfile=default_name,
            defaultextension=".txt",
            filetypes=[("Text log", "*.txt"), ("All files", "*.*")],
        )

        if not path:
            return

        self._save_log(Path(path))

    def _save_log(self, path: Path):
        try:
            path.write_text("".join(self.log_lines), encoding="utf-8", errors="replace")
            self._append_log(f"[INFO] Log saved: {path}\n")
        except OSError as exc:
            messagebox.showerror("Save failed", str(exc))

    def _autosave_log_on_exit(self):
        if not self.log_lines:
            return

        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / datetime.now().strftime("rover_wifi_log_%Y%m%d_%H%M%S_autosave.txt")

        try:
            path.write_text("".join(self.log_lines), encoding="utf-8", errors="replace")
        except OSError:
            pass

    def _on_close(self):
        self._autosave_log_on_exit()
        self.server.stop()
        self.destroy()


def main():
    app = RoverGui()
    app.mainloop()


if __name__ == "__main__":
    main()

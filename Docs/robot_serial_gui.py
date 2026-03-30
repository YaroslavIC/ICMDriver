#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import os
import queue
import re
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
import serial.tools.list_ports
import urllib.request
import urllib.error
import ssl

try:
    from openai import OpenAI
    import openai as openai_pkg
    OPENAI_AVAILABLE = True
except Exception:
    OpenAI = None
    openai_pkg = None
    OPENAI_AVAILABLE = False


APP_VERSION = "v10.0.0"

DBG_PATTERN = re.compile(
    r"DBG\s+.*?\bt=(?P<t>-?\d+).*?"
    r"\ben=(?P<en>-?\d+).*?"
    r"\bmode=(?P<mode>[A-Za-z0-9_]+).*?"
    r"\bpitch=(?P<pitch>-?\d+).*?"
    r"\brate=(?P<rate>-?\d+).*?"
    r"\bwv=(?P<wv>-?\d+).*?"
    r"\bu=(?P<u>-?\d+)"
)

KV_PATTERN = re.compile(
    r"(?P<key>[A-Za-z0-9_]+)\s*(?:=|:)\s*"
    r"(?P<value>[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)"
)

PARAM_NAMES = [
    "control_u_limit",
    "control_k_pitch",
    "control_k_pitch_rate",
    "control_k_wheel_vel",
    "control_k_wheel_pos",
    "control_k_sync",
    "control_u_sync_limit",
    "vertical_pitch_thresh_mrad",
    "vertical_rate_thresh_mrads",
    "imu_pitch_zero_offset_rad",
    "balance_target_pitch_rad",
    "catch2bal_pitch_th_rad",
    "catch2bal_rate_th_rads",
    "bal2catch_pitch_th_rad",
    "catch_hold_ms",
    "catch_u_limit",
    "catch_k_pitch",
    "catch_k_pitch_rate",
    "catch_k_wheel_vel",
    "catch_k_wheel_pos",
    "fall_pitch_pos_th_rad",
    "fall_pitch_neg_th_rad",
    "motion_pitch_bias_per_cmd_rad",
    "motion_cmd_rate_per_s",
    "motion_turn_u_limit",
    "motion_fwd_cmd",
    "motion_turn_cmd",
]


@dataclass
class TelemetryPoint:
    t_ms: int
    en: int
    mode: str
    pitch_mrad: int
    rate_mrads: int
    wheel_vel: int
    u: int


class SerialWorker(threading.Thread):
    def __init__(self, port: str, baudrate: int, rx_queue: queue.Queue[str], stop_event: threading.Event):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.rx_queue = rx_queue
        self.stop_event = stop_event
        self.ser: serial.Serial | None = None

    def run(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.2, write_timeout=0.5)
            self.rx_queue.put(f"__STATUS__:CONNECTED:{self.port}")
        except Exception as exc:
            self.rx_queue.put(f"__STATUS__:ERROR:{type(exc).__name__}: {exc}")
            return

        buffer = bytearray()
        try:
            while not self.stop_event.is_set():
                chunk = self.ser.read(256)
                if not chunk:
                    time.sleep(0.01)
                    continue
                buffer.extend(chunk)
                while b"\n" in buffer:
                    line, _, buffer = buffer.partition(b"\n")
                    text = line.decode("utf-8", errors="replace").strip("\r")
                    if text:
                        self.rx_queue.put(text)
        except Exception as exc:
            self.rx_queue.put(f"__STATUS__:ERROR:{type(exc).__name__}: {exc}")
        finally:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            self.rx_queue.put("__STATUS__:DISCONNECTED")

    def write_line(self, text: str) -> None:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Порт не открыт")
        self.ser.write((text.strip() + "\n").encode("utf-8"))
        self.ser.flush()


class PresetSelectDialog(tk.Toplevel):
    def __init__(self, parent: tk.Tk, preset_files: list[Path]):
        super().__init__(parent)
        self.title(f"Выбор пресета {APP_VERSION}")
        self.geometry("720x420")
        self.result: Path | None = None
        self._preset_files = preset_files

        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        ttk.Label(self, text="Выберите пресет", padding=(10, 10, 10, 6)).grid(row=0, column=0, sticky="w")

        frame = ttk.Frame(self, padding=(10, 0, 10, 10))
        frame.grid(row=1, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self.listbox = tk.Listbox(frame, exportselection=False)
        self.listbox.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.listbox.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.listbox.configure(yscrollcommand=scrollbar.set)

        for p in preset_files:
            self.listbox.insert("end", p.name)

        btns = ttk.Frame(self, padding=(10, 0, 10, 10))
        btns.grid(row=2, column=0, sticky="ew")
        ttk.Button(btns, text="Загрузить", command=self._accept).pack(side="left", padx=(0, 6))
        ttk.Button(btns, text="Отмена", command=self._cancel).pack(side="left")

        self.listbox.bind("<Double-Button-1>", lambda _e: self._accept())
        self.listbox.bind("<Return>", lambda _e: self._accept())

        if preset_files:
            self.listbox.selection_set(0)

        self.transient(parent)
        self.grab_set()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _accept(self) -> None:
        selection = self.listbox.curselection()
        if not selection:
            return
        self.result = self._preset_files[selection[0]]
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class RobotGuiApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(f"Balancing Robot USB CDC Tool {APP_VERSION}")
        self.root.geometry("1900x900")
        self.root.minsize(1500, 820)

        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.ui_queue: queue.Queue[tuple] = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: SerialWorker | None = None

        self.log_lines: list[str] = []
        self.telemetry: list[TelemetryPoint] = []
        self.param_vars: dict[str, tk.StringVar] = {}
        self.last_values: dict[str, float] = {}

        self.presets_dir = Path.cwd() / "robot_presets"
        self.presets_dir.mkdir(parents=True, exist_ok=True)
        self.current_preset_path = self.presets_dir / "current.json"

        self.reconnect_max_attempts = 5
        self.reconnect_attempt = 0
        self.reconnect_after_id = None
        self.reconnect_delays_ms = [0, 100, 200, 300, 500]

        self.ai_busy = False
        self.ai_timeout_s = 60.0
        self.ai_watchdog_after_id = None
        self.net_test_timeout_s = 5.0

        self.current_apply_queue = []
        self.current_apply_index = 0
        self.current_apply_after_id = None
        self.current_apply_delay_ms = 120
        self.command_batch_queue: list[str] = []
        self.command_batch_index = 0
        self.command_batch_after_id = None
        self.command_batch_delay_ms = 120
        self.dbg_since_connect = 0

        self.attempt_active = False
        self.attempt_started_on_dbg = False
        self.awaiting_final_en0_for_attempt = False
        self.last_dbg_en0_line = None
        self.attempt_lines: list[str] = []
        self.last_ai_commands: list[str] = []
        self.attempt_tail_collecting = False
        self.attempt_tail_deadline_monotonic = 0.0
        self.analysis_stop_requested = False

        self._build_ui()
        self._install_clipboard_shortcuts()
        self._log_startup()
        self._refresh_ports()
        self._schedule_poll()

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        left = ttk.Frame(self.root, padding=8, width=780)
        left.grid(row=0, column=0, sticky="nsw")
        left.grid_propagate(False)
        left.columnconfigure(0, weight=1)

        right = ttk.Frame(self.root, padding=8)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=3)
        right.rowconfigure(1, weight=2)

        self._build_left(left)
        self._build_right(right)

    def _build_left(self, left: ttk.Frame) -> None:
        conn = ttk.LabelFrame(left, text="Подключение", padding=8)
        conn.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        conn.columnconfigure(1, weight=1)

        ttk.Label(conn, text="COM:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, state="readonly", width=22)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(4, 4))
        ttk.Button(conn, text="Обновить", command=self._refresh_ports).grid(row=0, column=2)

        ttk.Label(conn, text="Baud:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=1, column=1, sticky="w", padx=(4, 4), pady=(6, 0))

        btns = ttk.Frame(conn)
        btns.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        ttk.Button(btns, text="Подключить", command=self.connect).pack(side="left", padx=(0, 4))
        ttk.Button(btns, text="Отключить", command=self.disconnect).pack(side="left")

        self.status_var = tk.StringVar(value="Не подключено")
        ttk.Label(conn, textvariable=self.status_var).grid(row=3, column=0, columnspan=3, sticky="w", pady=(8, 0))

        self.auto_reconnect_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(conn, text="Автопереподключение (5 попыток)", variable=self.auto_reconnect_var).grid(
            row=4, column=0, columnspan=3, sticky="w", pady=(6, 0)
        )

        cmd = ttk.LabelFrame(left, text="Команды", padding=8)
        cmd.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        cmd.columnconfigure(0, weight=1)
        cmd.rowconfigure(1, weight=1)

        cmd_top = ttk.Frame(cmd)
        cmd_top.grid(row=0, column=0, columnspan=2, sticky="ew")
        cmd_top.columnconfigure(1, weight=1)

        ttk.Label(cmd_top, text="Пауза, мс:").grid(row=0, column=0, sticky="w")
        self.cmd_batch_delay_var = tk.StringVar(value="120")
        ttk.Entry(cmd_top, textvariable=self.cmd_batch_delay_var, width=8).grid(row=0, column=1, sticky="w", padx=(6, 0))
        ttk.Button(cmd_top, text="Отправить пачкой", command=self.send_raw_command).grid(row=0, column=2, padx=(12, 0))
        ttk.Button(cmd_top, text="Очистить", command=self.clear_command_box).grid(row=0, column=3, padx=(6, 0))

        self.cmd_text = tk.Text(cmd, height=6, wrap="word")
        self.cmd_text.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        self.cmd_text.bind("<Control-Return>", lambda _e: self.send_raw_command())

        cmd_scroll = ttk.Scrollbar(cmd, orient="vertical", command=self.cmd_text.yview)
        cmd_scroll.grid(row=1, column=2, sticky="ns", pady=(8, 0))
        self.cmd_text.configure(yscrollcommand=cmd_scroll.set)

        quick = ttk.Frame(cmd)
        quick.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(quick, text="help", command=lambda: self.send_command("help")).pack(side="left", padx=(0, 4))
        ttk.Button(quick, text="get all", command=lambda: self.send_command("get all")).pack(side="left", padx=(0, 4))
        ttk.Button(quick, text="en 1", command=lambda: self.send_command("en 1")).pack(side="left", padx=(0, 4))
        ttk.Button(quick, text="en 0", command=lambda: self.send_command("en 0")).pack(side="left")

        presets = ttk.LabelFrame(left, text="Пресеты", padding=8)
        presets.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        ttk.Button(presets, text="Сохранить пресет", command=self.save_preset_auto).pack(side="left", padx=(0, 4))
        ttk.Button(presets, text="Загрузить пресет", command=self.load_preset_from_list).pack(side="left", padx=(0, 4))
        ttk.Button(presets, text="Открыть папку", command=self.show_presets_folder).pack(side="left")
        self.preset_status_var = tk.StringVar(value=f"Папка пресетов: {self.presets_dir}")
        ttk.Label(presets, textvariable=self.preset_status_var).pack(side="left", padx=(10, 0))

        ai = ttk.LabelFrame(left, text="AI analysis", padding=8)
        ai.grid(row=3, column=0, sticky="ew", pady=(0, 8))
        ai.columnconfigure(1, weight=1)

        ttk.Label(ai, text="API key:").grid(row=0, column=0, sticky="w")
        self.api_key_var = tk.StringVar(value=os.environ.get("OPENAI_API_KEY", ""))
        ttk.Entry(ai, textvariable=self.api_key_var, show="*", width=24).grid(row=0, column=1, columnspan=4, sticky="ew", padx=(4, 4))

        ttk.Label(ai, text="Model:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.ai_model_var = tk.StringVar(value="gpt-5.4")
        ttk.Entry(ai, textvariable=self.ai_model_var, width=12).grid(row=1, column=1, sticky="w", padx=(4, 4), pady=(6, 0))

        ttk.Label(ai, text="Timeout, s:").grid(row=1, column=2, sticky="e", pady=(6, 0))
        self.ai_timeout_var = tk.StringVar(value="60")
        ttk.Entry(ai, textvariable=self.ai_timeout_var, width=8).grid(row=1, column=3, sticky="w", padx=(4, 4), pady=(6, 0))

        ttk.Label(ai, text="Min DBG lines:").grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.min_lines_var = tk.StringVar(value="8")
        ttk.Entry(ai, textvariable=self.min_lines_var, width=8).grid(row=2, column=1, sticky="w", padx=(4, 4), pady=(6, 0))

        ttk.Label(ai, text="Attempt lines:").grid(row=2, column=2, sticky="e", pady=(6, 0))
        self.attempt_info_var = tk.StringVar(value="0")
        ttk.Label(ai, textvariable=self.attempt_info_var).grid(row=2, column=3, sticky="w", pady=(6, 0))

        ttk.Label(ai, text="Attempt active:").grid(row=3, column=0, sticky="w", pady=(6, 0))
        self.attempt_state_var = tk.StringVar(value="no")
        ttk.Label(ai, textvariable=self.attempt_state_var).grid(row=3, column=1, sticky="w", pady=(6, 0))

        self.ai_apply_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ai, text="Автоприменение set-команд", variable=self.ai_apply_var).grid(
            row=3, column=2, columnspan=2, sticky="w", pady=(6, 0)
        )

        self.ai_status_var = tk.StringVar(value="AI: готово")
        ttk.Label(ai, textvariable=self.ai_status_var).grid(row=4, column=0, columnspan=4, sticky="w", pady=(6, 0))

        ai_btns = ttk.Frame(ai)
        ai_btns.grid(row=5, column=0, columnspan=5, sticky="ew", pady=(8, 0))
        ttk.Button(ai_btns, text="Анализ текущей попытки", command=self.start_attempt_analysis).pack(side="left", padx=(0, 4))
        ttk.Button(ai_btns, text="Применить команды", command=self.apply_last_ai_commands).pack(side="left", padx=(0, 4))
        ttk.Button(ai_btns, text="Очистить AI", command=self.clear_ai_panel).pack(side="left", padx=(0, 4))
        ttk.Button(ai_btns, text="Test network (raw)", command=self.start_network_test).pack(side="left", padx=(0, 4))
        ttk.Button(ai_btns, text="Вставить ключ из ENV", command=self.fill_api_key_from_env).pack(side="left")

        params = ttk.LabelFrame(left, text="Параметры", padding=8)
        params.grid(row=4, column=0, sticky="nsew")
        left.rowconfigure(4, weight=1)

        self.param_inner = ttk.Frame(params)
        self.param_inner.pack(fill="both", expand=True)
        self._build_param_rows()

        param_buttons = ttk.Frame(left)
        param_buttons.grid(row=5, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(param_buttons, text="Обновить все из МК", command=lambda: self.send_command("get all")).pack(side="left", padx=(0, 4))
        ttk.Button(param_buttons, text="Применить current.json", command=self.apply_current_json_button).pack(side="left", padx=(0, 4))
        ttk.Button(param_buttons, text="Подтянуть в поля", command=self.fill_params_from_last_values).pack(side="left")


    def _install_clipboard_shortcuts(self) -> None:
        for class_name in ("Entry", "TEntry", "Text", "TCombobox"):
            self.root.bind_class(class_name, "<Control-c>", self._copy_from_focused_widget, add=True)
            self.root.bind_class(class_name, "<Control-C>", self._copy_from_focused_widget, add=True)
            self.root.bind_class(class_name, "<Control-v>", self._paste_to_focused_widget, add=True)
            self.root.bind_class(class_name, "<Control-V>", self._paste_to_focused_widget, add=True)
            self.root.bind_class(class_name, "<Control-x>", self._cut_from_focused_widget, add=True)
            self.root.bind_class(class_name, "<Control-X>", self._cut_from_focused_widget, add=True)
            self.root.bind_class(class_name, "<Control-a>", self._select_all_in_focused_widget, add=True)
            self.root.bind_class(class_name, "<Control-A>", self._select_all_in_focused_widget, add=True)

    def _copy_from_focused_widget(self, _event=None):
        widget = self.root.focus_get()
        if widget is None:
            return "break"
        try:
            widget.event_generate("<<Copy>>")
        except Exception:
            pass
        return "break"

    def _paste_to_focused_widget(self, _event=None):
        widget = self.root.focus_get()
        if widget is None:
            return "break"
        try:
            widget.event_generate("<<Paste>>")
        except Exception:
            pass
        return "break"

    def _cut_from_focused_widget(self, _event=None):
        widget = self.root.focus_get()
        if widget is None:
            return "break"
        try:
            widget.event_generate("<<Cut>>")
        except Exception:
            pass
        return "break"

    def _select_all_in_focused_widget(self, _event=None):
        widget = self.root.focus_get()
        if widget is None:
            return "break"
        try:
            if isinstance(widget, tk.Text):
                widget.tag_add("sel", "1.0", "end-1c")
                widget.mark_set("insert", "end-1c")
                widget.see("insert")
            elif isinstance(widget, tk.Entry):
                widget.select_range(0, "end")
                widget.icursor("end")
            else:
                widget.event_generate("<<SelectAll>>")
        except Exception:
            pass
        return "break"

    def _build_right(self, right: ttk.Frame) -> None:
        log_frame = ttk.LabelFrame(right, text="Лог / ответы МК", padding=8)
        log_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(log_frame, wrap="none", height=24)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self.log_text.bind("<Control-c>", self._on_log_copy)
        self.log_text.bind("<Button-3>", self._show_log_context_menu)

        log_y = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        log_y.grid(row=0, column=1, sticky="ns")
        log_x = ttk.Scrollbar(log_frame, orient="horizontal", command=self.log_text.xview)
        log_x.grid(row=1, column=0, sticky="ew")
        self.log_text.configure(yscrollcommand=log_y.set, xscrollcommand=log_x.set)

        log_btns = ttk.Frame(log_frame)
        log_btns.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(log_btns, text="Очистить лог", command=self.clear_log).pack(side="left", padx=(0, 4))
        ttk.Button(log_btns, text="Сохранить лог", command=self.save_log).pack(side="left")

        ai_frame = ttk.LabelFrame(right, text="AI result", padding=8)
        ai_frame.grid(row=1, column=0, sticky="nsew")
        ai_frame.columnconfigure(0, weight=1)
        ai_frame.rowconfigure(0, weight=1)

        self.ai_text = tk.Text(ai_frame, wrap="word", height=16)
        self.ai_text.grid(row=0, column=0, sticky="nsew")
        ai_y = ttk.Scrollbar(ai_frame, orient="vertical", command=self.ai_text.yview)
        ai_y.grid(row=0, column=1, sticky="ns")
        self.ai_text.configure(yscrollcommand=ai_y.set)

        ai_bottom = ttk.Frame(ai_frame)
        ai_bottom.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(ai_bottom, text="Скопировать AI", command=self.copy_ai_text).pack(side="left")

    def _build_param_rows(self) -> None:
        split = (len(PARAM_NAMES) + 1) // 2
        for idx, name in enumerate(PARAM_NAMES):
            block = 0 if idx < split else 1
            row = idx if idx < split else idx - split
            base_col = block * 3

            ttk.Label(self.param_inner, text=name, width=24).grid(row=row, column=base_col + 0, sticky="w", pady=2, padx=(0, 4))
            var = tk.StringVar()
            self.param_vars[name] = var
            ttk.Entry(self.param_inner, textvariable=var, width=12).grid(row=row, column=base_col + 1, sticky="ew", pady=2, padx=(0, 4))
            ttk.Button(self.param_inner, text="Set", command=lambda n=name: self.set_param(n)).grid(row=row, column=base_col + 2, sticky="ew", pady=2)

        self.param_inner.columnconfigure(1, weight=1)
        self.param_inner.columnconfigure(4, weight=1)

    def _log_startup(self) -> None:
        self._append_log(f"[APP] Balancing Robot USB CDC Tool {APP_VERSION}")
        self._append_log("[APP] Build features: multiline command batch send, Ctrl+C/Ctrl+V in editable fields, pre-en0 + all en1 + post-en0 capture, no auto commands after connect, manual get all/current.json apply only, full AI prompt+payload dump")
        self._append_log(f"[APP] openai available: {OPENAI_AVAILABLE}")
        if OPENAI_AVAILABLE:
            self._append_log(f"[APP] openai package: {getattr(openai_pkg, '__version__', 'unknown')}")

    def _refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        current = self.port_var.get().strip()
        if current and current in ports:
            return
        if "COM10" in ports:
            self.port_var.set("COM10")
        elif ports:
            self.port_var.set(ports[0])

    def connect(self) -> None:
        if self.worker is not None:
            return
        if self.reconnect_after_id is not None:
            try:
                self.root.after_cancel(self.reconnect_after_id)
            except Exception:
                pass
            self.reconnect_after_id = None

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Ошибка", "Выбери COM-порт")
            return

        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Ошибка", "Неверный baudrate")
            return

        self.stop_event.clear()
        self.dbg_since_connect = 0
        self.worker = SerialWorker(port, baud, self.rx_queue, self.stop_event)
        self.worker.start()
        self.status_var.set(f"Подключение к {port}...")

    def disconnect(self) -> None:
        self._cancel_ai_watchdog()
        if self.reconnect_after_id is not None:
            try:
                self.root.after_cancel(self.reconnect_after_id)
            except Exception:
                pass
            self.reconnect_after_id = None

        if self.current_apply_after_id is not None:
            try:
                self.root.after_cancel(self.current_apply_after_id)
            except Exception:
                pass
            self.current_apply_after_id = None

        if self.command_batch_after_id is not None:
            try:
                self.root.after_cancel(self.command_batch_after_id)
            except Exception:
                pass
            self.command_batch_after_id = None

        self.current_apply_queue = []
        self.current_apply_index = 0
        self.command_batch_queue = []
        self.command_batch_index = 0
        self.reconnect_attempt = 0
        if self.worker is not None:
            self.stop_event.set()
            self.worker = None
        self.status_var.set("Отключено")

    def send_command(self, command: str) -> None:
        command = command.strip()
        if not command:
            return
        if self.worker is None:
            messagebox.showerror("Ошибка", "Нет подключения к COM-порту")
            return

        lowered = command.lower()
        if lowered == "en 1":
            self.attempt_lines.clear()
            self.telemetry.clear()
            self.attempt_active = True
            self.attempt_started_on_dbg = False
            self.awaiting_final_en0_for_attempt = False
            self.attempt_tail_collecting = False
            self.attempt_tail_deadline_monotonic = 0.0
            self.analysis_stop_requested = False
            self._update_attempt_state()
            self._append_ai("[AI] Начата новая попытка: ожидание первой DBG строки с en=1")
        elif lowered == "en 0":
            if self.ai_busy:
                self.analysis_stop_requested = True
                self.awaiting_final_en0_for_attempt = True
                self._append_ai("[AI] Ожидание первой завершающей DBG строки с en=0")
            else:
                self.attempt_active = False
            self._update_attempt_state()

        try:
            self.worker.write_line(command)
            self._append_log(f">>> {command}")
        except Exception as exc:
            self._append_log(f"TX ERROR: {type(exc).__name__}: {exc}")

    def send_raw_command(self) -> None:
        text = self.cmd_text.get("1.0", "end-1c")
        commands = [line.strip() for line in text.splitlines() if line.strip()]
        if not commands:
            return

        try:
            delay_ms = int(self.cmd_batch_delay_var.get().strip())
        except ValueError:
            messagebox.showerror("Ошибка", "Неверная пауза между командами")
            return

        if delay_ms < 0:
            delay_ms = 0

        self.command_batch_delay_ms = delay_ms
        self.command_batch_queue = commands
        self.command_batch_index = 0

        if self.command_batch_after_id is not None:
            try:
                self.root.after_cancel(self.command_batch_after_id)
            except Exception:
                pass
            self.command_batch_after_id = None

        self._append_log(f"[CMD] queued commands: {len(commands)}, delay_ms={self.command_batch_delay_ms}")
        self._send_next_batch_command()

    def _send_next_batch_command(self) -> None:
        self.command_batch_after_id = None
        if self.worker is None:
            self._append_log("[CMD] batch stopped: no connection")
            self.command_batch_queue = []
            self.command_batch_index = 0
            return

        if self.command_batch_index >= len(self.command_batch_queue):
            self._append_log(f"[CMD] batch done, sent: {self.command_batch_index}")
            return

        command = self.command_batch_queue[self.command_batch_index]
        self.command_batch_index += 1
        self.send_command(command)

        if self.command_batch_index < len(self.command_batch_queue):
            self.command_batch_after_id = self.root.after(self.command_batch_delay_ms, self._send_next_batch_command)
        else:
            self._append_log(f"[CMD] batch done, sent: {self.command_batch_index}")

    def clear_command_box(self) -> None:
        self.cmd_text.delete("1.0", "end")

    def set_param(self, name: str) -> None:
        value = self.param_vars[name].get().strip()
        if not value:
            messagebox.showerror("Ошибка", f"Пустое значение для {name}")
            return
        self.send_command(f"set {name} {value}")
        self.save_current_json()


    def save_current_json(self) -> None:
        data = self._collect_preset_data()
        payload = {
            "app_version": APP_VERSION,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "parameters": data,
        }
        self.current_preset_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self._append_log(f"[PRESET] current.json saved: {self.current_preset_path}")

    def apply_current_json_button(self) -> None:
        self._append_log("[PRESET] manual action: apply current.json")
        self.apply_current_json_to_controller()

    def apply_current_json_to_controller(self) -> None:
        if self.worker is None:
            return
        if not self.current_preset_path.exists():
            self._append_log("[PRESET] current.json not found")
            return

        if not hasattr(self, "current_apply_queue"):
            self.current_apply_queue = []
        if not hasattr(self, "current_apply_index"):
            self.current_apply_index = 0
        if not hasattr(self, "current_apply_after_id"):
            self.current_apply_after_id = None
        if not hasattr(self, "current_apply_delay_ms"):
            self.current_apply_delay_ms = 350
        if not hasattr(self, "dbg_since_connect"):
            self.dbg_since_connect = 0


        try:
            payload = json.loads(self.current_preset_path.read_text(encoding="utf-8"))
            params = payload.get("parameters", {})
            if not isinstance(params, dict):
                self._append_log("[PRESET] current.json has no parameters block")
                return

            self.current_apply_queue = []
            self.current_apply_index = 0

            for name, value in params.items():
                if name in self.param_vars:
                    self._update_param_field_from_value(name, float(value))
                    self.current_apply_queue.append((name, value))

            if not self.current_apply_queue:
                self._append_log("[PRESET] current.json empty")
                return

            if self.current_apply_after_id is not None:
                try:
                    self.root.after_cancel(self.current_apply_after_id)
                except Exception:
                    pass
                self.current_apply_after_id = None

            self._append_log(f"[PRESET] current.json queued manually, commands: {len(self.current_apply_queue)}, delay_ms={self.current_apply_delay_ms}")
            self._schedule_next_current_apply()
        except Exception as exc:
            self._append_log(f"[PRESET] current.json apply error: {type(exc).__name__}: {exc}")

    def _schedule_next_current_apply(self) -> None:
        if self.current_apply_index >= len(self.current_apply_queue):
            self._append_log(f"[PRESET] current.json applied, commands sent: {self.current_apply_index}")
            self.current_apply_after_id = None
            return

        self.current_apply_after_id = self.root.after(self.current_apply_delay_ms, self._apply_next_current_item)

    def _apply_next_current_item(self) -> None:
        self.current_apply_after_id = None
        if self.worker is None:
            self._append_log("[PRESET] current.json apply stopped: no connection")
            self.current_apply_queue = []
            self.current_apply_index = 0
            return

        if self.current_apply_index >= len(self.current_apply_queue):
            self._append_log(f"[PRESET] current.json applied, commands sent: {self.current_apply_index}")
            return

        name, value = self.current_apply_queue[self.current_apply_index]
        self.current_apply_index += 1
        self._append_log(f"[PRESET] apply {self.current_apply_index}/{len(self.current_apply_queue)}: set {name} {value}")
        self.send_command(f"set {name} {value}")
        self._schedule_next_current_apply()

    def fill_api_key_from_env(self) -> None:
        self.api_key_var.set(os.environ.get("OPENAI_API_KEY", ""))

    def start_attempt_analysis(self) -> None:
        if not OPENAI_AVAILABLE:
            messagebox.showerror("Ошибка", "Пакет openai не установлен")
            return
        if self.ai_busy:
            return
        if self.worker is None:
            messagebox.showerror("Ошибка", "Нет подключения к COM-порту")
            return

        api_key = self.api_key_var.get().strip()
        if not api_key:
            messagebox.showerror("Ошибка", "Укажи OPENAI API key")
            return

        try:
            timeout_s = max(3.0, float(self.ai_timeout_var.get().strip()))
            min_lines = max(1, int(self.min_lines_var.get().strip()))
        except ValueError:
            messagebox.showerror("Ошибка", "Неверные параметры AI")
            return

        self.ai_timeout_s = timeout_s
        self.ai_busy = True
        self.ui_queue.put(("ai_clear",))
        self.ui_queue.put(("ai_append", f"[AI] {APP_VERSION}"))
        self.ui_queue.put(("ai_append", "[AI] Останов попытки: отправка en 0"))
        self.ai_status_var.set("AI: остановка попытки...")
        self.send_command("en 0")
        self._start_ai_watchdog(timeout_s + 8.0, "attempt_analysis")
        threading.Thread(target=self._run_attempt_analysis, args=(api_key, min_lines), daemon=True).start()

    def _run_attempt_analysis(self, api_key: str, min_lines: int) -> None:
        try:
            deadline = time.monotonic() + 5.0
            while self.awaiting_final_en0_for_attempt and time.monotonic() < deadline:
                time.sleep(0.05)

            self.ui_queue.put(("send_command", "get all"))
            time.sleep(1.0)

            dbg_count = len(self.attempt_lines)
            self.ui_queue.put(("ai_append", f"[AI] DBG lines sent to AI: {dbg_count}"))

            if dbg_count < min_lines:
                raise RuntimeError(
                    f"Попытка слишком короткая для анализа: {dbg_count} DBG строк(и). "
                    f"Нужно минимум {min_lines}. Запусти новую попытку через en 1."
                )

            result = self._request_attempt_analysis(api_key)
            self.ui_queue.put(("ai_finish_ok", result))
        except Exception as exc:
            self.ui_queue.put(("ai_finish_error", exc))

    def start_network_test(self) -> None:
        if self.ai_busy:
            return
        try:
            timeout_s = max(3.0, float(self.ai_timeout_var.get().strip()))
        except ValueError:
            messagebox.showerror("Ошибка", "Неверное значение Timeout, s")
            return

        self.net_test_timeout_s = timeout_s
        self.ai_busy = True
        self.ui_queue.put(("ai_append", f"[NET] Старт raw network test, timeout={timeout_s:.1f}s"))
        self.ai_status_var.set("NET: test...")
        self._start_ai_watchdog(timeout_s, "network")
        threading.Thread(target=self._run_network_test, daemon=True).start()

    def _run_network_test(self) -> None:
        try:
            result = self._request_network_test()
            self.ui_queue.put(("net_finish_ok", result))
        except Exception as exc:
            self.ui_queue.put(("ai_finish_error", exc))

    def _collect_dbg_lines_for_ai(self) -> list[str]:
        return list(self.attempt_lines)

    def _collect_forced_param_snapshot(self) -> dict[str, float | int]:
        data: dict[str, float | int] = {}
        for name in PARAM_NAMES:
            if name in self.last_values:
                value = self.last_values[name]
                data[name] = int(value) if float(value).is_integer() else value
        return data

    def _build_attempt_summary(self) -> dict:
        if not self.telemetry:
            return {"lines": len(self.attempt_lines), "telemetry_points": 0}

        pitch_vals = [x.pitch_mrad for x in self.telemetry]
        rate_vals = [x.rate_mrads for x in self.telemetry]
        modes = []
        for x in self.telemetry:
            if x.mode not in modes:
                modes.append(x.mode)

        return {
            "lines": len(self.attempt_lines),
            "telemetry_points": len(self.telemetry),
            "t_start_ms": self.telemetry[0].t_ms,
            "t_end_ms": self.telemetry[-1].t_ms,
            "pitch_min_mrad": min(pitch_vals),
            "pitch_max_mrad": max(pitch_vals),
            "rate_min_mrads": min(rate_vals),
            "rate_max_mrads": max(rate_vals),
            "modes_seen": modes,
        }

    def _request_attempt_analysis(self, api_key: str) -> dict:
        client = OpenAI(api_key=api_key, timeout=self.ai_timeout_s, max_retries=0)

        params = self._collect_forced_param_snapshot()
        dbg_lines = self._collect_dbg_lines_for_ai()
        summary = self._build_attempt_summary()

        if not dbg_lines:
            raise RuntimeError("Буфер текущей попытки пуст. Сначала выполни попытку после en 1.")

        payload = {
            "parameters": params,
            "attempt_summary": summary,
            "attempt_dbg_lines": dbg_lines,
        }
        payload_text = json.dumps(payload, ensure_ascii=False, indent=2)

        self.ui_queue.put(("ai_append", f"[AI] Параметров из контроллера: {len(params)}"))
        self.ui_queue.put(("ai_append", f"[AI] Строк попытки: {len(dbg_lines)}"))
        self.ui_queue.put(("ai_append", "[AI] Отправка анализа текущей попытки..."))
        self.ui_queue.put(("ai_append", "[AI] ----- PAYLOAD TO AI BEGIN -----"))
        self.ui_queue.put(("ai_append", payload_text))
        self.ui_queue.put(("ai_append", "[AI] ----- PAYLOAD TO AI END -----"))

        instructions_text = (
            "Ты анализируешь одну попытку балансировки STM32/ODrive. "
            "Во входе всегда есть полный набор текущих коэффициентов balance/catch и лог только текущей попытки. "
            "Верни строго JSON с полями: summary, risks, recommendations, commands. "
            "summary: короткий вывод на русском. "
            "risks: список коротких строк. "
            "recommendations: список объектов {param, value, reason}. "
            "commands: список из 1-3 строк вида 'set name value'. "
            "Если нет уверенности, всё равно верни хотя бы 1 осторожную команду set. "
            "Используй только имена параметров, уже присутствующие во входных данных."
        )
        self.ui_queue.put(("ai_append", "[AI] ----- PROMPT TO AI BEGIN -----"))
        self.ui_queue.put(("ai_append", instructions_text))
        self.ui_queue.put(("ai_append", "[AI] ----- PROMPT TO AI END -----"))

        response = client.responses.create(
            model=self.ai_model_var.get().strip() or "gpt-5.4",
            instructions=instructions_text,
            input=payload_text,
        )

        text = response.output_text
        if not text:
            raise RuntimeError("AI не вернул текст ответа")
        self.ui_queue.put(("ai_append", "[AI] ----- RAW RESPONSE BEGIN -----"))
        self.ui_queue.put(("ai_append", text))
        self.ui_queue.put(("ai_append", "[AI] ----- RAW RESPONSE END -----"))
        return json.loads(text)

    def _request_network_test(self) -> dict:
        timeout = self.net_test_timeout_s
        checks = []
        targets = [
            ("api.openai.com", "https://api.openai.com/v1/models"),
            ("platform.openai.com", "https://platform.openai.com/"),
        ]

        for name, url in targets:
            started = time.time()
            req = urllib.request.Request(
                url,
                headers={
                    "User-Agent": f"robot-serial-gui/{APP_VERSION}",
                    "Accept": "application/json,text/plain,*/*",
                },
                method="GET",
            )
            try:
                with urllib.request.urlopen(req, timeout=timeout, context=ssl.create_default_context()) as resp:
                    elapsed = time.time() - started
                    checks.append({"target": name, "ok": True, "status": getattr(resp, "status", None), "elapsed_s": round(elapsed, 3)})
            except urllib.error.HTTPError as exc:
                elapsed = time.time() - started
                checks.append({"target": name, "ok": True, "status": exc.code, "elapsed_s": round(elapsed, 3), "note": "HTTPError received, network path exists"})
            except Exception as exc:
                elapsed = time.time() - started
                checks.append({"target": name, "ok": False, "elapsed_s": round(elapsed, 3), "error": f"{type(exc).__name__}: {exc}"})

        return {"ok": any(item.get("ok") for item in checks), "checks": checks}

    def apply_last_ai_commands(self) -> None:
        if not self.last_ai_commands:
            messagebox.showinfo("AI", "Нет команд для применения")
            return
        if self.worker is None:
            messagebox.showerror("Ошибка", "Нет подключения к COM-порту")
            return

        sent = 0
        for cmd in self.last_ai_commands:
            cmd = cmd.strip()
            if cmd.lower().startswith("set "):
                self.send_command(cmd)
                sent += 1
        if sent > 0:
            self.save_current_json()
        self._append_ai(f"[AI] Применено команд: {sent}")

    def clear_ai_panel(self) -> None:
        self.ai_text.delete("1.0", "end")
        self.last_ai_commands = []

    def copy_ai_text(self) -> None:
        text = self.ai_text.get("1.0", "end-1c")
        self.root.clipboard_clear()
        self.root.clipboard_append(text)

    def _start_ai_watchdog(self, timeout_s: float, kind: str) -> None:
        self._cancel_ai_watchdog()
        delay_ms = int(max(1000.0, (timeout_s + 1.0) * 1000.0))
        self.ai_watchdog_after_id = self.root.after(delay_ms, lambda: self._on_ai_watchdog_timeout(kind, timeout_s))

    def _cancel_ai_watchdog(self) -> None:
        if self.ai_watchdog_after_id is not None:
            try:
                self.root.after_cancel(self.ai_watchdog_after_id)
            except Exception:
                pass
            self.ai_watchdog_after_id = None

    def _on_ai_watchdog_timeout(self, kind: str, timeout_s: float) -> None:
        self.ai_watchdog_after_id = None
        if not self.ai_busy:
            return
        self.ai_busy = False
        self.ai_status_var.set("AI/NET: watchdog timeout")
        self._append_ai(f"[AI] Watchdog timeout: {kind}, ожидание превысило {timeout_s:.1f}s")
        self._append_ai("[AI] Принудительный выход из ожидания; проверь сеть, billing, openai package и model")

    def _finish_ai_analysis_ok(self, result: dict) -> None:
        self._cancel_ai_watchdog()
        self.ai_busy = False
        self.ai_status_var.set("AI: готово")

        summary = result.get("summary", "")
        risks = result.get("risks", [])
        recommendations = result.get("recommendations", [])
        commands = result.get("commands", [])

        self.last_ai_commands = []
        for cmd in commands:
            cmd_str = str(cmd).strip()
            if cmd_str.lower().startswith("set "):
                self.last_ai_commands.append(cmd_str)

        self._append_ai("[AI] Анализ завершён")
        if summary:
            self._append_ai("Summary:\n" + str(summary))
        if risks:
            self._append_ai("Risks:\n" + "\n".join(f"- {x}" for x in risks))
        if recommendations:
            lines = []
            for item in recommendations:
                if isinstance(item, dict):
                    lines.append(f"- {item.get('param', '?')} -> {item.get('value', '?')} : {item.get('reason', '')}")
                else:
                    lines.append(f"- {item}")
            self._append_ai("Recommendations:\n" + "\n".join(lines))

        self._append_ai("Commands:")
        if self.last_ai_commands:
            for cmd in self.last_ai_commands:
                self._append_ai(cmd)
        else:
            self._append_ai("<empty>")

        if self.ai_apply_var.get():
            self.apply_last_ai_commands()

    def _finish_network_test_ok(self, result: dict) -> None:
        self._cancel_ai_watchdog()
        self.ai_busy = False
        self.ai_status_var.set("NET: done")
        self._append_ai(f"[NET] ok: {result.get('ok', False)}")
        for item in result.get("checks", []):
            line = f"{item.get('target')}: ok={item.get('ok')} elapsed={item.get('elapsed_s')}s"
            if "status" in item:
                line += f" status={item.get('status')}"
            if "note" in item:
                line += f" note={item.get('note')}"
            if "error" in item:
                line += f" error={item.get('error')}"
            self._append_ai(line)

    def _finish_ai_error(self, exc: Exception) -> None:
        self._cancel_ai_watchdog()
        self.ai_busy = False
        self.ai_status_var.set("AI/NET: ошибка")
        err_text = f"{type(exc).__name__}: {exc}"
        self._append_ai(f"[AI] Ошибка: {err_text}")

    def _schedule_poll(self) -> None:
        self.root.after(20, self._poll_rx)

    def _poll_rx(self) -> None:
        try:
            while True:
                try:
                    line = self.rx_queue.get_nowait()
                except queue.Empty:
                    break
                self._handle_rx_line(line)

            while True:
                try:
                    event = self.ui_queue.get_nowait()
                except queue.Empty:
                    break
                self._handle_ui_event(event)
        except Exception as exc:
            try:
                self._append_log(f"[GUI] poll error: {type(exc).__name__}: {exc}")
            except Exception:
                pass
        finally:
            self._schedule_poll()

    def _handle_ui_event(self, event: tuple) -> None:
        kind = event[0]
        if kind == "ai_append":
            self._append_ai(str(event[1]))
        elif kind == "ai_clear":
            self.clear_ai_panel()
        elif kind == "send_command":
            self.send_command(str(event[1]))
        elif kind == "ai_finish_ok":
            self._finish_ai_analysis_ok(event[1])
        elif kind == "ai_finish_error":
            self._finish_ai_error(event[1])
        elif kind == "net_finish_ok":
            self._finish_network_test_ok(event[1])

    def _handle_rx_line(self, line: str) -> None:
        if line.startswith("__STATUS__:"):
            self._handle_status(line)
            return

        self._append_log(line)

        dbg_match = DBG_PATTERN.search(line)
        if dbg_match:
            self.dbg_since_connect = getattr(self, "dbg_since_connect", 0) + 1
            point = TelemetryPoint(
                t_ms=int(dbg_match.group("t")),
                en=int(dbg_match.group("en")),
                mode=str(dbg_match.group("mode")),
                pitch_mrad=int(dbg_match.group("pitch")),
                rate_mrads=int(dbg_match.group("rate")),
                wheel_vel=int(dbg_match.group("wv")),
                u=int(dbg_match.group("u")),
            )

            if point.en == 0:
                self.last_dbg_en0_line = line

            # 1) Старт попытки: первая реально пришедшая DBG строка с en=1.
            if self.attempt_active and point.en == 1:
                if not self.attempt_started_on_dbg:
                    self.attempt_started_on_dbg = True
                    if self.last_dbg_en0_line:
                        self.attempt_lines.append(self.last_dbg_en0_line)
                    self._append_ai("[AI] Старт попытки зафиксирован по первой DBG строке с en=1")

                self.attempt_lines.append(line)
                self.telemetry.append(point)
                self._update_attempt_state()

            # 2) Завершение попытки: только первая DBG строка с en=0 после запроса анализа.
            elif self.awaiting_final_en0_for_attempt and self.attempt_started_on_dbg and point.en == 0:
                self.attempt_lines.append(line)
                self.telemetry.append(point)
                self.awaiting_final_en0_for_attempt = False
                self.analysis_stop_requested = False
                self.attempt_active = False
                self._append_ai(f"[AI] Завершающая DBG строка с en=0 зафиксирована, итоговых строк: {len(self.attempt_lines)}")
                self._update_attempt_state()

        for match in KV_PATTERN.finditer(line):
            key = match.group("key")
            value = match.group("value")
            try:
                value_float = float(value)
            except ValueError:
                continue
            self.last_values[key] = value_float
            self._update_param_field_from_value(key, value_float)

    def _handle_status(self, line: str) -> None:
        if line.startswith("__STATUS__:CONNECTED:"):
            port = line.split(":", 2)[2]
            self.status_var.set(f"Подключено: {port}")
            self._append_log(f"[STATUS] Подключено к {port}")
            self.reconnect_attempt = 0
            if self.reconnect_after_id is not None:
                try:
                    self.root.after_cancel(self.reconnect_after_id)
                except Exception:
                    pass
                self.reconnect_after_id = None
        elif line.startswith("__STATUS__:ERROR:"):
            text = line.split(":", 2)[2]
            self.status_var.set(f"Ошибка: {text}")
            self._append_log(f"[STATUS] Ошибка: {text}")
            self.worker = None
            self._schedule_auto_reconnect()
        elif line == "__STATUS__:DISCONNECTED":
            self.status_var.set("Отключено")
            self._append_log("[STATUS] Порт отключён")
            self.worker = None
            self._schedule_auto_reconnect()

    def _schedule_auto_reconnect(self) -> None:
        if not self.auto_reconnect_var.get():
            return
        if self.worker is not None:
            return
        if self.reconnect_attempt >= self.reconnect_max_attempts:
            self._append_log("[STATUS] Автопереподключение: попытки исчерпаны")
            self.status_var.set("Связь потеряна: попытки исчерпаны")
            return
        if self.reconnect_after_id is not None:
            return

        self.reconnect_attempt += 1
        idx = min(self.reconnect_attempt - 1, len(self.reconnect_delays_ms) - 1)
        delay_ms = int(self.reconnect_delays_ms[idx])

        self.status_var.set(f"Связь потеряна, переподключение {self.reconnect_attempt}/{self.reconnect_max_attempts}...")
        self._append_log(f"[STATUS] Автопереподключение {self.reconnect_attempt}/{self.reconnect_max_attempts} через {delay_ms} мс")

        if delay_ms <= 0:
            self._auto_reconnect_now()
        else:
            self.reconnect_after_id = self.root.after(delay_ms, self._auto_reconnect_now)

    def _auto_reconnect_now(self) -> None:
        self.reconnect_after_id = None
        if self.worker is not None:
            return
        try:
            self._append_log("[STATUS] Попытка автопереподключения...")
            self.connect()
        except Exception as exc:
            self._append_log(f"[STATUS] Ошибка автопереподключения: {type(exc).__name__}: {exc}")
            self._schedule_auto_reconnect()

    def _append_log(self, line: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        full = f"[{timestamp}] {line}"
        self.log_lines.append(full)
        self.log_text.insert("end", full + "\n")
        self.log_text.see("end")

    def _append_ai(self, line: str) -> None:
        self.ai_text.insert("end", line + "\n")
        self.ai_text.see("end")

    def _on_log_copy(self, _event=None):
        try:
            text = self.log_text.get("sel.first", "sel.last")
        except Exception:
            return "break"
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        return "break"

    def _show_log_context_menu(self, event):
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label="Копировать", command=self._copy_log_selection)
        menu.add_command(label="Копировать всё", command=self._copy_all_log)
        menu.tk_popup(event.x_root, event.y_root)

    def _copy_log_selection(self):
        try:
            text = self.log_text.get("sel.first", "sel.last")
        except Exception:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(text)

    def _copy_all_log(self):
        text = self.log_text.get("1.0", "end-1c")
        self.root.clipboard_clear()
        self.root.clipboard_append(text)

    def clear_log(self) -> None:
        self.log_lines.clear()
        self.log_text.delete("1.0", "end")
        self._log_startup()

    def save_log(self) -> None:
        path = filedialog.asksaveasfilename(title="Сохранить лог", defaultextension=".txt", filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not path:
            return
        Path(path).write_text("\n".join(self.log_lines), encoding="utf-8")
        messagebox.showinfo("Готово", "Лог сохранён")

    def _update_param_field_from_value(self, name: str, value: float) -> None:
        var = self.param_vars.get(name)
        if var is None:
            return
        if float(value).is_integer():
            var.set(str(int(value)))
        else:
            var.set(f"{value:.6f}")

    def fill_params_from_last_values(self) -> None:
        count = 0
        for name, value in self.last_values.items():
            if name in self.param_vars:
                self._update_param_field_from_value(name, value)
                count += 1
        messagebox.showinfo("Параметры", f"Обновлено полей: {count}")

    def _collect_preset_data(self) -> dict[str, float | int]:
        data: dict[str, float | int] = {}
        for name, var in self.param_vars.items():
            raw = var.get().strip()
            if not raw:
                continue
            try:
                value = float(raw)
            except ValueError:
                continue
            data[name] = int(value) if value.is_integer() else value
        return data

    def save_preset_auto(self) -> None:
        data = self._collect_preset_data()
        if not data:
            messagebox.showerror("Ошибка", "Нет параметров для сохранения")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.presets_dir / f"preset_{stamp}.json"
        payload = {
            "app_version": APP_VERSION,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "port": self.port_var.get().strip(),
            "parameters": data,
        }
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self.preset_status_var.set(f"Последний пресет: {path.name}")
        self._append_log(f"[PRESET] Сохранён {path}")
        messagebox.showinfo("Пресет сохранён", str(path))

    def load_preset_from_list(self) -> None:
        preset_files = sorted(self.presets_dir.glob("preset_*.json"), reverse=True)
        if not preset_files:
            messagebox.showinfo("Пресеты", f"В папке нет пресетов:\n{self.presets_dir}")
            return
        dialog = PresetSelectDialog(self.root, preset_files)
        self.root.wait_window(dialog)
        if dialog.result is None:
            return
        self._load_preset_file(dialog.result)

    def _load_preset_file(self, path: Path) -> None:
        payload = json.loads(path.read_text(encoding="utf-8"))
        params = payload.get("parameters", {})
        if not isinstance(params, dict):
            messagebox.showerror("Ошибка", "В пресете нет блока parameters")
            return

        updated = 0
        sent = 0
        for name, value in params.items():
            if name in self.param_vars:
                self._update_param_field_from_value(name, float(value))
                updated += 1
                if self.worker is not None:
                    self.send_command(f"set {name} {value}")
                    sent += 1

        self.preset_status_var.set(f"Загружен пресет: {path.name}")
        self._append_log(f"[PRESET] Загружен {path}")
        messagebox.showinfo("Пресет загружен", f"Файл: {path.name}\nОбновлено полей: {updated}\nОтправлено команд в МК: {sent}")

    def show_presets_folder(self) -> None:
        messagebox.showinfo("Папка пресетов", str(self.presets_dir))

    def _update_attempt_state(self) -> None:
        active = self.attempt_active or self.awaiting_final_en0_for_attempt
        self.attempt_state_var.set("yes" if active else "no")
        self.attempt_info_var.set(str(len(self.attempt_lines)))


def main() -> None:
    root = tk.Tk()
    RobotGuiApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()

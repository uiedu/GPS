#!/usr/bin/env python3
"""
navigate_gui.py
Generated using Claude AI (https://claude.ai) wit multiple prompts to create a complete Tkinter GUI for live field navigation with an RTK-capable GNSS receiver.
Tkinter GUI for live field navigation with an RTK-capable GNSS receiver.


Install PyGPSClient in Raspberry Pi OS / Debian / Ubuntu:
https://pypi.org/project/pygpsclient/ 
sudo apt update 
sudo apt install -y python3-pip python3-venv python3-tk 
python3 -m venv ~/pygpsclient 
source ~/pygpsclient/bin/activate 
python3 -m pip install --upgrade pip 
python3 -m pip install pygpsclient

Add the venv's bin to your PATH in ~/.bashrc if you want to launch it without activating each time:

echo 'export PATH="$HOME/pygpsclient/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

Navigation Points
Use a text file with the following format to define waypoints. Each line should contain a waypoint name, latitude, and longitude, separated by commas. Lines starting with '#' are treated as comments and ignored.

# name, latitude, longitude
Beacon1, 46.731100, -117.180500
Beacon2, 46.731400, -117.180800
Beacon3, 46.731050, -117.181200

Lines starting with # are comments. Once you arrive within the threshold of one waypoint, it automatically advances to the next and tells you the new bearing/distance.

Run:
    python3 navigate_gui.py



Features:
  - Connect to a serial GNSS receiver (with optional NTRIP RTK corrections
    for centimeter accuracy).
  - Load a text/CSV waypoint file (name,lat,lon per line).
  - Live compass display + distance readout showing direction to the
    current target waypoint.
  - Local top-down map showing your current position and all waypoints.
  - Auto-advances to the next waypoint on arrival; double-click a waypoint
    in the list to jump to it manually.

Requires: pynmeagps, pyserial, pygnssutils (all installed alongside
pygpsclient), tkinter (usually bundled with Python; on Debian/Raspberry Pi
OS install with: sudo apt install python3-tk)

Free NTRIP Client for Moscow area
Server: rtk2go.com
Port: 2101
Mountpoint: FN-PAL
User: your email 
Password: none or leave blank

To set up GPS-Client Go to https://www.ardusimple.com/how-to-configure-ublox-zed-f9p/#update-firmware



Added navigate-gui.py a python scirpt for Raspberry pi to navigate to a point and also uses optional NTRIP


"""

import csv
import json
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

import serial
import serial.tools.list_ports
from pynmeagps import NMEAReader, NMEAMessage

try:
    from pygnssutils import GNSSNTRIPClient
except ImportError:
    try:
        from pygnssutils.gnssntripclient import GNSSNTRIPClient
    except ImportError:
        GNSSNTRIPClient = None

EARTH_RADIUS_M = 6371000.0

CONFIG_PATH = os.path.expanduser("~/.navigate_gui_settings.json")

FIX_QUALITY_LABELS = {
    0: "NO FIX", 1: "GPS", 2: "DGPS", 4: "RTK FIXED", 5: "RTK FLOAT", 6: "DR",
}
FIX_QUALITY_COLORS = {
    0: "#c0392b", 1: "#e67e22", 2: "#e67e22",
    4: "#27ae60", 5: "#f1c40f", 6: "#8e44ad",
}


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------

def haversine_distance_bearing(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance_m = EARTH_RADIUS_M * c

    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(
        dlambda
    )
    bearing_deg = (math.degrees(math.atan2(y, x)) + 360) % 360
    return distance_m, bearing_deg


def bearing_to_compass(bearing_deg):
    dirs = [
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
    ]
    idx = int((bearing_deg + 11.25) // 22.5) % 16
    return dirs[idx]


def local_enu(lat0, lon0, lat, lon):
    """Approximate east/north offset in meters of (lat,lon) from (lat0,lon0)."""
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    north = dlat * EARTH_RADIUS_M
    east = dlon * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return east, north


def load_waypoints(filepath):
    waypoints = []
    with open(filepath, "r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            row = [c.strip() for c in row]
            if not row or not row[0] or row[0].startswith("#"):
                continue
            if len(row) < 3:
                continue
            name, lat_s, lon_s = row[0], row[1], row[2]
            try:
                lat, lon = float(lat_s), float(lon_s)
            except ValueError:
                continue
            waypoints.append({"name": name, "lat": lat, "lon": lon})
    return waypoints


# --------------------------------------------------------------------------
# Thread-safe GNSS state (used by NTRIP client for live GGA position)
# --------------------------------------------------------------------------

class GnssState:
    def __init__(self):
        self._lock = threading.Lock()
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.sep = 0.0

    def update(self, lat, lon, alt=None, sep=None):
        with self._lock:
            self.lat = lat
            self.lon = lon
            if alt is not None:
                self.alt = alt
            if sep is not None:
                self.sep = sep

    def get_coordinates(self):
        with self._lock:
            return {"lat": self.lat, "lon": self.lon, "alt": self.alt, "sep": self.sep}


# --------------------------------------------------------------------------
# Serial reader thread - pushes parsed fixes into a queue for the GUI thread
# --------------------------------------------------------------------------

class SerialReaderThread(threading.Thread):
    def __init__(self, stream, out_queue, gnss_state, stop_event):
        super().__init__(daemon=True)
        self.stream = stream
        self.out_queue = out_queue
        self.gnss_state = gnss_state
        self.stop_event = stop_event

    def run(self):
        try:
            nmr = NMEAReader(self.stream)
            for (_raw, parsed) in nmr:
                if self.stop_event.is_set():
                    break
                if parsed is None or not isinstance(parsed, NMEAMessage):
                    continue
                msg_id = parsed.identity
                if msg_id not in ("GNGGA", "GPGGA", "GNRMC", "GPRMC"):
                    continue

                lat = getattr(parsed, "lat", None)
                lon = getattr(parsed, "lon", None)
                if lat in (None, "") or lon in (None, ""):
                    continue

                quality = 0
                hdop = None
                numsv = None
                if msg_id.endswith("GGA"):
                    quality = getattr(parsed, "quality", 0) or 0
                    alt = getattr(parsed, "alt", None)
                    sep = getattr(parsed, "sep", None)
                    hdop = getattr(parsed, "HDOP", None)
                    numsv = getattr(parsed, "numSV", None)
                    self.gnss_state.update(lat, lon, alt=alt, sep=sep)
                else:
                    self.gnss_state.update(lat, lon)

                self.out_queue.put(
                    {"lat": lat, "lon": lon, "quality": quality, "hdop": hdop, "numsv": numsv}
                )
        except Exception as e:
            raw_msg = str(e)
            # A physically unplugged USB GNSS receiver often surfaces as a
            # low-level pyserial/OS error (e.g. "'NoneType' object cannot be
            # interpreted as an integer", "device reports readiness to read
            # but returned no data", or a bare OSError/SerialException) once
            # the underlying file descriptor disappears mid-read. Detect
            # these and report something the user can actually act on.
            disconnect_signatures = (
                "nonetype",
                "device reports readiness",
                "input/output error",
                "errno 5",
                "errno 6",
                "no such device",
            )
            is_probable_disconnect = (
                isinstance(e, (OSError, serial.SerialException, TypeError))
                or any(sig in raw_msg.lower() for sig in disconnect_signatures)
            )
            if is_probable_disconnect:
                friendly = (
                    "GNSS receiver disconnected (USB/serial connection lost). "
                    "Check the cable/port and press Connect again."
                )
            else:
                friendly = raw_msg
            self.out_queue.put({"error": friendly, "raw_error": raw_msg})


# --------------------------------------------------------------------------
# Main GUI application
# --------------------------------------------------------------------------

class NavigateApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Field Navigator")
        self._fit_to_screen()

        self.stream = None
        self.reader_thread = None
        self.ntrip_client = None
        self.stop_event = threading.Event()
        self.fix_queue = queue.Queue()
        self.gnss_state = GnssState()

        self.waypoints = []
        self.target_index = 0
        self.current_lat = None
        self.current_lon = None
        self.current_quality = 0
        self.current_hdop = None
        self.current_numsv = None
        self._arrived_index = None

        self.waypoint_filepath = None
        self.last_rtcm_time = None
        self.rtcm_byte_count = 0
        self._last_ntrip_status = None

        self.connect_btn = None
        self.port_var = tk.StringVar(value=self._guess_port())
        self.baud_var = tk.StringVar(value="38400")
        self.ntrip_enabled = tk.BooleanVar(value=False)
        self.ntrip_vars = {
            "server": tk.StringVar(value=""),
            "port": tk.StringVar(value="2101"),
            "mountpoint": tk.StringVar(value=""),
            "user": tk.StringVar(value="anon"),
            "password": tk.StringVar(value="password"),
        }
        self._setup_win = None

        self._build_widgets()
        self._load_settings()
        self._log("Application started")
        self.after(200, self._poll_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- UI construction ----

    def _fit_to_screen(self):
        self.update_idletasks()
        screen_w = self.winfo_screenwidth()
        screen_h = self.winfo_screenheight()
        w = max(640, int(screen_w * 0.9))
        h = max(480, int(screen_h * 0.85))
        x = (screen_w - w) // 2
        y = (screen_h - h) // 2
        self.geometry(f"{w}x{h}+{x}+{y}")

    def _build_widgets(self):
        # Everything lives inside a scrollable canvas so content is never
        # cut off regardless of screen size - a vertical scrollbar appears
        # on the right whenever content exceeds the visible window height.
        outer_canvas = tk.Canvas(self, highlightthickness=0)
        vscroll = ttk.Scrollbar(self, orient="vertical", command=outer_canvas.yview)
        outer_canvas.configure(yscrollcommand=vscroll.set)

        vscroll.pack(side="right", fill="y")
        outer_canvas.pack(side="left", fill="both", expand=True)

        inner = ttk.Frame(outer_canvas)
        inner_window = outer_canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_inner_configure(_event):
            outer_canvas.configure(scrollregion=outer_canvas.bbox("all"))

        def _on_canvas_configure(event):
            outer_canvas.itemconfig(inner_window, width=event.width)

        inner.bind("<Configure>", _on_inner_configure)
        outer_canvas.bind("<Configure>", _on_canvas_configure)

        def _on_mousewheel(event):
            if event.num == 4:
                outer_canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                outer_canvas.yview_scroll(1, "units")
            else:
                outer_canvas.yview_scroll(int(-event.delta / 120), "units")

        outer_canvas.bind_all("<Button-4>", _on_mousewheel)
        outer_canvas.bind_all("<Button-5>", _on_mousewheel)
        outer_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        root = ttk.Frame(inner, padding=8)
        root.pack(fill="both", expand=True)

        # Left column: controls
        left = ttk.Frame(root)
        left.pack(side="left", fill="y", padx=(0, 8))

        conn_status_frame = ttk.LabelFrame(left, text="Connection", padding=6)
        conn_status_frame.pack(fill="x", pady=4)

        self.conn_status_label = ttk.Label(
            conn_status_frame, text="Serial: disconnected", foreground="#888888"
        )
        self.conn_status_label.pack(anchor="w")

        self.ntrip_status_label = ttk.Label(
            conn_status_frame, text="NTRIP: not connected", foreground="#888888"
        )
        self.ntrip_status_label.pack(anchor="w")

        ttk.Button(
            conn_status_frame, text="Open Setup...", command=self._open_setup_window
        ).pack(fill="x", pady=(6, 0))

        wp_frame = ttk.LabelFrame(left, text="Waypoints", padding=6)
        wp_frame.pack(fill="x", pady=4)

        ttk.Button(wp_frame, text="Load Waypoint File...", command=self._load_waypoints).pack(
            fill="x"
        )

        ttk.Label(
            wp_frame, text="(double-click a waypoint to make it the target)",
            font=("TkDefaultFont", 8), foreground="#888888",
        ).pack(anchor="w")

        wp_list_frame = ttk.Frame(wp_frame)
        wp_list_frame.pack(fill="x", pady=4)

        wp_scroll = ttk.Scrollbar(wp_list_frame, orient="vertical")
        wp_scroll.pack(side="right", fill="y")

        self.wp_listbox = tk.Listbox(
            wp_list_frame, height=5, yscrollcommand=wp_scroll.set
        )
        self.wp_listbox.pack(side="left", fill="both", expand=True)
        wp_scroll.config(command=self.wp_listbox.yview)
        self.wp_listbox.bind("<Double-Button-1>", self._on_waypoint_selected)

        self.loop_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(wp_frame, text="Loop waypoints", variable=self.loop_var).pack(
            anchor="w"
        )

        save_frame = ttk.LabelFrame(left, text="Save Current Location", padding=6)
        save_frame.pack(fill="x", pady=4)
        ttk.Label(save_frame, text="Name:").grid(row=0, column=0, sticky="w")
        self.save_name_var = tk.StringVar(value="WP1")
        ttk.Entry(save_frame, textvariable=self.save_name_var, width=16).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Button(
            save_frame, text="Save Current Location", command=self._save_current_location
        ).grid(row=1, column=0, columnspan=2, pady=(4, 0), sticky="ew")

        opts_frame = ttk.LabelFrame(left, text="Options", padding=6)
        opts_frame.pack(fill="x", pady=4)
        ttk.Label(opts_frame, text="Units:").grid(row=0, column=0, sticky="w")
        self.units_var = tk.StringVar(value="ft")
        ttk.Combobox(
            opts_frame, textvariable=self.units_var, values=["ft", "m"], width=5,
            state="readonly",
        ).grid(row=0, column=1, sticky="w")

        ttk.Label(opts_frame, text="Arrive threshold:").grid(row=1, column=0, sticky="w")
        self.threshold_var = tk.StringVar(value="3")
        ttk.Entry(opts_frame, textvariable=self.threshold_var, width=8).grid(
            row=1, column=1, sticky="w"
        )

        # Right column: live display
        right = ttk.Frame(root)
        right.pack(side="left", fill="both", expand=True)

        status_frame = ttk.Frame(right)
        status_frame.pack(fill="x", pady=(0, 6))

        self.fix_label = ttk.Label(
            status_frame, text="NO FIX", font=("TkDefaultFont", 14, "bold")
        )
        self.fix_label.pack(side="left")

        self.pos_label = ttk.Label(status_frame, text="lat: --   lon: --")
        self.pos_label.pack(side="right")

        self.dop_label = ttk.Label(right, text="HDOP: --    Sats: --")
        self.dop_label.pack(anchor="w")

        self.target_label = ttk.Label(
            right, text="No target loaded", font=("TkDefaultFont", 12)
        )
        self.target_label.pack(anchor="w")

        self.distance_label = ttk.Label(
            right, text="", font=("TkDefaultFont", 16, "bold")
        )
        self.distance_label.pack(anchor="w", pady=(4, 0))

        self.bearing_label = ttk.Label(right, text="", font=("TkDefaultFont", 14))
        self.bearing_label.pack(anchor="w")

        self.canvas = tk.Canvas(right, bg="#1a1a1a", width=600, height=420)
        self.canvas.pack(fill="both", expand=True, pady=6)

        history_frame = ttk.LabelFrame(right, text="History", padding=4)
        history_frame.pack(fill="x")

        self.history_text = scrolledtext.ScrolledText(
            history_frame, height=6, state="disabled", wrap="word",
            bg="#111111", fg="#cccccc", font=("TkFixedFont", 9),
        )
        self.history_text.pack(fill="x")

        ttk.Button(
            history_frame, text="Clear History", command=self._clear_history
        ).pack(anchor="e", pady=(4, 0))

    def _open_setup_window(self):
        if getattr(self, "_setup_win", None) is not None and self._setup_win.winfo_exists():
            self._setup_win.lift()
            self._setup_win.focus_force()
            return

        win = tk.Toplevel(self)
        win.title("Setup - Connection & NTRIP")
        win.geometry("340x560")
        win.transient(self)
        self._setup_win = win

        container = ttk.Frame(win, padding=8)
        container.pack(fill="both", expand=True)

        conn_frame = ttk.LabelFrame(container, text="Serial Connection", padding=6)
        conn_frame.pack(fill="x", pady=4)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(
            conn_frame, textvariable=self.port_var, width=18,
            values=self._list_ports(),
        )
        self.port_combo.grid(row=0, column=1, pady=2)

        ttk.Label(conn_frame, text="Baud:").grid(row=1, column=0, sticky="w")
        ttk.Entry(conn_frame, textvariable=self.baud_var, width=20).grid(
            row=1, column=1, pady=2
        )

        self.connect_btn = ttk.Button(
            conn_frame, text="Disconnect" if self.stream is not None else "Connect",
            command=self._toggle_connect,
        )
        self.connect_btn.grid(row=2, column=0, columnspan=2, pady=(6, 0), sticky="ew")

        ntrip_frame = ttk.LabelFrame(container, text="NTRIP RTK (optional)", padding=6)
        ntrip_frame.pack(fill="x", pady=4)

        ttk.Checkbutton(
            ntrip_frame, text="Enable NTRIP corrections", variable=self.ntrip_enabled
        ).grid(row=0, column=0, columnspan=2, sticky="w")

        labels = ["Server", "Port", "Mountpoint", "User", "Password"]
        for i, lbl in enumerate(labels, start=1):
            ttk.Label(ntrip_frame, text=f"{lbl}:").grid(row=i, column=0, sticky="w")
            show = "*" if lbl == "Password" else ""
            ttk.Entry(
                ntrip_frame, textvariable=self.ntrip_vars[lbl.lower()], width=20, show=show
            ).grid(row=i, column=1, pady=1)

        ttk.Button(container, text="Close", command=win.destroy).pack(
            fill="x", pady=(8, 0)
        )

    def _list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _guess_port(self):
        ports = self._list_ports()
        return ports[0] if ports else "/dev/ttyACM0"

    # ---- History log ----

    def _log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.history_text.config(state="normal")
        self.history_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.history_text.see(tk.END)
        self.history_text.config(state="disabled")

    def _clear_history(self):
        self.history_text.config(state="normal")
        self.history_text.delete("1.0", tk.END)
        self.history_text.config(state="disabled")

    # ---- Settings persistence ----

    def _load_settings(self):
        if not os.path.exists(CONFIG_PATH):
            return
        try:
            with open(CONFIG_PATH, "r") as f:
                cfg = json.load(f)
        except (OSError, json.JSONDecodeError):
            return

        self.port_var.set(cfg.get("port", self.port_var.get()))
        self.baud_var.set(cfg.get("baud", self.baud_var.get()))
        self.ntrip_enabled.set(cfg.get("ntrip_enabled", False))

        ntrip_cfg = cfg.get("ntrip", {})
        for key, var in self.ntrip_vars.items():
            if key in ntrip_cfg:
                var.set(ntrip_cfg[key])

        self.units_var.set(cfg.get("units", self.units_var.get()))
        self.threshold_var.set(cfg.get("threshold", self.threshold_var.get()))
        self.loop_var.set(cfg.get("loop", False))
        self.save_name_var.set(cfg.get("save_name", self.save_name_var.get()))

        geometry = cfg.get("geometry")
        if geometry:
            try:
                size_part = geometry.split("+")[0]
                saved_w, saved_h = (int(v) for v in size_part.split("x"))
                if saved_w <= self.winfo_screenwidth() and saved_h <= self.winfo_screenheight():
                    self.geometry(geometry)
                # else: keep the screen-fitted geometry set in _fit_to_screen()
            except (ValueError, IndexError):
                pass

        wp_path = cfg.get("waypoint_filepath")
        if wp_path and os.path.exists(wp_path):
            try:
                wps = load_waypoints(wp_path)
            except Exception:
                wps = []
            if wps:
                self.waypoints = wps
                self.waypoint_filepath = wp_path
                for wp in wps:
                    self.wp_listbox.insert(
                        tk.END, f"{wp['name']}  ({wp['lat']:.8f}, {wp['lon']:.8f})"
                    )
                self.target_index = min(cfg.get("target_index", 0), len(wps) - 1)
                self._highlight_target()
                self._update_target_label()

        self._log("Loaded saved settings from last session")

    def _save_settings(self):
        cfg = {
            "port": self.port_var.get(),
            "baud": self.baud_var.get(),
            "ntrip_enabled": self.ntrip_enabled.get(),
            "ntrip": {k: v.get() for k, v in self.ntrip_vars.items()},
            "units": self.units_var.get(),
            "threshold": self.threshold_var.get(),
            "loop": self.loop_var.get(),
            "save_name": self.save_name_var.get(),
            "waypoint_filepath": self.waypoint_filepath,
            "target_index": self.target_index,
            "geometry": self.geometry(),
        }
        try:
            with open(CONFIG_PATH, "w") as f:
                json.dump(cfg, f, indent=2)
        except OSError:
            pass

    def _on_close(self):
        self._save_settings()
        self.destroy()

    # ---- Connection handling ----

    def _toggle_connect(self):
        if self.stream is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        port = self.port_var.get()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Error", "Baud rate must be a number")
            return

        try:
            self.stream = serial.Serial(port, baud, timeout=3)
        except serial.SerialException as e:
            messagebox.showerror("Serial error", str(e))
            self.stream = None
            self._log(f"Connect failed on {port}: {e}")
            return

        self.stop_event.clear()
        self.reader_thread = SerialReaderThread(
            self.stream, self.fix_queue, self.gnss_state, self.stop_event
        )
        self.reader_thread.start()
        self._log(f"Connected to {port} @ {baud} baud")

        if self.ntrip_enabled.get():
            if GNSSNTRIPClient is None:
                messagebox.showwarning(
                    "pygnssutils not found",
                    "NTRIP requires pygnssutils. Install with:\n"
                    "pip install pygnssutils",
                )
                self._log("NTRIP enabled but pygnssutils not installed")
            else:
                self._start_ntrip()

        if self.connect_btn is not None:
            self.connect_btn.config(text="Disconnect")
        self.conn_status_label.config(text=f"Serial: connected ({port})", foreground="#27ae60")

    def _start_ntrip(self):
        v = self.ntrip_vars
        server = v["server"].get().strip()
        mountpoint = v["mountpoint"].get().strip()
        if not server or not mountpoint:
            messagebox.showwarning(
                "NTRIP", "Server and Mountpoint are required to enable NTRIP."
            )
            return
        try:
            port = int(v["port"].get())
        except ValueError:
            port = 2101

        # Wrap the serial stream's write() so we can detect whether RTCM
        # correction bytes are actually flowing from the caster. We patch
        # the instance method (not replace the object) so isinstance(stream,
        # Serial) checks inside pygnssutils still pass.
        self.last_rtcm_time = None
        self.rtcm_byte_count = 0
        original_write = self.stream.write

        def _write_wrapper(data, *a, **kw):
            self.last_rtcm_time = time.time()
            self.rtcm_byte_count += len(data)
            return original_write(data, *a, **kw)

        self.stream.write = _write_wrapper

        self.ntrip_status_label.config(text="NTRIP: connecting...", foreground="#e67e22")
        self._log(f"NTRIP: connecting to {server}:{port} mountpoint '{mountpoint}'")

        self.ntrip_client = GNSSNTRIPClient(app=self.gnss_state)
        kwargs = dict(
            server=server,
            port=port,
            mountpoint=mountpoint,
            ntripuser=v["user"].get(),
            ntrippassword=v["password"].get(),
            ggainterval=10,
            ggamode=0,
            output=self.stream,
        )

        def _run():
            try:
                self.ntrip_client.run(**kwargs)
            except Exception as e:
                self.fix_queue.put({"ntrip_error": str(e)})

        threading.Thread(target=_run, daemon=True).start()

    def _disconnect(self):
        self.stop_event.set()
        if self.ntrip_client is not None:
            try:
                self.ntrip_client.stop()
            except Exception:
                pass
            self.ntrip_client = None
        if self.stream is not None:
            try:
                self.stream.close()
            except Exception:
                pass
            self.stream = None
        if self.connect_btn is not None:
            self.connect_btn.config(text="Connect")
        self.conn_status_label.config(text="Serial: disconnected", foreground="#888888")
        self.fix_label.config(text="DISCONNECTED", foreground="#888888")
        self.ntrip_status_label.config(text="NTRIP: not connected", foreground="#888888")
        self.last_rtcm_time = None
        self.rtcm_byte_count = 0
        self._log("Disconnected")

    # ---- Waypoint handling ----

    def _load_waypoints(self):
        filepath = filedialog.askopenfilename(
            title="Select waypoint file",
            filetypes=[("Text/CSV files", "*.txt *.csv"), ("All files", "*.*")],
        )
        if not filepath:
            return
        try:
            wps = load_waypoints(filepath)
        except Exception as e:
            messagebox.showerror("Error loading waypoints", str(e))
            return
        if not wps:
            messagebox.showwarning("No waypoints", "No valid waypoints found in file.")
            return

        self.waypoints = wps
        self.waypoint_filepath = filepath
        self.target_index = 0
        self._arrived_index = None
        self.wp_listbox.delete(0, tk.END)
        for wp in wps:
            self.wp_listbox.insert(
                tk.END, f"{wp['name']}  ({wp['lat']:.8f}, {wp['lon']:.8f})"
            )
        self._highlight_target()
        self._update_target_label()
        self._log(f"Loaded {len(wps)} waypoint(s) from {filepath}")

    def _on_waypoint_selected(self, _event):
        sel = self.wp_listbox.curselection()
        if sel:
            self.target_index = sel[0]
            self._arrived_index = None
            self._highlight_target()
            self._update_target_label()

    def _highlight_target(self):
        self.wp_listbox.selection_clear(0, tk.END)
        if self.waypoints:
            self.wp_listbox.selection_set(self.target_index)
            self.wp_listbox.see(self.target_index)

    def _update_target_label(self):
        if self.waypoints:
            t = self.waypoints[self.target_index]
            self.target_label.config(
                text=f"Target: {t['name']}  ({t['lat']:.8f}, {t['lon']:.8f})"
            )
        else:
            self.target_label.config(text="No target loaded")

    # ---- Save current location ----

    def _save_current_location(self):
        if self.current_lat is None or self.current_lon is None:
            messagebox.showwarning("No fix", "No GNSS fix yet - nothing to save.")
            return

        name = self.save_name_var.get().strip()
        if not name:
            messagebox.showwarning("Name required", "Enter a name for this waypoint.")
            return

        if self.waypoint_filepath is None:
            filepath = filedialog.asksaveasfilename(
                title="Create/select waypoint file",
                defaultextension=".txt",
                filetypes=[("Text/CSV files", "*.txt *.csv"), ("All files", "*.*")],
            )
            if not filepath:
                return
            self.waypoint_filepath = filepath
            if not __import__("os").path.exists(filepath):
                with open(filepath, "w") as f:
                    f.write("# name, latitude, longitude\n")

        line = f"{name}, {self.current_lat:.8f}, {self.current_lon:.8f}\n"
        try:
            with open(self.waypoint_filepath, "a") as f:
                f.write(line)
        except OSError as e:
            messagebox.showerror("Save error", str(e))
            return

        wp = {"name": name, "lat": self.current_lat, "lon": self.current_lon}
        self.waypoints.append(wp)
        self.wp_listbox.insert(
            tk.END, f"{wp['name']}  ({wp['lat']:.8f}, {wp['lon']:.8f})"
        )
        if len(self.waypoints) == 1:
            # first waypoint ever added - make it the active target
            self.target_index = 0
            self._highlight_target()
            self._update_target_label()

        messagebox.showinfo("Saved", f"Saved '{name}' to {self.waypoint_filepath}")
        self._log(f"Saved current location as '{name}' ({wp['lat']:.8f}, {wp['lon']:.8f})")

    # ---- Queue polling / live update ----

    def _poll_queue(self):
        latest = None
        had_fatal_error = False
        try:
            while True:
                item = self.fix_queue.get_nowait()
                if "error" in item:
                    if item.get("raw_error") and item["raw_error"] != item["error"]:
                        self._log(f"Serial error (raw): {item['raw_error']}")
                    messagebox.showerror("Serial error", item["error"])
                    self._disconnect()
                    had_fatal_error = True
                    break
                if "ntrip_error" in item:
                    self.fix_label.config(text="NTRIP ERROR")
                    continue
                latest = item
        except queue.Empty:
            pass

        if had_fatal_error:
            # Reader thread has exited; nothing more to drain. Still
            # reschedule below so polling resumes cleanly after reconnect.
            self.after(200, self._poll_queue)
            return

        if latest is not None:
            self.current_lat = latest["lat"]
            self.current_lon = latest["lon"]
            self.current_quality = latest["quality"]
            if latest.get("hdop") is not None:
                self.current_hdop = latest["hdop"]
            if latest.get("numsv") is not None:
                self.current_numsv = latest["numsv"]
            self._update_display()

        self._update_ntrip_status()

        self.after(200, self._poll_queue)

    def _update_ntrip_status(self):
        if self.ntrip_client is None:
            return  # not enabled, leave as "not connected"

        now = time.time()
        if self.last_rtcm_time is None:
            status, color = "NTRIP: connecting...", "#e67e22"
        elif now - self.last_rtcm_time < 15:
            kb = self.rtcm_byte_count / 1024
            status, color = f"NTRIP: connected ({kb:.1f} KB recv'd)", "#27ae60"
        else:
            status, color = "NTRIP: stalled - no data recently", "#c0392b"

        if status != self._last_ntrip_status:
            self.ntrip_status_label.config(text=status, foreground=color)
            if self._last_ntrip_status is not None:
                self._log(status)
            self._last_ntrip_status = status

    def _update_display(self):
        q = self.current_quality
        label = FIX_QUALITY_LABELS.get(q, str(q))
        color = FIX_QUALITY_COLORS.get(q, "#888888")
        self.fix_label.config(text=label, foreground=color)
        self.pos_label.config(
            text=f"lat: {self.current_lat:.8f}   lon: {self.current_lon:.8f}"
        )

        hdop_str = f"{self.current_hdop:.1f}" if self.current_hdop is not None else "--"
        sats_str = str(self.current_numsv) if self.current_numsv is not None else "--"
        self.dop_label.config(text=f"HDOP: {hdop_str}    Sats: {sats_str}")

        if not self.waypoints or self.current_lat is None:
            self._draw_canvas()
            return

        target = self.waypoints[self.target_index]
        distance_m, bearing_deg = haversine_distance_bearing(
            self.current_lat, self.current_lon, target["lat"], target["lon"]
        )

        units = self.units_var.get()
        distance_display = distance_m * 3.28084 if units == "ft" else distance_m

        try:
            threshold = float(self.threshold_var.get())
        except ValueError:
            threshold = 3.0

        if distance_display <= threshold:
            self.distance_label.config(text="ARRIVED", foreground="#27ae60")
            self.bearing_label.config(text=f"at '{target['name']}'")
            was_new_arrival = self._arrived_index != self.target_index
            self._advance_waypoint()
            if was_new_arrival:
                self._log(f"Arrived at waypoint '{target['name']}'")
        else:
            compass = bearing_to_compass(bearing_deg)
            self.distance_label.config(
                text=f"{distance_display:.1f} {units}", foreground="white"
            )
            self.bearing_label.config(
                text=f"Bearing: {bearing_deg:.0f}\u00b0 ({compass})  to '{target['name']}'"
            )

        self._draw_canvas()

    def _advance_waypoint(self):
        # Called repeatedly while within threshold; only advance once per arrival.
        if self._arrived_index == self.target_index:
            return
        self._arrived_index = self.target_index
        self.target_index += 1
        if self.target_index >= len(self.waypoints):
            self.target_index = 0 if self.loop_var.get() else len(self.waypoints) - 1
        self._highlight_target()
        self._update_target_label()

    # ---- Canvas drawing ----

    def _draw_canvas(self):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width() or 600
        h = c.winfo_height() or 420
        cx, cy = w / 2, h / 2

        if self.current_lat is None:
            c.create_text(
                cx, cy, text="Waiting for GNSS fix...", fill="#888888",
                font=("TkDefaultFont", 14),
            )
            return

        # Compass rose
        radius = min(w, h) * 0.42
        c.create_oval(
            cx - radius, cy - radius, cx + radius, cy + radius,
            outline="#444444", width=1,
        )
        for ang, lbl in [(0, "N"), (90, "E"), (180, "S"), (270, "W")]:
            rad = math.radians(ang)
            x = cx + radius * math.sin(rad)
            y = cy - radius * math.cos(rad)
            c.create_text(x, y, text=lbl, fill="#666666", font=("TkDefaultFont", 10))

        # Current position marker (center)
        c.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#3498db", outline="")

        if not self.waypoints:
            return

        # Plot only the currently selected target waypoint
        target = self.waypoints[self.target_index]
        east, north = local_enu(
            self.current_lat, self.current_lon, target["lat"], target["lon"]
        )
        dist = max(math.hypot(east, north), 1.0)
        scale = radius / dist

        x = cx + east * scale
        y = cy - north * scale
        c.create_line(cx, cy, x, y, fill="#e74c3c", dash=(4, 2))
        c.create_oval(x - 7, y - 7, x + 7, y + 7, fill="#e74c3c", outline="")
        c.create_text(
            x, y - 17, text=target["name"], fill="#cccccc", font=("TkDefaultFont", 9)
        )

    def destroy(self):
        self._disconnect()
        super().destroy()


if __name__ == "__main__":
    app = NavigateApp()
    app.mainloop()

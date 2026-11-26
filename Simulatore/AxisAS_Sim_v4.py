# -*- coding: utf-8 -*-
"""
AxisAS_Sim_v4 - Simulatore encoder per il posizionatore AxisAS_Mng_v4 (FMT)

Legge dal PLC (PLCSIM Advanced o PLC reale) i comandi motore scritti nel DB "AxisAS_SimIO"
(marcia avanti/indietro, sblocco freno, riferimento velocità), calcola il moto con un modello
fisico di motore asincrono + inverter + trasmissione e riscrive nel DB la posizione encoder,
l'inverter pronto e un contatore di vita.

DB "AxisAS_SimIO" (accesso STANDARD, offset fissi):
    0.0  xCmdForw    Bool  PLC -> SIM
    0.1  xCmdBack    Bool  PLC -> SIM
    0.2  xCmdBrake   Bool  PLC -> SIM
    2    fSpeedRef   Real  PLC -> SIM   (mm/s, modulo)
    6    fEncoder    Real  SIM -> PLC   (mm)
    10.0 xDriveReady Bool  SIM -> PLC
    10.1 xLevelSensor Bool SIM -> PLC   (sensore di livello virtuale, per la ricerca livello)
    12   iHeartbeat  Int   SIM -> PLC

Requisiti:  Python 3.9+,  pip install python-snap7
In PLCSIM Advanced / CPU: abilitare "Accesso PUT/GET da partner remoto" nelle proprietà della CPU.
Avvio:      python AxisAS_Sim_v4.py
"""
import json
import math
import os
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import snap7
    from snap7.util import get_bool, get_real, set_bool, set_real, set_int
    SNAP7_OK = True
except ImportError:  # l'app parte comunque, senza collegamento
    SNAP7_OK = False

APP_TITLE = "AxisAS_Sim_v4 - Simulatore encoder posizionatore"
CFG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "AxisAS_Sim_v4.json")
SIM_PERIOD = 0.002          # periodo del ciclo di simulazione (s). L'encoder scritto ogni SIM_PERIOD e letto dal PLC
                            # in modo asincrono produce un errore sulla velocità stimata dal PLC di circa
                            # SIM_PERIOD / ciclo OB (5 ms: 91..106 mm/s su 100, 2 ms: 96..103, verificato con modello)
COMM_TIMEOUT = 1.0          # errori di comunicazione continui per più di questo tempo -> scollegato (s)
RECONNECT_PERIOD = 2.0      # intervallo tra i tentativi di ricollegamento automatico (s)

# ------------------------------------------------------------------------------------------------
# Parametri (valori di default = motore 1,5 kW 4 poli, riduttore 7,5, puleggia 50 mm, come PosSim)
# ------------------------------------------------------------------------------------------------
DEFAULTS = {
    # collegamento
    "ip": "192.168.20.11", "rack": 0, "slot": 1, "db": 900,
    # motore
    "rated_power_kw": 1.5, "rated_speed_rpm": 1420.0, "rated_freq_hz": 50.0, "rated_current_a": 3.5,
    "cos_phi": 0.80, "torque_overload": 2.2, "slip_pct": 0.0, "motor_inertia": 0.0025,
    # meccanica
    "kinematics": "Puleggia", "pulley_diameter": 50.0, "screw_pitch": 10.0, "gear_ratio": 7.5,
    "efficiency": 0.85, "load_inertia": 0.005, "friction_torque": 0.6, "load_torque": 0.0,
    "brake_torque_factor": 2.0,
    # inverter / encoder
    "magn_time_ms": 150.0, "drive_lag_ms": 10.0, "enc_resolution": 0.01, "enc_invert": False, "enc_offset": 0.0,
    # finestra oscilloscopio
    "scope_window_s": 20.0,
    # sensore di livello virtuale
    "level_enabled": False, "level_pos": 500.0, "level_above": True,
}

PARAM_UI = [  # (scheda, chiave, etichetta, tipo)
    ("Motore", "rated_power_kw", "Potenza nominale [kW]", float),
    ("Motore", "rated_speed_rpm", "Velocità nominale [rpm]", float),
    ("Motore", "rated_freq_hz", "Frequenza nominale [Hz]", float),
    ("Motore", "rated_current_a", "Corrente nominale [A]", float),
    ("Motore", "cos_phi", "cos φ", float),
    ("Motore", "torque_overload", "Coppia massima [x Cn]", float),
    ("Motore", "slip_pct", "Scorrimento a coppia nominale [%]", float),
    ("Motore", "motor_inertia", "Inerzia motore [kg·m²]", float),
    ("Meccanica", "kinematics", "Trasmissione (Puleggia / Vite)", str),
    ("Meccanica", "pulley_diameter", "Diametro puleggia [mm]", float),
    ("Meccanica", "screw_pitch", "Passo vite [mm]", float),
    ("Meccanica", "gear_ratio", "Rapporto riduttore", float),
    ("Meccanica", "efficiency", "Rendimento", float),
    ("Meccanica", "load_inertia", "Inerzia carico [kg·m²] (lato uscita riduttore)", float),
    ("Meccanica", "friction_torque", "Coppia di attrito [Nm] (lato motore)", float),
    ("Meccanica", "load_torque", "Carico costante [Nm] (gravità, + frena l'avanti)", float),
    ("Meccanica", "brake_torque_factor", "Coppia freno [x Cn]", float),
    ("Inverter / Encoder", "magn_time_ms", "Tempo magnetizzazione [ms]", float),
    ("Inverter / Encoder", "drive_lag_ms", "Ritardo riferimento inverter [ms]", float),
    ("Inverter / Encoder", "enc_resolution", "Risoluzione encoder [mm]", float),
    ("Inverter / Encoder", "enc_invert", "Encoder invertito (prova allarme)", bool),
    ("Inverter / Encoder", "enc_offset", "Offset encoder [mm]", float),
    ("Livello", "level_enabled", "Sensore livello attivo", bool),
    ("Livello", "level_pos", "Quota del livello (encoder) [mm]", float),
    ("Livello", "level_above", "Si attiva con posizione >= quota (spento: <= quota)", bool),
]


# ------------------------------------------------------------------------------------------------
# Modello fisico: motore asincrono su inverter U/f + riduttore + puleggia/vite + freno
# ------------------------------------------------------------------------------------------------
class AxisModel:
    def __init__(self, p):
        self.p = dict(p)
        self.pos = 0.0            # posizione carico (mm)
        self.speed = 0.0          # velocità carico (mm/s)
        self.ref_filt = 0.0       # riferimento visto dall'inverter dopo il ritardo (mm/s, con segno)
        self.torque = 0.0         # coppia motore (Nm)
        self.current = 0.0        # corrente (A)
        self.rpm = 0.0
        self.magn_t = 0.0         # tempo con marcia comandata (s)
        self.drive_ready = False
        # guasti simulabili
        self.f_jammed = False
        self.f_drive_fault = False
        self.f_enc_frozen = False
        self._enc_frozen_val = 0.0

    # --- grandezze derivate
    def k_trans(self):
        """mm di corsa per radiante all'albero motore."""
        p = self.p
        gr = p["gear_ratio"] if p["gear_ratio"] > 0 else 1.0
        if str(p["kinematics"]).lower().startswith("vite"):
            k = p["screw_pitch"] / (2.0 * math.pi * gr)
        else:
            k = p["pulley_diameter"] / (2.0 * gr)
        return k if k > 0 else 1.0

    def rated_torque(self):
        wn = self.p["rated_speed_rpm"] * 2.0 * math.pi / 60.0
        return self.p["rated_power_kw"] * 1000.0 / wn if wn > 0 else 10.0

    def sync_rpm(self):
        p = self.p
        poles = round(120.0 * p["rated_freq_hz"] / (p["rated_speed_rpm"] * 1.05)) if p["rated_speed_rpm"] > 0 else 4
        if poles % 2:
            poles += 1
        poles = max(poles, 2)
        return 120.0 * p["rated_freq_hz"] / poles

    def max_mech_speed(self):
        """velocità del carico a velocità nominale del motore (mm/s)."""
        return self.p["rated_speed_rpm"] * 2.0 * math.pi / 60.0 * self.k_trans()

    def total_inertia(self):
        gr = self.p["gear_ratio"] if self.p["gear_ratio"] > 0 else 1.0
        j = self.p["motor_inertia"] + self.p["load_inertia"] / (gr * gr)
        return j if j > 0 else 1e-6

    def encoder(self):
        p = self.p
        e = (-self.pos if p["enc_invert"] else self.pos) + p["enc_offset"]
        r = p["enc_resolution"]
        if r > 0:
            e = round(e / r) * r
        if self.f_enc_frozen:
            return self._enc_frozen_val
        self._enc_frozen_val = e
        return e

    def set_position(self, enc_value):
        p = self.p
        x = enc_value - p["enc_offset"]
        self.pos = -x if p["enc_invert"] else x
        self.speed = 0.0

    # --- passo di simulazione
    def step(self, fwd, back, brake_open, speed_ref, dt):
        p = self.p
        if dt <= 0:
            return
        run = (fwd or back) and not (fwd and back)

        # inverter: magnetizzazione e pronto
        self.magn_t = self.magn_t + dt if run else 0.0
        self.drive_ready = run and (self.magn_t * 1000.0 >= p["magn_time_ms"]) and not self.f_drive_fault

        # riferimento con segno e ritardo del primo ordine dell'inverter
        ref = abs(speed_ref) * (1.0 if fwd else -1.0) if (run and self.drive_ready) else 0.0
        lag = p["drive_lag_ms"] / 1000.0
        self.ref_filt += (ref - self.ref_filt) * (min(1.0, dt / lag) if lag > 0 else 1.0)

        k = self.k_trans()
        w = self.speed / k                       # rad/s albero motore
        J = self.total_inertia()
        Tn = self.rated_torque()
        eta = p["efficiency"] if p["efficiency"] > 0 else 1.0
        Tf = p["friction_torque"]
        Tl = p["load_torque"]                    # positivo: si oppone al moto in avanti (es. gravità)

        if self.f_jammed:
            self.speed, self.torque, w_new = 0.0, Tn * p["torque_overload"] if run else 0.0, 0.0
        else:
            # coppia disponibile (deflussaggio sopra la velocità di sincronismo)
            Tmax = Tn * p["torque_overload"]
            rpm = abs(w) * 60.0 / (2.0 * math.pi)
            if rpm > self.sync_rpm() > 0:
                Tmax *= self.sync_rpm() / rpm

            if run and self.drive_ready:
                # inverter U/f: insegue il riferimento con lo scorrimento proporzionale alla coppia
                wn = p["rated_speed_rpm"] * 2.0 * math.pi / 60.0
                w_ref = self.ref_filt / k
                # coppia di regime richiesta dal carico (gravità + attrito) -> scorrimento U/f
                T_ss = Tl + (Tf if w_ref > 0 else -Tf if w_ref < 0 else 0.0)
                droop = (p["slip_pct"] / 100.0) * wn * (T_ss / Tn) if Tn > 0 else 0.0
                w_target = w_ref - droop
                T = J * (w_target - w) / dt + T_ss
                T = T / eta if T * w_target >= 0 else T * eta        # perdite del riduttore
                self.torque = max(-Tmax, min(Tmax, T))
            else:
                self.torque = 0.0

            # coppie resistenti
            T_drive = self.torque * eta if self.torque * w >= 0 else self.torque / eta
            T_res = Tl
            if brake_open:
                T_fric = Tf
            else:
                T_fric = Tf + Tn * p["brake_torque_factor"]          # freno meccanico
            net = T_drive - T_res
            # attrito/freno: si oppone al moto, non lo inverte
            if abs(w) > 1e-6:
                net -= math.copysign(T_fric, w)
                w_new = w + net / J * dt
                if w_new * w < 0:
                    w_new = 0.0
            else:
                if abs(net) <= T_fric:
                    w_new = 0.0          # aderenza statica
                else:
                    net -= math.copysign(T_fric, net)
                    w_new = net / J * dt
            self.speed = w_new * k

        self.pos += self.speed * dt
        self.rpm = w_new * 60.0 / (2.0 * math.pi) if not self.f_jammed else 0.0
        # corrente (stima): magnetizzante + attiva proporzionale alla coppia
        In, cphi = p["rated_current_a"], max(0.05, min(1.0, p["cos_phi"]))
        i_mag = In * math.sin(math.acos(cphi)) if (run and self.drive_ready) or self.magn_t > 0 else 0.0
        i_act = In * cphi * abs(self.torque) / Tn if Tn > 0 else 0.0
        self.current = math.sqrt(i_mag ** 2 + i_act ** 2)


# ------------------------------------------------------------------------------------------------
# Collegamento PLC via snap7
# ------------------------------------------------------------------------------------------------
class PlcLink:
    """Client snap7. Tutte le chiamate al client passano da io_lock: il client non è thread-safe
    (lo usano il thread di simulazione e il thread di ricollegamento)."""

    def __init__(self):
        self.client = None
        self.connected = False
        self.db = 900
        self.error = ""
        self.io_lock = threading.Lock()

    def connect(self, ip, rack, slot, db):
        if not SNAP7_OK:
            self.error = "python-snap7 non installato (pip install python-snap7)"
            return False
        with self.io_lock:
            self.connected = False
            self._close()
            self.db = db
            try:
                # client nuovo a ogni collegamento: dopo una caduta il vecchio può restare in uno stato non valido
                self.client = snap7.client.Client()
                self.client.connect(ip, rack, slot)
                # prova di lettura del DB: collegato solo se il DB SimIO risponde
                self.client.db_read(self.db, 0, 14)
                self.error = ""
                self.connected = True
            except Exception as e:
                self.error = str(e)
                self._close()
        return self.connected

    def _close(self):
        if self.client is not None:
            try:
                self.client.disconnect()
            except Exception:
                pass
            try:
                self.client.destroy()
            except Exception:
                pass
        self.client = None

    def disconnect(self):
        with self.io_lock:
            self.connected = False
            self._close()

    def read_cmd(self):
        with self.io_lock:
            if self.client is None:
                raise ConnectionError("client non collegato")
            data = self.client.db_read(self.db, 0, 6)
        return get_bool(data, 0, 0), get_bool(data, 0, 1), get_bool(data, 0, 2), get_real(data, 2)

    def write_fb(self, encoder, ready, heartbeat, level=False):
        data = bytearray(8)
        set_real(data, 0, encoder)
        set_bool(data, 4, 0, ready)
        set_bool(data, 4, 1, level)
        set_int(data, 6, heartbeat)
        with self.io_lock:
            if self.client is None:
                raise ConnectionError("client non collegato")
            self.client.db_write(self.db, 6, data)


# ------------------------------------------------------------------------------------------------
# Motore di simulazione (thread)
# ------------------------------------------------------------------------------------------------
class SimEngine:
    """Thread di simulazione + thread di ricollegamento.

    Comunicazione: lettura comandi e scrittura encoder avvengono FUORI da self.lock (che protegge modello e storico),
    così una chiamata di rete lenta non blocca l'interfaccia. Un errore isolato non scollega: si mantiene l'ultimo
    comando e si scollega solo dopo COMM_TIMEOUT secondi di errori continui. Se il collegamento cade (non per
    "Scollega"), il thread di ricollegamento riprova ogni RECONNECT_PERIOD secondi senza bloccare la simulazione.
    """

    def __init__(self, params):
        self.model = AxisModel(params)
        self.link = PlcLink()
        self.lock = threading.Lock()
        self.running = True
        self.heartbeat = 0
        self.cmd = (False, False, False, 0.0)
        self.level_on = False
        self.level_force = False
        self.cycle_ms = 0.0
        self.cycle_max_ms = 0.0      # ciclo di simulazione più lungo nell'ultimo secondo
        self.history = []            # (t, pos_enc, speed_ref_signed, speed, torque)
        self.t0 = time.perf_counter()
        # stato del collegamento
        self.want_connected = False  # True da "Connetti" a "Scollega": abilita il ricollegamento automatico
        self.conn_params = None      # (ip, rack, slot, db)
        self.comm_fail_since = None  # inizio della serie di errori in corso (perf_counter) o None
        self.reconnects = 0          # ricollegamenti automatici riusciti
        self.drops = 0               # cadute del collegamento
        self.last_error = ""
        self.last_error_time = ""
        self.manual_result = None    # (ok, errore) del collegamento chiesto con "Connetti", letto dall'interfaccia
        self._manual_pending = False
        self._wake = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self.rthread = threading.Thread(target=self._reconnect_loop, daemon=True)
        self.rthread.start()

    # --- comandi dall'interfaccia
    def request_connect(self, ip, rack, slot, db):
        self.conn_params = (ip, rack, slot, db)
        self.manual_result = None
        self._manual_pending = True
        self.want_connected = True
        self._wake.set()

    def request_disconnect(self):
        self.want_connected = False
        self._manual_pending = False
        self.comm_fail_since = None
        self.link.disconnect()

    def conn_state(self):
        """'on' collegato, 'retry' collegamento/ricollegamento in corso, 'off' scollegato."""
        if self.link.connected:
            return "on"
        return "retry" if self.want_connected else "off"

    # --- errori di comunicazione
    def _note_error(self, msg):
        self.last_error = msg
        self.last_error_time = time.strftime("%H:%M:%S")

    def _comm_error(self, e, now):
        self.link.error = str(e)
        self._note_error(str(e))
        if self.comm_fail_since is None:
            self.comm_fail_since = now
        elif now - self.comm_fail_since > COMM_TIMEOUT and self.link.connected:
            self.link.disconnect()
            self.comm_fail_since = None
            self.drops += 1
            self._note_error(f"collegamento perso dopo {COMM_TIMEOUT:.0f} s di errori: {e}")

    # --- thread di simulazione
    def _loop(self):
        last = time.perf_counter()
        win_start, win_max = last, 0.0
        while self.running:
            now = time.perf_counter()
            dt_real = now - last
            dt = min(dt_real, 0.05)
            last = now
            link = self.link
            ok = True

            # lettura comandi (fuori da self.lock)
            fwd, back, brake, ref = self.cmd
            if link.connected:
                try:
                    fwd, back, brake, ref = link.read_cmd()
                except Exception as e:
                    ok = False
                    self._comm_error(e, now)
                    # errore isolato: si tiene l'ultimo comando; se il collegamento è caduto, tutto a zero
                    if not link.connected:
                        fwd, back, brake, ref = False, False, False, 0.0
            else:
                fwd, back, brake, ref = False, False, False, 0.0

            with self.lock:
                self.cmd = (fwd, back, brake, ref)
                m = self.model
                m.step(fwd, back, brake, ref, dt)
                enc = m.encoder()
                ready = m.drive_ready
                p = m.p
                lev = False
                if p.get("level_enabled"):
                    lp = float(p.get("level_pos", 0.0))
                    lev = enc >= lp if p.get("level_above", True) else enc <= lp
                self.level_on = lev or self.level_force
                level_on = self.level_on
                self.heartbeat = (self.heartbeat + 1) % 32000
                heartbeat = self.heartbeat
                t = now - self.t0
                sref = ref if fwd else (-ref if back else 0.0)
                self.history.append((t, enc, sref, m.speed, m.torque))
                # almeno 90 s di storico con SIM_PERIOD = 2 ms (finestra massima dell'oscilloscopio 60 s)
                if len(self.history) > 60000:
                    del self.history[:15000]

            # scrittura encoder (fuori da self.lock)
            if link.connected:
                try:
                    link.write_fb(enc, ready, heartbeat, level_on)
                except Exception as e:
                    ok = False
                    self._comm_error(e, now)
                if ok:
                    self.comm_fail_since = None

            # tempo ciclo: medio e massimo sull'ultimo secondo
            self.cycle_ms = self.cycle_ms * 0.9 + dt_real * 1000.0 * 0.1
            win_max = max(win_max, dt_real * 1000.0)
            if now - win_start >= 1.0:
                self.cycle_max_ms = win_max
                win_start, win_max = now, 0.0

            spent = time.perf_counter() - now
            time.sleep(max(0.0005, SIM_PERIOD - spent))

    # --- thread di ricollegamento (il collegamento può bloccare qualche secondo: non lo fa la simulazione)
    def _reconnect_loop(self):
        last_try = -RECONNECT_PERIOD
        while self.running:
            self._wake.wait(0.2)
            self._wake.clear()
            if not self.running:
                break
            if not self.want_connected or self.link.connected or self.conn_params is None:
                continue
            manual = self._manual_pending
            now = time.perf_counter()
            if not manual and now - last_try < RECONNECT_PERIOD:
                continue
            last_try = now
            ok = self.link.connect(*self.conn_params)
            if not self.want_connected:
                # "Scollega" premuto durante il tentativo: annullo senza segnalare errori
                if ok:
                    self.link.disconnect()
                continue
            if ok:
                self.comm_fail_since = None
                if not manual:
                    self.reconnects += 1
            elif self.want_connected:
                self._note_error(self.link.error)
            if manual:
                self._manual_pending = False
                self.manual_result = (ok, self.link.error)
                if not ok:
                    # primo collegamento non riuscito: niente tentativi automatici, l'interfaccia mostra l'errore
                    self.want_connected = False

    def stop(self):
        self.running = False
        self.want_connected = False
        self._wake.set()
        self.link.disconnect()


# ------------------------------------------------------------------------------------------------
# Oscilloscopio
# ------------------------------------------------------------------------------------------------
class Scope:
    """Oscilloscopio con zoom e cursori.

    Rotella: zoom sul tempo (centrato sul mouse) - trascina col tasto sinistro: zoom sull'area selezionata
    trascina col tasto destro: sposta - clic: cursore A - Shift+clic: cursore B - doppio clic: mostra tutto
    Esc: cancella i cursori. Qualsiasi zoom o spostamento congela il grafico (Riprendi per tornare in tempo reale).
    """
    COLORS = {"pos": "#00d8ff", "ref": "#ffb000", "spd": "#44ff44", "trq": "#ff5555"}
    NAMES = (("pos", 1, "Posizione", "mm", "{:.2f}"), ("ref", 2, "Rif. vel.", "mm/s", "{:.1f}"),
             ("spd", 3, "Velocità", "mm/s", "{:.1f}"), ("trq", 4, "Coppia", "Nm", "{:.2f}"))
    PAD_L, PAD_R, TOP, BOT = 60, 60, 22, 22

    def __init__(self, parent, on_freeze=None, get_full=None):
        self.on_freeze = on_freeze
        self.get_full = get_full    # funzione che restituisce lo storico completo (usata quando si congela)
        self.cv = tk.Canvas(parent, bg="#101418", highlightthickness=0, height=360, cursor="crosshair")
        self.cv.pack(fill="both", expand=True)
        self.readout = tk.StringVar(value="")
        self.delta = tk.StringVar(value="")
        ttk.Label(parent, textvariable=self.readout, font=("Consolas", 9)).pack(anchor="w")
        ttk.Label(parent, textvariable=self.delta, font=("Consolas", 9), foreground="#a05000").pack(anchor="w")
        ttk.Label(parent, foreground="#666", font=("Segoe UI", 8),
                  text="Rotella: zoom  ·  trascina sx: zoom area  ·  trascina dx: sposta  ·  clic: cursore A  ·  "
                       "Shift+clic: cursore B  ·  doppio clic: tutto  ·  Esc: cancella cursori").pack(anchor="w")
        self.level_pos = None    # quota del sensore di livello (linea tratteggiata), None = non mostrata
        self.frozen = False
        self.hist = []          # dati mostrati (copia congelata quando frozen)
        self.times = []
        self.view = None        # (t0, t1) quando zoomato; None = finestra mobile
        self.window = 20.0
        self.hover_x = None
        self.cur_a = None
        self.cur_b = None
        self._drag = None       # ("zoom"|"pan", x_start, view_start)
        self._sel_x = None
        cv = self.cv
        cv.bind("<Motion>", self._motion)
        cv.bind("<Leave>", lambda e: self._set_hover(None))
        cv.bind("<MouseWheel>", self._wheel)                       # Windows / macOS
        cv.bind("<Button-4>", lambda e: self._wheel(e, +120))      # Linux
        cv.bind("<Button-5>", lambda e: self._wheel(e, -120))
        cv.bind("<ButtonPress-1>", self._press_l)
        cv.bind("<B1-Motion>", self._drag_l)
        cv.bind("<ButtonRelease-1>", self._release_l)
        cv.bind("<Double-Button-1>", lambda e: self.zoom_all())
        cv.bind("<ButtonPress-3>", self._press_r)
        cv.bind("<B3-Motion>", self._drag_r)
        cv.bind("<ButtonRelease-3>", lambda e: setattr(self, "_drag", None))
        cv.bind("<Escape>", lambda e: self.clear_cursors())
        cv.bind("<Enter>", lambda e: cv.focus_set())

    # ---------------------------------------------------------------- stato
    def freeze(self, on=True):
        if on and not self.frozen:
            self.frozen = True
            # in tempo reale si riceve solo la coda dello storico: da congelato serve tutto (zoom, "Zoom tutto")
            if self.get_full is not None:
                full = self.get_full()
                if len(full) >= 2:
                    self.hist = full
                    self.times = [h[0] for h in full]
            if self.on_freeze:
                self.on_freeze(True)
        elif not on and self.frozen:
            self.frozen = False
            self.view = None
            if self.on_freeze:
                self.on_freeze(False)

    def clear_cursors(self):
        self.cur_a = self.cur_b = None
        self.redraw()

    def zoom_all(self):
        if len(self.times) >= 2:
            self.freeze(True)
            self.view = (self.times[0], self.times[-1])
            self.redraw()

    def _cur_view(self):
        if self.view is not None:
            return self.view
        t_end = self.times[-1] if self.times else 0.0
        return (t_end - self.window, t_end)

    def _plot_w(self):
        return max(10, max(self.cv.winfo_width(), 100) - self.PAD_L - self.PAD_R)

    def _x2t(self, x):
        t0, t1 = self._cur_view()
        return t0 + (x - self.PAD_L) / self._plot_w() * (t1 - t0)

    def _set_view(self, t0, t1):
        if not self.times:
            return
        span = max(t1 - t0, 0.02)                     # minimo 20 ms (4 campioni)
        lo, hi = self.times[0], self.times[-1]
        full = max(hi - lo, 0.02)
        span = min(span, full)
        t0 = min(max(t0, lo), hi - span)
        self.view = (t0, t0 + span)
        self.freeze(True)
        self.redraw()

    # ---------------------------------------------------------------- mouse
    def _set_hover(self, x):
        self.hover_x = x
        self.redraw()

    def _motion(self, e):
        self._set_hover(e.x)

    def _wheel(self, e, delta=None):
        d = delta if delta is not None else e.delta
        if not self.times:
            return
        t0, t1 = self._cur_view()
        tm = self._x2t(min(max(e.x, self.PAD_L), self.PAD_L + self._plot_w()))
        k = 0.8 if d > 0 else 1.25
        self._set_view(tm - (tm - t0) * k, tm + (t1 - tm) * k)

    def _press_l(self, e):
        self._drag = ("zoom", e.x, None)
        self._sel_x = None

    def _drag_l(self, e):
        if self._drag and self._drag[0] == "zoom" and abs(e.x - self._drag[1]) > 4:
            self._sel_x = e.x
            self.redraw()

    def _release_l(self, e):
        if not self._drag:
            return
        x0 = self._drag[1]
        self._drag = None
        if self._sel_x is not None and abs(e.x - x0) > 4:       # zoom sull'area
            ta, tb = sorted((self._x2t(x0), self._x2t(e.x)))
            self._sel_x = None
            self._set_view(ta, tb)
            return
        self._sel_x = None
        if self.PAD_L <= e.x <= self.PAD_L + self._plot_w() and self.times:   # clic: cursore
            t = self._x2t(e.x)
            if e.state & 0x0001:            # Shift
                self.cur_b = t
            else:
                self.cur_a = t
            self.freeze(True)
            if self.view is None:
                self.view = self._cur_view()
            self.redraw()

    def _press_r(self, e):
        self._drag = ("pan", e.x, self._cur_view())

    def _drag_r(self, e):
        if self._drag and self._drag[0] == "pan":
            v0 = self._drag[2]
            dt = (self._drag[1] - e.x) / self._plot_w() * (v0[1] - v0[0])
            self._set_view(v0[0] + dt, v0[1] + dt)

    # ---------------------------------------------------------------- dati
    def _value_at(self, t):
        """Valori interpolati linearmente all'istante t."""
        import bisect
        ts = self.times
        if not ts:
            return None
        i = bisect.bisect_left(ts, t)
        if i <= 0:
            return self.hist[0]
        if i >= len(ts):
            return self.hist[-1]
        a, b = self.hist[i - 1], self.hist[i]
        f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return tuple(a[k] + (b[k] - a[k]) * f for k in range(5))

    def _fmt(self, h, t_ref):
        return "  ".join(f"{n}={fmt.format(h[i])} {u}" for _, i, n, u, fmt in self.NAMES) + f"   (t={h[0] - t_ref:+.3f} s)"

    def draw(self, hist, window):
        """Chiamata dall'app ogni 50 ms: in tempo reale aggiorna i dati, da congelato ridisegna soltanto."""
        self.window = window
        if not self.frozen:
            self.hist = hist
            self.times = [h[0] for h in hist]
        self.redraw()

    def redraw(self):
        import bisect
        cv = self.cv
        cv.delete("all")
        W, H = max(cv.winfo_width(), 100), max(cv.winfo_height(), 100)
        L, R, top, bot = self.PAD_L, W - self.PAD_R, self.TOP, H - self.BOT
        mid = (top + bot) / 2
        for i in range(0, 11):
            x = L + (R - L) * i / 10
            cv.create_line(x, top, x, bot, fill="#20262c")
        for k in range(5):
            y = top + (bot - top) * k / 4
            cv.create_line(L, y, R, y, fill="#20262c")
        cv.create_line(L, mid, R, mid, fill="#38414a")
        x = L + 4
        for key, _, n, u, _f in self.NAMES:
            cv.create_text(x, 3, anchor="nw", fill=self.COLORS[key], text=f"{n} [{u}]", font=("Segoe UI", 8))
            x += 130
        if self.frozen:
            cv.create_text(R, 3, anchor="ne", fill="#ff8040", text="CONGELATO" + ("  ·  ZOOM" if self.view else ""),
                           font=("Segoe UI", 8, "bold"))
        if len(self.hist) < 2:
            return
        t0, t1 = self._cur_view()
        span = max(t1 - t0, 1e-6)
        i0 = max(0, bisect.bisect_left(self.times, t0) - 1)
        i1 = min(len(self.hist), bisect.bisect_right(self.times, t1) + 1)
        pts = self.hist[i0:i1]
        if len(pts) < 2:
            return
        step = max(1, len(pts) // 1500)
        pts = pts[::step]
        pmin = min(h[1] for h in pts); pmax = max(h[1] for h in pts)
        if pmax - pmin < 0.1:
            c = (pmax + pmin) / 2; pmin, pmax = c - 0.05, c + 0.05
        m = (pmax - pmin) * 0.05; pmin -= m; pmax += m
        vmax = max(1.0, max(max(abs(h[2]), abs(h[3])) for h in pts)) * 1.1
        tmax = max(0.5, max(abs(h[4]) for h in pts)) * 1.1
        fx = lambda t: L + (t - t0) / span * (R - L)
        fp = lambda p: bot - (p - pmin) / (pmax - pmin) * (bot - top)
        fv = lambda v: mid - v / vmax * (mid - top)
        ft = lambda q: mid - q / tmax * (mid - top)
        fun = {"pos": fp, "ref": fv, "spd": fv, "trq": ft}
        for key, idx in (("trq", 4), ("ref", 2), ("spd", 3), ("pos", 1)):
            line = []
            for h in pts:
                line += [fx(h[0]), fun[key](h[idx])]
            cv.create_line(line, fill=self.COLORS[key], width=2 if key in ("pos", "spd") else 1)
        # scale
        f8 = ("Consolas", 8)
        for k in range(5):
            y = top + (bot - top) * k / 4
            cv.create_text(L - 4, y, anchor="e", fill=self.COLORS["pos"], text=f"{pmax - (pmax - pmin) * k / 4:.1f}", font=f8)
            cv.create_text(R + 4, y, anchor="w", fill=self.COLORS["spd"], text=f"{vmax - 2 * vmax * k / 4:.0f}", font=f8)
        t_ref = self.times[-1]
        for i in range(0, 11, 2):
            t = t0 + span * i / 10
            cv.create_text(L + (R - L) * i / 10, bot + 3, anchor="n", fill="#8899aa", text=f"{t - t_ref:+.2f}s", font=f8)
        # quota del sensore di livello
        if self.level_pos is not None and pmin <= self.level_pos <= pmax:
            yl = fp(self.level_pos)
            cv.create_line(L, yl, R, yl, fill="#c080ff", dash=(6, 4))
            cv.create_text(L + 4, yl - 2, anchor="sw", fill="#c080ff", text=f"livello {self.level_pos:.1f}", font=f8)
        # selezione zoom
        if self._sel_x is not None and self._drag:
            cv.create_rectangle(self._drag[1], top, self._sel_x, bot, outline="#ffffff", dash=(2, 2), fill="")
        # cursori fissi
        for t, col, lab in ((self.cur_a, "#ffffff", "A"), (self.cur_b, "#ff60ff", "B")):
            if t is not None and t0 <= t <= t1:
                xc = fx(t)
                cv.create_line(xc, top, xc, bot, fill=col, width=1)
                cv.create_text(xc + 3, top + 2, anchor="nw", fill=col, text=lab, font=("Segoe UI", 9, "bold"))
                h = self._value_at(t)
                for key, idx in (("pos", 1), ("spd", 3), ("ref", 2)):
                    y = fun[key](h[idx])
                    cv.create_oval(xc - 3, y - 3, xc + 3, y + 3, outline=self.COLORS[key])
        # cursore mobile
        txt = ""
        if self.hover_x is not None and L <= self.hover_x <= R:
            t = t0 + (self.hover_x - L) / (R - L) * span
            cv.create_line(self.hover_x, top, self.hover_x, bot, fill="#ff4040", dash=(3, 3))
            h = self._value_at(t)
            for key, idx in (("pos", 1), ("spd", 3)):
                y = fun[key](h[idx])
                cv.create_oval(self.hover_x - 3, y - 3, self.hover_x + 3, y + 3, fill=self.COLORS[key], outline="")
            txt = "Mouse: " + self._fmt(h, t_ref)
        if self.cur_a is not None:
            txt = "A: " + self._fmt(self._value_at(self.cur_a), t_ref) + ("\n" + txt if txt else "")
        self.readout.set(txt)
        if self.cur_a is not None and self.cur_b is not None:
            a, b = self._value_at(self.cur_a), self._value_at(self.cur_b)
            dt = b[0] - a[0]
            dp = b[1] - a[1]
            vm = dp / dt if abs(dt) > 1e-6 else 0.0
            self.delta.set(f"B: {self._fmt(b, t_ref)}\nB-A:  Δt={dt:+.3f} s  Δpos={dp:+.2f} mm  "
                           f"Δvel={b[3] - a[3]:+.1f} mm/s  vel. media={vm:+.1f} mm/s")
        else:
            self.delta.set("")


# ------------------------------------------------------------------------------------------------
# Interfaccia
# ------------------------------------------------------------------------------------------------
class App:
    def __init__(self, root):
        self.root = root
        self.params = dict(DEFAULTS)
        self._load()
        self.eng = SimEngine(self.params)
        self.vars = {}
        self.paused = False
        root.title(APP_TITLE)
        root.geometry("1280x800")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._build()
        self._update()

    # --- parametri su file
    def _load(self):
        try:
            with open(CFG_FILE, encoding="utf-8") as f:
                self.params.update(json.load(f))
        except Exception:
            pass

    def _save(self):
        self._apply()
        try:
            with open(CFG_FILE, "w", encoding="utf-8") as f:
                json.dump(self.params, f, indent=2, ensure_ascii=False)
            self.status(f"Parametri salvati in {CFG_FILE}")
        except Exception as e:
            messagebox.showerror(APP_TITLE, f"Salvataggio non riuscito:\n{e}")

    # --- costruzione
    def _build(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        for i, (key, lab, w) in enumerate((("ip", "IP PLC", 15), ("rack", "Rack", 4), ("slot", "Slot", 4), ("db", "DB SimIO", 6))):
            ttk.Label(top, text=lab).pack(side="left", padx=(8 if i else 0, 2))
            v = tk.StringVar(value=str(self.params[key]))
            self.vars[key] = v
            ttk.Entry(top, textvariable=v, width=w).pack(side="left")
        self.btn_conn = ttk.Button(top, text="Connetti", command=self.toggle_conn)
        self.btn_conn.pack(side="left", padx=10)
        self.lbl_conn = tk.Label(top, text="  NON COLLEGATO  ", bg="#aa2222", fg="white", font=("Segoe UI", 9, "bold"))
        self.lbl_conn.pack(side="left")
        self.lbl_cycle = ttk.Label(top, text="")
        self.lbl_cycle.pack(side="left", padx=10)
        if not SNAP7_OK:
            ttk.Label(top, text="python-snap7 non installato: pip install python-snap7", foreground="red").pack(side="left")
        # diagnostica comunicazione: cadute, ricollegamenti, ultimo errore
        self.lbl_comm = ttk.Label(self.root, text="", foreground="#a03000", padding=(6, 0))
        self.lbl_comm.pack(fill="x")

        body = ttk.PanedWindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=6, pady=6)
        left = ttk.Frame(body, width=420)
        right = ttk.Frame(body)
        body.add(left, weight=0)
        body.add(right, weight=1)

        # parametri
        nb = ttk.Notebook(left)
        nb.pack(fill="both", expand=True)
        tabs = {}
        for tab, key, lab, typ in PARAM_UI:
            if tab not in tabs:
                tabs[tab] = ttk.Frame(nb, padding=8)
                nb.add(tabs[tab], text=tab)
                tabs[tab].columnconfigure(1, weight=1)
            fr = tabs[tab]
            r = fr.grid_size()[1]
            ttk.Label(fr, text=lab).grid(row=r, column=0, sticky="w", pady=2)
            if typ is bool:
                v = tk.BooleanVar(value=bool(self.params[key]))
                ttk.Checkbutton(fr, variable=v).grid(row=r, column=1, sticky="w")
            elif key == "kinematics":
                v = tk.StringVar(value=str(self.params[key]))
                ttk.Combobox(fr, textvariable=v, values=("Puleggia", "Vite"), state="readonly", width=12).grid(row=r, column=1, sticky="w")
            else:
                v = tk.StringVar(value=str(self.params[key]))
                ttk.Entry(fr, textvariable=v, width=12).grid(row=r, column=1, sticky="w")
            self.vars[key] = (v, typ)

        # sensore livello: forzatura manuale e spiegazione
        lf = tabs.get("Livello")
        if lf is not None:
            r = lf.grid_size()[1]
            self.f_lev = tk.BooleanVar()
            ttk.Checkbutton(lf, text="Forza sensore ON (manuale)", variable=self.f_lev,
                            command=self._level_force).grid(row=r, column=0, columnspan=2, sticky="w", pady=(8, 2))
            ttk.Label(lf, foreground="#666", wraplength=360, justify="left",
                      text="Il sensore viene scritto nel DB SimIO (10.1) e usato da AxisAS_Test_v4 come ixLevelSensor "
                           "quando ixUseSim = TRUE. Premi Applica dopo aver cambiato quota o condizione. "
                           "Per provare 'livello non trovato' metti la quota oltre la quota di ricerca o disattiva il sensore."
                      ).grid(row=r + 1, column=0, columnspan=2, sticky="w")

        # guasti
        gf = ttk.Frame(nb, padding=8)
        nb.add(gf, text="Guasti")
        self.f_jam = tk.BooleanVar(); self.f_drv = tk.BooleanVar(); self.f_enc = tk.BooleanVar()
        ttk.Checkbutton(gf, text="Asse bloccato (allarme 1)", variable=self.f_jam, command=self._faults).pack(anchor="w", pady=2)
        ttk.Checkbutton(gf, text="Inverter in guasto / non pronto (allarmi 4 e 7)", variable=self.f_drv, command=self._faults).pack(anchor="w", pady=2)
        ttk.Checkbutton(gf, text="Encoder congelato", variable=self.f_enc, command=self._faults).pack(anchor="w", pady=2)
        ttk.Label(gf, text="Encoder invertito: scheda Inverter / Encoder", foreground="#666").pack(anchor="w", pady=8)

        bf = ttk.Frame(left, padding=(0, 6))
        bf.pack(fill="x")
        ttk.Button(bf, text="Applica", command=self._apply).pack(side="left")
        ttk.Button(bf, text="Salva", command=self._save).pack(side="left", padx=4)
        ttk.Button(bf, text="Valori di default", command=self._defaults).pack(side="left")

        pf = ttk.LabelFrame(left, text="Posizione", padding=6)
        pf.pack(fill="x")
        self.pos_set = tk.StringVar(value="0.0")
        ttk.Entry(pf, textvariable=self.pos_set, width=10).pack(side="left")
        ttk.Button(pf, text="Imposta posizione encoder", command=self._set_pos).pack(side="left", padx=6)

        self.lbl_info = ttk.Label(left, text="", foreground="#335")
        self.lbl_info.pack(anchor="w", pady=4)

        # valori in linea
        vf = ttk.LabelFrame(right, text="Stato", padding=6)
        vf.pack(fill="x")
        self.leds = {}
        for i, (k, lab) in enumerate((("fwd", "Marcia AV"), ("back", "Marcia IND"), ("brake", "Freno sbloccato"), ("ready", "Inverter pronto"),
                                          ("lev", "Sensore livello"))):
            l = tk.Label(vf, text=lab, width=15, bg="#444", fg="white", font=("Segoe UI", 9, "bold"))
            l.grid(row=0, column=i, padx=3, pady=2)
            self.leds[k] = l
        self.vals = {}
        for i, (k, lab) in enumerate((("enc", "Encoder [mm]"), ("ref", "Rif. velocità [mm/s]"), ("spd", "Velocità [mm/s]"),
                                      ("rpm", "Motore [rpm]"), ("trq", "Coppia [Nm]"), ("cur", "Corrente [A]"))):
            ttk.Label(vf, text=lab).grid(row=1 + i // 3 * 2, column=i % 3, sticky="w", padx=4)
            v = tk.StringVar(value="-")
            ttk.Label(vf, textvariable=v, font=("Consolas", 14, "bold")).grid(row=2 + i // 3 * 2, column=i % 3, sticky="w", padx=4)
            self.vals[k] = v

        sf = ttk.LabelFrame(right, text="Oscilloscopio", padding=4)
        sf.pack(fill="both", expand=True, pady=(6, 0))
        cf = ttk.Frame(sf)
        cf.pack(fill="x")
        ttk.Label(cf, text="Finestra [s]").pack(side="left")
        self.win = tk.StringVar(value=str(self.params["scope_window_s"]))
        ttk.Combobox(cf, textvariable=self.win, values=("5", "10", "20", "60"), width=5).pack(side="left", padx=4)
        self.btn_pause = ttk.Button(cf, text="Pausa", command=self._pause)
        self.btn_pause.pack(side="left", padx=4)
        ttk.Button(cf, text="Zoom tutto", command=lambda: self.scope.zoom_all()).pack(side="left")
        ttk.Button(cf, text="Cancella cursori", command=lambda: self.scope.clear_cursors()).pack(side="left", padx=4)
        ttk.Button(cf, text="Pulisci", command=self._clear).pack(side="left")
        self.scope = Scope(sf, on_freeze=self._on_freeze, get_full=self._full_history)

        self.status_var = tk.StringVar(value="Pronto")
        ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w").pack(fill="x", side="bottom")

    # --- azioni
    def status(self, msg):
        self.status_var.set(msg)

    def _apply(self):
        new = dict(self.params)
        try:
            for key, val in self.vars.items():
                if isinstance(val, tuple):
                    v, typ = val
                    if typ is bool:
                        new[key] = bool(v.get())
                    elif typ is float:
                        new[key] = float(str(v.get()).replace(",", "."))
                    else:
                        new[key] = v.get()
                else:
                    new[key] = int(val.get()) if key in ("rack", "slot", "db") else val.get()
        except ValueError as e:
            messagebox.showerror(APP_TITLE, f"Valore non valido:\n{e}")
            return
        new["scope_window_s"] = float(self.win.get() or 20)
        self.params = new
        with self.eng.lock:
            self.eng.model.p.update(new)
        self.status("Parametri applicati")

    def _defaults(self):
        for key, val in self.vars.items():
            if isinstance(val, tuple):
                val[0].set(DEFAULTS[key])
        self._apply()

    def _faults(self):
        with self.eng.lock:
            m = self.eng.model
            m.f_jammed, m.f_drive_fault, m.f_enc_frozen = self.f_jam.get(), self.f_drv.get(), self.f_enc.get()

    def _level_force(self):
        with self.eng.lock:
            self.eng.level_force = bool(self.f_lev.get())

    def _set_pos(self):
        try:
            v = float(self.pos_set.get().replace(",", "."))
        except ValueError:
            return
        with self.eng.lock:
            self.eng.model.set_position(v)
        self.status(f"Posizione encoder impostata a {v:.2f} mm")

    def _pause(self):
        self.scope.freeze(not self.scope.frozen)

    def _on_freeze(self, frozen):
        self.paused = frozen
        self.btn_pause.config(text="Riprendi" if frozen else "Pausa")

    def _full_history(self):
        with self.eng.lock:
            return list(self.eng.history)

    def _clear(self):
        with self.eng.lock:
            self.eng.history.clear()
        self.scope.hist, self.scope.times = [], []
        self.scope.cur_a = self.scope.cur_b = None
        self.scope.freeze(False)

    def toggle_conn(self):
        e = self.eng
        if e.want_connected or e.link.connected:
            e.request_disconnect()
            self.status("Scollegato")
            return
        self._apply()
        p = self.params
        self.status(f"Collegamento a {p['ip']} ...")
        # il collegamento avviene nel thread di ricollegamento: l'esito arriva in _update (eng.manual_result)
        e.request_connect(p["ip"], int(p["rack"]), int(p["slot"]), int(p["db"]))

    def _check_manual_result(self):
        res = self.eng.manual_result
        if res is None:
            return
        self.eng.manual_result = None
        ok, err = res
        p = self.params
        if ok:
            self.status(f"Collegato a {p['ip']}, DB{p['db']} (ricollegamento automatico attivo)")
        else:
            self.status("Collegamento non riuscito")
            messagebox.showerror(APP_TITLE, "Collegamento non riuscito.\n\n"
                                 f"{err}\n\n"
                                 "Controlla: IP del PLC, DB AxisAS_SimIO presente con accesso STANDARD (non ottimizzato), "
                                 "'Accesso PUT/GET' abilitato nella CPU, rack 0 / slot 1 per S7-1500 (slot 1 anche per S7-1200).")

    # --- aggiornamento periodico
    def _update(self):
        e = self.eng
        try:
            win = float(self.win.get())
        except ValueError:
            win = 20.0
        # in tempo reale basta la coda dello storico che copre la finestra (copia breve, lock tenuto poco);
        # lo storico completo viene copiato solo quando l'oscilloscopio si congela (Scope.freeze -> _full_history)
        n_tail = int(max(win, 1.0) / SIM_PERIOD * 1.2) + 100
        with e.lock:
            fwd, back, brake, ref = e.cmd
            m = e.model
            enc, spd, rpm, trq, cur, ready = m.encoder(), m.speed, m.rpm, m.torque, m.current, m.drive_ready
            hist = e.history[-n_tail:] if not self.scope.frozen else None
            vmax = m.max_mech_speed()
            lev = e.level_on
            self.scope.level_pos = float(m.p.get("level_pos", 0.0)) if m.p.get("level_enabled") else None
        state, cyc, cmax = e.conn_state(), e.cycle_ms, e.cycle_max_ms
        self._check_manual_result()
        on = {"fwd": fwd, "back": back, "brake": brake, "ready": ready, "lev": lev}
        for k, l in self.leds.items():
            l.config(bg="#1a9a3a" if on[k] else "#444")
        self.vals["enc"].set(f"{enc:10.2f}")
        self.vals["ref"].set(f"{(ref if fwd else -ref if back else 0.0):8.1f}")
        self.vals["spd"].set(f"{spd:8.1f}")
        self.vals["rpm"].set(f"{rpm:8.0f}")
        self.vals["trq"].set(f"{trq:7.2f}")
        self.vals["cur"].set(f"{cur:6.2f}")
        if state == "on":
            self.lbl_conn.config(text="  COLLEGATO  ", bg="#1a9a3a")
        elif state == "retry":
            self.lbl_conn.config(text="  COLLEGAMENTO...  " if e._manual_pending else "  RICOLLEGAMENTO...  ", bg="#c07000")
        else:
            self.lbl_conn.config(text="  NON COLLEGATO  ", bg="#aa2222")
        self.btn_conn.config(text="Connetti" if state == "off" else "Scollega")
        self.lbl_cycle.config(text=f"ciclo simulazione {cyc:.1f} ms (max {cmax:.0f} ms nell'ultimo secondo)")
        if e.last_error or e.drops:
            self.lbl_comm.config(text=f"Cadute collegamento: {e.drops}   ricollegamenti automatici: {e.reconnects}   "
                                      f"ultimo errore ({e.last_error_time}): {e.last_error}")
        else:
            self.lbl_comm.config(text="")
        self.lbl_info.config(text=f"Velocità carico a giri nominali: {vmax:.0f} mm/s  (riferimento per Cfg.fMaxSpeed)")
        self.scope.draw(hist, win)
        self.root.after(50, self._update)

    def on_close(self):
        self.eng.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except tk.TclError:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()

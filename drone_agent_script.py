"""
=============================================================
 Autonomous Drone Flight Agent
 Natural Language Drone Control with AI Agents
=============================================================
"""
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
except Exception:
    from collections import abc
    import collections
    collections.MutableMapping = abc.MutableMapping
    from dronekit import connect, VehicleMode, LocationGlobalRelative

import os
os.environ["OPENROUTER_API_KEY"] = "your_api_key_here"
import time, math, threading, queue, datetime, json, logging, operator as op_module
from pymavlink import mavutil
import tcp_relay
from agno.agent import Agent
from agno.tools import Toolkit
from agno.db.sqlite import SqliteDb
from agno.models.openrouter import OpenRouter
from agno.learn import (LearningMachine, LearningMode,
    UserProfileConfig, UserMemoryConfig, SessionContextConfig, DecisionLogConfig)
from agno.compression.manager import CompressionManager
from agno.skills import Skills, LocalSkills
from agno.utils.log import configure_agno_logging
from typing import List, Optional
from pydantic import BaseModel, Field

# ---------------------------------------------------------------
# LOGGING
# ---------------------------------------------------------------
_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
os.makedirs(_LOG_DIR, exist_ok=True)
_today = datetime.datetime.now().strftime("%Y%m%d")
_agno_logger = logging.getLogger("agno")
_agno_logger.setLevel(logging.INFO)
_agno_handler = logging.FileHandler(os.path.join(_LOG_DIR, f"agno_agent_{_today}.log"), encoding="utf-8")
_agno_handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)-8s | %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
_agno_logger.addHandler(_agno_handler)
_agno_logger.propagate = False
configure_agno_logging(custom_default_logger=_agno_logger)
flight_logger = logging.getLogger("flight")
flight_logger.setLevel(logging.DEBUG)
_fh = logging.FileHandler(os.path.join(_LOG_DIR, f"flight_log_{_today}.log"), encoding="utf-8")
_fh.setFormatter(logging.Formatter("%(asctime)s | %(levelname)-8s | %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
_ch = logging.StreamHandler()
_ch.setLevel(logging.WARNING)
_ch.setFormatter(logging.Formatter("%(levelname)s %(message)s"))
flight_logger.addHandler(_fh); flight_logger.addHandler(_ch); flight_logger.propagate = False

def flog(level: str, msg: str):
    getattr(flight_logger, level.lower(), flight_logger.info)(msg)

try:
    from agno.workflow import Workflow, Step, Steps, StepInput, StepOutput
    WORKFLOW_AVAILABLE = True
except ImportError:
    WORKFLOW_AVAILABLE = False

# ---------------------------------------------------------------
# STRUCTURED OUTPUTS
# ---------------------------------------------------------------
class DroneCommand(BaseModel):
    action:    str             = Field(description="Drone action to execute")
    altitude:  Optional[float] = Field(None, description="Target altitude in meters")
    latitude:  Optional[float] = Field(None, description="Target latitude")
    longitude: Optional[float] = Field(None, description="Target longitude")
    radius:    Optional[float] = Field(None, description="Circle radius in meters")
    direction: Optional[str]   = Field(None, description="Movement direction")
    distance:  Optional[float] = Field(None, description="Movement distance in meters")
    speed:     Optional[float] = Field(None, description="Target speed in m/s")
    reason:    str             = Field(default="", description="Why this command was chosen")

class MissionPlan(BaseModel):
    mission_name:           str                = Field(description="Short name for this mission")
    objective:              str                = Field(description="What this mission accomplishes")
    steps:                  List[DroneCommand] = Field(description="Ordered drone commands")
    risk_level:             str                = Field(description="LOW / MEDIUM / HIGH")
    estimated_time_seconds: int                = Field(description="Estimated total time")
    notes:                  str                = Field(default="", description="Special notes")

class SafetyAssessment(BaseModel):
    is_safe:         bool      = Field(description="True if mission is safe")
    risk_level:      str       = Field(description="LOW / MEDIUM / HIGH / CRITICAL")
    issues:          List[str] = Field(description="Identified safety issues")
    recommendations: List[str] = Field(description="Suggested mitigations")
    approved:        bool      = Field(description="Final approval to proceed")

class FlightReport(BaseModel):
    session_id:        str           = Field(description="Session identifier")
    total_commands:    int           = Field(description="Commands executed")
    commands_executed: List[str]     = Field(description="All commands run")
    max_altitude_m:    float         = Field(description="Peak altitude reached")
    duration_seconds:  int           = Field(description="Session length in seconds")
    battery_start:     Optional[int] = Field(None, description="Battery at start")
    battery_end:       Optional[int] = Field(None, description="Battery now")
    incidents:         List[str]     = Field(description="Blocked or failed commands")
    summary:           str           = Field(description="Human-readable narrative")

# ---------------------------------------------------------------
# PRESET LOCATIONS
# ---------------------------------------------------------------
PRESET_LOCATIONS = {
    "home":        {"lat": -35.363261, "lon": 149.165230, "description": "SITL home / launch point"},
    "airfield":    {"lat": -35.362749, "lon": 149.165353, "description": "Canberra Airfield"},
    "runway 35":   {"lat": -35.363328, "lon": 149.165223, "description": "Runway 35"},
    "runway 17":   {"lat": -35.362227, "lon": 149.165074, "description": "Runway 17"},
    "hospital":    {"lat": -35.354167, "lon": 149.150560, "description": "Mugga Mugga Hospital"},
    "prison":      {"lat": -35.371077, "lon": 149.172684, "description": "West Jerrabomberra Prison"},
    "camp a":      {"lat": -35.360338, "lon": 149.151874, "description": "West Jerrabomberra Camp A"},
    "camp b":      {"lat": -35.361530, "lon": 149.154562, "description": "West Jerrabomberra Location 2"},
    "reserve":     {"lat": -35.366030, "lon": 149.150095, "description": "West Jerrabomberra Reserve"},
    "residence 1": {"lat": -35.357340, "lon": 149.170626, "description": "Jerrabomberra Residence 1"},
    "residence 2": {"lat": -35.346840, "lon": 149.154976, "description": "Jerrabomberra North Residence"},
    "creek south": {"lat": -35.363393, "lon": 149.175728, "description": "Jerrabomberra Creek South"},
    "location 1":  {"lat": -35.364759, "lon": 149.152459, "description": "West Jerrabomberra Location 1"},
}

# ---------------------------------------------------------------
# VEHICLE CONNECTION
# ---------------------------------------------------------------
print("Connecting to vehicle...")
vehicle = connect("tcp:127.0.0.1:5763", wait_ready=True, baud=57600, rate=60)
print("Vehicle connected.")
while vehicle.location.local_frame.north is None:
    time.sleep(1); print("Waiting for local frame...")
print("Local frame ready.")

# ---------------------------------------------------------------
# TCP RELAY
# ---------------------------------------------------------------
relay = tcp_relay.TCP_Relay()

def vehicle_to_unreal(v, z_invert=True, scale=100):
    return {"n": v.location.local_frame.north*scale, "e": v.location.local_frame.east*scale,
            "d": v.location.local_frame.down*scale*(-1 if z_invert else 1),
            "roll": math.degrees(v.attitude.roll), "pitch": math.degrees(v.attitude.pitch),
            "yaw": math.degrees(v.attitude.yaw)}

def unreal_stream_loop():
    while True:
        data = vehicle_to_unreal(vehicle); fields = [0.0] * relay.num_fields
        fields[0]=data["n"]; fields[1]=data["e"]; fields[2]=data["d"]
        fields[3]=data["roll"]; fields[4]=data["pitch"]; fields[5]=data["yaw"]
        relay.message = tcp_relay.create_fields_string(fields); time.sleep(1/60)

threading.Thread(target=unreal_stream_loop, daemon=True).start()

# ---------------------------------------------------------------
# STATE MANAGEMENT
# ---------------------------------------------------------------
SESSION_START = datetime.datetime.now()
SESSION_ID    = f"drone-{SESSION_START.strftime('%H%M%S')}"
flog("info", f"SESSION START — ID: {SESSION_ID} | Vehicle connected at tcp:127.0.0.1:5763")

# WP_YAW_BEHAVIOR = 1 — face next waypoint.
try:
    vehicle.parameters["WP_YAW_BEHAVIOR"] = 1
    flog("info", "WP_YAW_BEHAVIOR=1 set — drone will face direction of travel")
except Exception as e:
    flog("warning", f"Could not set WP_YAW_BEHAVIOR: {e}")

mission_state = {
    "phase": "idle", "flight_log": [], "max_altitude": 0.0,
    "battery_start": None, "battery_current": None, "incidents": [],
    "last_command": {}, "pending_mission": [], "current_mission": None,
}
try: mission_state["battery_start"] = vehicle.battery.level
except: pass

# ---------------------------------------------------------------
# ACTIVE NAVIGATION TARGET TRACKING (for speed-change re-issue)
# ---------------------------------------------------------------
# Tracks the most recent simple_goto() target so that a mid-flight
# speed change can re-issue simple_goto() to the SAME target,
# forcing ArduPilot to recompute the velocity profile using the
# new WPNAV_SPEED. Without this, DO_CHANGE_SPEED updates the
# parameter but the already-active position-controller leg keeps
# its original speed shaping until the leg completes.
_active_target_lock = threading.Lock()
_active_nav_target = {"lat": None, "lon": None, "alt": None, "active": False}

def _set_active_target(lat, lon, alt):
    with _active_target_lock:
        _active_nav_target["lat"] = lat
        _active_nav_target["lon"] = lon
        _active_nav_target["alt"] = alt
        _active_nav_target["active"] = True

def _clear_active_target():
    with _active_target_lock:
        _active_nav_target["active"] = False

def _get_active_target():
    with _active_target_lock:
        if _active_nav_target["active"]:
            return (_active_nav_target["lat"], _active_nav_target["lon"], _active_nav_target["alt"])
        return None

# ---------------------------------------------------------------
# FILESYSTEM TOOLKIT
# ---------------------------------------------------------------
class FilesystemToolkit(Toolkit):
    def __init__(self):
        self._missions_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "missions")
        self._reports_dir  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")
        os.makedirs(self._missions_dir, exist_ok=True)
        os.makedirs(self._reports_dir,  exist_ok=True)
        super().__init__(name="filesystem",
            tools=[self.list_missions, self.read_mission, self.save_report, self.list_reports])

    def list_missions(self) -> str:
        "List all available mission files in the missions/ folder."
        files = [f for f in os.listdir(self._missions_dir) if f.endswith((".txt",".json"))]
        if not files: return "No mission files found. Drop .txt or .json files into missions/."
        return "Available missions:\n" + "\n".join(f"  - {f}" for f in sorted(files))

    def read_mission(self, filename: str) -> str:
        "Read a mission file from missions/. Use when user says load mission X."
        filename = os.path.basename(filename)
        path = os.path.join(self._missions_dir, filename)
        if not os.path.exists(path):
            return f"Mission file not found: {filename}. Available: {os.listdir(self._missions_dir)}"
        try:
            with open(path, "r", encoding="utf-8") as f: content = f.read()
            flog("info", f"FILES: read mission {filename}")
            return f"Mission file '{filename}':\n{content}"
        except Exception as e: return f"Error reading {filename}: {e}"

    def save_report(self, content: str, filename: str = "") -> str:
        "Write a flight report to reports/. Use when user says save a report."
        if not filename: filename = f"report_{SESSION_ID}.txt"
        filename = os.path.basename(filename)
        if not filename.endswith((".txt",".json",".md")): filename += ".txt"
        path = os.path.join(self._reports_dir, filename)
        try:
            with open(path, "w", encoding="utf-8") as f: f.write(content)
            flog("info", f"FILES: saved report {filename}")
            return f"Report saved to reports/{filename}"
        except Exception as e: return f"Error saving report: {e}"

    def list_reports(self) -> str:
        "List all saved flight reports in the reports/ folder."
        files = [f for f in os.listdir(self._reports_dir) if f.endswith((".txt",".json",".md"))]
        if not files: return "No reports saved yet."
        return "Saved reports:\n" + "\n".join(f"  - {f}" for f in sorted(files, reverse=True))

_filesystem_toolkit = FilesystemToolkit()
print("[FILES] missions/ and reports/ ready next to script.")

# ---------------------------------------------------------------
# COMMAND QUEUE + FLAGS
# ---------------------------------------------------------------
command_queue      = queue.Queue()
stop_flag          = threading.Event()
_navigation_queued = threading.Event()
_hold_active       = threading.Event()   # Prevents executor from clearing stop_flag while holding

def _clear_queue():
    pending = []; cleared = 0
    while not command_queue.empty():
        try: pending.append(command_queue.get_nowait()); command_queue.task_done(); cleared += 1
        except queue.Empty: break
    if cleared:
        print(f"[QUEUE] Cleared {cleared} pending command(s).")
        flog("warning", f"QUEUE cleared: {cleared} command(s) dropped")
    mission_state["pending_mission"] = pending

# ---------------------------------------------------------------
# CONDITION MONITOR
# ---------------------------------------------------------------
_OPERATORS = {
    "==": op_module.eq, "!=": op_module.ne, "<": op_module.lt, "<=": op_module.le,
    ">": op_module.gt, ">=": op_module.ge,
    "in": lambda a,b: a in b, "not in": lambda a,b: a not in b,
}
CONDITION_FIELDS = ["rel_alt","battery_level","battery_voltage","groundspeed","armed","mode","airborne","yaw"]

class ConditionWatch:
    def __init__(self, field, operator, value, then_action, then_params=None, label=""):
        self.field=field; self.operator=operator; self.value=value
        self.then_action=then_action; self.then_params=then_params or {}
        self.label=label or f"{field} {operator} {value} -> {then_action}"; self.triggered=False
    def evaluate(self, state):
        sv=state.get(self.field)
        if sv is None: return False
        op_func=_OPERATORS.get(self.operator)
        if not op_func: return False
        try: return op_func(sv, self.value)
        except: return False

class ConditionMonitor:
    def __init__(self):
        self._watches=[]; self._lock=threading.Lock()
        threading.Thread(target=self._tick, daemon=True).start()
    def add_watch(self, watch):
        with self._lock: self._watches.append(watch)
        print(f"[CONDITION] Watching: {watch.label}"); flog("info", f"CONDITION registered: {watch.label}")
    def clear_watches(self):
        with self._lock: self._watches.clear()
        print("[CONDITION] All watches cleared.")
    def list_watches(self):
        with self._lock:
            if not self._watches: return "No active condition watches."
            return "\n".join(f"  - {w.label}" for w in self._watches)
    def _get_state(self):
        try:
            loc=vehicle.location.global_relative_frame; batt=vehicle.battery
            return {"rel_alt": round(loc.alt,1) if loc.alt else 0.0, "lat": loc.lat, "lon": loc.lon,
                    "battery_level": batt.level or 0, "battery_voltage": round(batt.voltage or 0.0,2),
                    "groundspeed": round(vehicle.groundspeed,1), "armed": vehicle.armed,
                    "mode": vehicle.mode.name, "yaw": round(math.degrees(vehicle.attitude.yaw),1),
                    "airborne": (loc.alt or 0)>1.0, "time": time.time()}
        except: return {}
    def _tick(self):
        while True:
            time.sleep(0.5)
            with self._lock:
                if not self._watches: continue
                state=self._get_state(); remaining=[]
                for watch in self._watches:
                    if watch.triggered: continue
                    if watch.evaluate(state):
                        print(f"\n[CONDITION] TRIGGERED: {watch.label}")
                        watch.triggered=True; stop_flag.set(); _clear_queue()
                        cmd={"action": watch.then_action}; cmd.update(watch.then_params)
                        command_queue.put(cmd)
                    else: remaining.append(watch)
                self._watches=remaining

condition_monitor = ConditionMonitor()

# ---------------------------------------------------------------
# KNOWLEDGE BASE
# ---------------------------------------------------------------
_loc_lines = "\n".join(
    f"  {name}: {data['description']} (lat={data['lat']}, lon={data['lon']})"
    for name, data in PRESET_LOCATIONS.items())
DRONE_KNOWLEDGE = (
    "ENVIRONMENT: SITL simulation. Battery is NOTIONAL/FAKE — ignore completely.\n"
    f"PRESET LOCATIONS:\n{_loc_lines}\n"
    "ALTITUDE: Max 120m AGL. Min 2m. Cruise 15-50m.\n"
    "SPEED: Max 30 m/s. Cruise 5-8 m/s. Use higher speeds for urgent reposition.\n"
    "CIRCLE: Min radius 5m. Recommended 20-50m.\n"
    "FLIGHT MODES: GUIDED / LOITER / AUTO / RTL / LAND"
)

# ---------------------------------------------------------------
# EXECUTOR HELPERS
# ---------------------------------------------------------------
def _wait_for_arrival(target_lat, target_lon, target_alt, tolerance_m=4.0, timeout=60):
    R=6378137.0; start=time.time()
    while time.time()-start < timeout:
        if stop_flag.is_set():
            print("[DRONE] Arrival interrupted."); flog("warning","Arrival interrupted by stop_flag"); return False
        loc=vehicle.location.global_relative_frame
        if loc.lat is None: time.sleep(0.5); continue
        d_lat=math.radians(loc.lat-target_lat)*R
        d_lon=math.radians(loc.lon-target_lon)*R*math.cos(math.radians(target_lat))
        d_alt=abs(loc.alt-target_alt)
        if math.sqrt(d_lat**2+d_lon**2+d_alt**2)<=tolerance_m: return True
        time.sleep(0.3)
    print("[DRONE] Arrival timeout — proceeding."); flog("warning","Arrival timeout"); return False

def _haversine_m(lat1, lon1, lat2, lon2):
    R=6378137.0; dlat=math.radians(lat2-lat1); dlon=math.radians(lon2-lon1)
    a=(math.sin(dlat/2)**2+math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2)
    return R*2*math.asin(math.sqrt(a))

def _body_frame_target(distance, direction_deg):
    R=6378137.0; cur=vehicle.location.global_relative_frame
    bearing=math.radians(direction_deg); lat1=math.radians(cur.lat); lon1=math.radians(cur.lon)
    lat2=math.asin(math.sin(lat1)*math.cos(distance/R)+math.cos(lat1)*math.sin(distance/R)*math.cos(bearing))
    lon2=lon1+math.atan2(math.sin(bearing)*math.sin(distance/R)*math.cos(lat1),
                         math.cos(distance/R)-math.sin(lat1)*math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)

def _resolve_direction(direction_str):
    d=direction_str.lower().strip(); yaw=math.degrees(vehicle.attitude.yaw)%360
    compass={"north":0,"south":180,"east":90,"west":270}
    body={"forward":yaw,"backward":(yaw+180)%360,"back":(yaw+180)%360,
          "left":(yaw-90)%360,"right":(yaw+90)%360}
    if d in compass: return compass[d], False
    if d in body:    return body[d], True
    return 0, False

def _fly_circle(radius=20, altitude=None, points=12):
    cur_lat=vehicle.location.global_relative_frame.lat
    cur_lon=vehicle.location.global_relative_frame.lon
    cur_alt=altitude or vehicle.location.global_relative_frame.alt; R=6378137.0
    print(f"[DRONE] Circle: r={radius}m alt={cur_alt}m pts={points}")
    for i in range(points):
        if stop_flag.is_set(): print("[DRONE] Circle interrupted."); return
        angle=(2*math.pi/points)*i
        d_lat=(radius*math.cos(angle))/R
        d_lon=(radius*math.sin(angle))/(R*math.cos(math.radians(cur_lat)))
        target_lat = cur_lat+math.degrees(d_lat)
        target_lon = cur_lon+math.degrees(d_lon)
        _set_active_target(target_lat, target_lon, cur_alt)
        vehicle.simple_goto(LocationGlobalRelative(target_lat, target_lon, cur_alt))
        for _ in range(30):
            if stop_flag.is_set(): return
            time.sleep(0.1)
    print("[DRONE] Circle complete.")

def _log(action, details=None):
    loc=vehicle.location.global_relative_frame
    entry={"time": datetime.datetime.now().strftime("%H:%M:%S"), "action": action,
           "lat": round(loc.lat,6) if loc.lat else None,
           "lon": round(loc.lon,6) if loc.lon else None,
           "alt": round(loc.alt,1) if loc.alt else None, "details": details or {}}
    mission_state["flight_log"].append(entry)
    if loc.alt: mission_state["max_altitude"]=max(mission_state["max_altitude"],loc.alt)
    try: mission_state["battery_current"]=vehicle.battery.level
    except: pass

def _set_mode_confirmed(mode_name, retries=5):
    for _ in range(retries):
        vehicle.mode=VehicleMode(mode_name)
        for _ in range(10):
            if vehicle.mode.name==mode_name: return True
            time.sleep(0.2)
    return False

def _set_speed_mavlink(speed_ms: float, retries: int = 3, reissue_target: bool = True):
    """
    Change groundspeed (WPNAV_SPEED) in real time and make it actually take effect.

    Two things happen:

    1. We send MAV_CMD_DO_CHANGE_SPEED (speed_type=1, groundspeed) AND directly
       write the WPNAV_SPEED parameter (in cm/s). ArduPilot Copter navigates by
       WPNAV_SPEED — airspeed (speed_type=0 / vehicle.airspeed) is largely
       ignored by multirotors. Writing the parameter directly is the most
       reliable path in SITL; the COMMAND_LONG is sent as well for vehicles/
       firmwares that prefer that route.

    2. CRITICAL: if the drone is already mid-leg toward a simple_goto() target,
       ArduPilot's position controller has ALREADY shaped the velocity profile
       for that leg using the OLD WPNAV_SPEED. Just updating the parameter does
       NOT retroactively change the active leg's speed. So we re-issue
       simple_goto() to the SAME active target — this forces ArduPilot to
       recompute the velocity profile using the NEW WPNAV_SPEED, and the
       groundspeed changes immediately without altering the destination or
       interrupting the mission queue.
    """
    speed_ms = min(30.0, max(0.5, float(speed_ms)))
    speed_cms = int(speed_ms * 100)

    # 1a. Direct parameter write (most reliable in SITL)
    try:
        vehicle.parameters['WPNAV_SPEED'] = speed_cms
    except Exception as e:
        flog("warning", f"_set_speed_mavlink: WPNAV_SPEED param write failed: {e}")

    # 1b. MAVLink DO_CHANGE_SPEED (groundspeed) — belt and suspenders
    for _ in range(retries):
        msg = vehicle.message_factory.command_long_encode(
            0, 0,                                   # target system, target component (0,0 = autoresolve)
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,                                      # confirmation
            1,                                      # speed type: 1 = groundspeed (Copter uses this)
            speed_ms,                               # new speed m/s
            -1,                                     # throttle: -1 = no change
            0, 0, 0, 0)                             # unused
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.1)

    flog("info", f"_set_speed_mavlink: groundspeed -> {speed_ms} m/s (WPNAV_SPEED={speed_cms}cm/s, {retries} MAVLink sends)")

    # 2. Re-issue simple_goto to the active target so the CURRENT leg
    #    recomputes its velocity profile with the new speed.
    if reissue_target:
        target = _get_active_target()
        if target is not None and vehicle.mode.name == "GUIDED" and not _hold_active.is_set():
            tlat, tlon, talt = target
            try:
                vehicle.simple_goto(LocationGlobalRelative(tlat, tlon, talt))
                flog("info", f"_set_speed_mavlink: re-issued simple_goto to active target "
                              f"({tlat:.6f},{tlon:.6f},{talt}m) to apply new speed mid-leg")
            except Exception as e:
                flog("warning", f"_set_speed_mavlink: could not re-issue goto: {e}")

    return speed_ms

# ---------------------------------------------------------------
# EXECUTOR THREAD
# ---------------------------------------------------------------
def executor_loop():
    while True:
        cmd=command_queue.get(); action=cmd.get("action")
        try:
            # Only clear stop_flag for nav actions when NOT actively holding
            if action in ("arm","takeoff","goto","change_altitude","adjust_altitude",
                          "move","keep_moving","fly_until_distance","circle",
                          "flyover","rtl","land","set_mode","set_speed"):
                if not _hold_active.is_set():   # Guard: don't clear stop if hold is active
                    stop_flag.clear()
                mission_state["last_command"] = cmd
                mission_state["phase"] = "executing"

            if action=="arm":
                _set_mode_confirmed("GUIDED"); vehicle.armed=True
                while not vehicle.armed: time.sleep(0.5)
                _log("arm"); print("[DRONE] Armed."); flog("info","ARMED — motors armed, mode GUIDED")

            elif action=="disarm":
                vehicle.armed=False; _clear_active_target(); _log("disarm"); print("[DRONE] Disarmed.")

            elif action=="takeoff":
                alt=cmd.get("altitude",10); _set_mode_confirmed("GUIDED"); vehicle.armed=True
                while not vehicle.armed: time.sleep(0.5)
                vehicle.simple_takeoff(alt)
                cur = vehicle.location.global_relative_frame
                _set_active_target(cur.lat, cur.lon, alt)
                while True:
                    if stop_flag.is_set(): break
                    if vehicle.location.global_relative_frame.alt>=alt*0.95: break
                    time.sleep(0.5)
                _log("takeoff",{"altitude":alt})
                if not stop_flag.is_set():
                    print(f"[DRONE] Reached {alt}m."); flog("info",f"TAKEOFF complete — reached {alt}m")

            elif action=="goto":
                tlat,tlon,talt=cmd["latitude"],cmd["longitude"],cmd["altitude"]
                if vehicle.mode.name!="GUIDED": _set_mode_confirmed("GUIDED")
                _set_active_target(tlat, tlon, talt)
                vehicle.simple_goto(LocationGlobalRelative(tlat,tlon,talt))
                print(f"[DRONE] Going to {tlat:.5f}, {tlon:.5f} @ {talt}m")
                flog("info",f"GOTO {tlat:.5f},{tlon:.5f} @ {talt}m")
                _wait_for_arrival(tlat,tlon,talt); _log("goto",{"lat":tlat,"lon":tlon,"alt":talt})
                if not stop_flag.is_set(): print("[DRONE] Arrived.")

            elif action=="change_altitude":
                cur=vehicle.location.global_relative_frame; new_alt=cmd["altitude"]
                _set_active_target(cur.lat, cur.lon, new_alt)
                vehicle.simple_goto(LocationGlobalRelative(cur.lat,cur.lon,new_alt))
                print(f"[DRONE] Changing altitude to {new_alt}m"); flog("info",f"ALTITUDE target: {new_alt}m")
                while True:
                    if stop_flag.is_set(): break
                    if abs(vehicle.location.global_relative_frame.alt-new_alt)<=1.0:
                        print(f"[DRONE] Altitude reached: {new_alt}m"); break
                    time.sleep(0.5)
                _log("change_altitude",{"altitude":new_alt})

            elif action=="adjust_altitude":
                cur_alt=vehicle.location.global_relative_frame.alt
                new_alt=max(2,min(120,cur_alt+cmd["change"]))
                cur=vehicle.location.global_relative_frame
                _set_active_target(cur.lat, cur.lon, new_alt)
                vehicle.simple_goto(LocationGlobalRelative(cur.lat,cur.lon,new_alt))
                _log("adjust_altitude",{"change":cmd["change"],"new_alt":round(new_alt,1)})

            elif action=="move":
                d,dist=cmd["direction"].lower(),cmd["distance"]; alt=cmd.get("altitude")
                cur=vehicle.location.global_relative_frame; cur_alt=alt if alt else cur.alt
                bearing,_=_resolve_direction(d); nlat,nlon=_body_frame_target(dist,bearing)
                _set_active_target(nlat, nlon, cur_alt)
                vehicle.simple_goto(LocationGlobalRelative(nlat,nlon,cur_alt))
                _wait_for_arrival(nlat,nlon,cur_alt); _log("move",{"direction":d,"distance":dist})

            elif action=="circle":
                mission_state["last_command"]=cmd
                _fly_circle(radius=cmd.get("radius",20),
                            altitude=cmd.get("altitude",vehicle.location.global_relative_frame.alt),
                            points=cmd.get("points",12))
                _log("circle",{"radius":cmd.get("radius",20)})

            elif action=="flyover":
                tlat=cmd["latitude"]; tlon=cmd["longitude"]; alt=cmd.get("altitude",50)
                radius=cmd.get("radius",80); name=cmd.get("location_name","target")
                print(f"[DRONE] Flyover of {name} — approaching at {alt}m")
                if vehicle.mode.name!="GUIDED": _set_mode_confirmed("GUIDED")
                _set_active_target(tlat, tlon, alt)
                vehicle.simple_goto(LocationGlobalRelative(tlat,tlon,alt))
                _wait_for_arrival(tlat,tlon,alt,tolerance_m=30,timeout=180)
                if not stop_flag.is_set():
                    print(f"[DRONE] Over {name} — orbiting"); _fly_circle(radius=radius,altitude=alt,points=16)
                if not stop_flag.is_set(): _fly_circle(radius=radius,altitude=alt,points=16)
                _log("flyover",{"location":name,"lat":tlat,"lon":tlon,"alt":alt})
                if not stop_flag.is_set(): print(f"[DRONE] Flyover of {name} complete.")

            elif action=="rtl":
                home=vehicle.home_location; cur_alt=vehicle.location.global_relative_frame.alt
                if home:
                    _set_mode_confirmed("GUIDED")
                    _set_active_target(home.lat, home.lon, cur_alt)
                    vehicle.simple_goto(LocationGlobalRelative(home.lat,home.lon,cur_alt))
                    print(f"[DRONE] Flying home at {round(cur_alt,1)}m..."); flog("info",f"RTL — flying home at {round(cur_alt,1)}m")
                    _wait_for_arrival(home.lat,home.lon,cur_alt,tolerance_m=5.0,timeout=180)
                    if not stop_flag.is_set(): _set_mode_confirmed("LAND"); print("[DRONE] Home reached — landing.")
                    flog("info","RTL complete — home reached, landing")
                else: _set_mode_confirmed("RTL"); print("[DRONE] RTL mode activated.")
                _log("rtl")

            elif action=="land":
                _clear_active_target()
                _set_mode_confirmed("LAND"); print("[DRONE] Landing."); flog("info","LAND command executing")
                while vehicle.location.global_relative_frame.alt>0.5:
                    if stop_flag.is_set():
                        _set_mode_confirmed("GUIDED"); cur=vehicle.location.global_relative_frame
                        _set_active_target(cur.lat, cur.lon, cur.alt)
                        vehicle.simple_goto(LocationGlobalRelative(cur.lat,cur.lon,cur.alt))
                        print("[DRONE] Landing interrupted — holding."); break
                    time.sleep(0.5)
                _log("land")
                if not stop_flag.is_set():
                    mission_state["phase"]="idle"; print("[DRONE] Landed."); flog("info","LANDED successfully")

            elif action=="hold":
                # Stay in GUIDED — issue simple_goto to current position so ArduPilot
                # has a "go here" target that is already satisfied and just hovers.
                # Never switch modes; LOITER causes crashes in SITL.
                _hold_active.set()
                stop_flag.set()
                try:
                    cur = vehicle.location.global_relative_frame
                    # Ensure we are in GUIDED so simple_goto is accepted
                    if vehicle.mode.name != "GUIDED":
                        _set_mode_confirmed("GUIDED")
                    # Command the drone to its current position — it will hover in place
                    _set_active_target(cur.lat, cur.lon, cur.alt)
                    vehicle.simple_goto(LocationGlobalRelative(cur.lat, cur.lon, cur.alt))
                    print(f"[DRONE] Holding position in GUIDED @ {round(cur.alt, 1)}m.")
                    flog("info", f"HOLD — hovering in GUIDED @ {round(cur.alt, 1)}m")
                except Exception as e:
                    flog("warning", f"HOLD: could not issue hold goto: {e}")
                _log("hold")
                # _hold_active stays set until resume explicitly clears it

            elif action=="set_speed":
                speed=cmd.get("speed",5)
                applied=_set_speed_mavlink(speed)
                _log("set_speed",{"speed":applied})
                print(f"[DRONE] Speed -> {applied} m/s (groundspeed, DO_CHANGE_SPEED + WPNAV_SPEED).")
                flog("info", f"SET_SPEED: {applied} m/s via DO_CHANGE_SPEED + WPNAV_SPEED")

            elif action=="set_mode":
                ALLOWED=["GUIDED","LOITER","AUTO","RTL","LAND","STABILIZE","ALT_HOLD"]
                mode_name=cmd.get("mode","GUIDED").upper()
                if mode_name not in ALLOWED: print(f"[DRONE] Mode '{mode_name}' not allowed.")
                else:
                    _set_mode_confirmed(mode_name); _log("set_mode",{"mode":mode_name})
                    print(f"[DRONE] Mode -> {mode_name}."); flog("info",f"MODE changed to {mode_name}")

            elif action=="keep_moving":
                d=cmd["direction"].lower(); step=200; bearing,_=_resolve_direction(d)
                print(f"[DRONE] Continuous flight: {d} (bearing {bearing:.0f}deg) — say stop to interrupt.")
                _log("keep_moving",{"direction":d})
                while not stop_flag.is_set():
                    cur=vehicle.location.global_relative_frame; cur_alt=cur.alt
                    nlat,nlon=_body_frame_target(step,bearing)
                    _set_active_target(nlat, nlon, cur_alt)
                    vehicle.simple_goto(LocationGlobalRelative(nlat,nlon,cur_alt))
                    for _ in range(5):
                        if stop_flag.is_set(): break
                        time.sleep(0.1)
                print("[DRONE] Continuous movement stopped."); flog("info","keep_moving stopped")

            elif action=="fly_until_distance":
                d=cmd["direction"].lower(); target_m=cmd["distance_m"]; bearing,_=_resolve_direction(d)
                start=vehicle.location.global_relative_frame; cur_alt=start.alt; step=100
                print(f"[DRONE] Flying {d} until {target_m}m from start point.")
                _log("fly_until_distance",{"direction":d,"distance_m":target_m})
                while not stop_flag.is_set():
                    cur=vehicle.location.global_relative_frame
                    gone=_haversine_m(start.lat,start.lon,cur.lat,cur.lon)
                    if gone>=target_m:
                        stop_flag.set(); print(f"[DRONE] Reached {round(gone,1)}m — stopping.")
                        flog("info",f"fly_until_distance: reached {round(gone,1)}m — auto-stopped"); break
                    nlat,nlon=_body_frame_target(step,bearing)
                    _set_active_target(nlat, nlon, cur_alt)
                    vehicle.simple_goto(LocationGlobalRelative(nlat,nlon,cur_alt))
                    for _ in range(5):
                        if stop_flag.is_set(): break
                        time.sleep(0.1)

        except Exception as e:
            mission_state["incidents"].append(
                f"[{datetime.datetime.now().strftime('%H:%M:%S')}] {action} error: {e}")
            print(f"[EXECUTOR ERROR] {e}"); flog("error",f"EXECUTOR ERROR: {e}")
        command_queue.task_done()

threading.Thread(target=executor_loop, daemon=True).start()

# ---------------------------------------------------------------
# LIVE CONTEXT
# ---------------------------------------------------------------
def get_live_context():
    try:
        loc=vehicle.location.global_relative_frame; att=vehicle.attitude; batt=vehicle.battery
        return (f"Mode: {vehicle.mode.name} | Armed: {vehicle.armed}\n"
                f"GPS: lat={round(loc.lat,6)}, lon={round(loc.lon,6)}, alt={round(loc.alt,1)}m\n"
                f"Attitude: roll={round(math.degrees(att.roll),1)} pitch={round(math.degrees(att.pitch),1)} yaw={round(math.degrees(att.yaw),1)}\n"
                f"Groundspeed: {round(vehicle.groundspeed,1)} m/s\n"
                f"Battery: {batt.level}% | {batt.voltage}V\n"
                f"Phase: {mission_state['phase']} | Commands this session: {len(mission_state['flight_log'])}")
    except: return "[telemetry unavailable]"

def _safety_check(action, altitude=None):
    if altitude is not None:
        if altitude>120: return False, f"Altitude {altitude}m exceeds 120m limit."
        if action not in ("takeoff","arm") and altitude<2: return False, f"Altitude {altitude}m below 2m minimum."
    return True, "ok"

# ---------------------------------------------------------------
# DRONE TOOLKIT
# ---------------------------------------------------------------
class DroneToolkit(Toolkit):
    def __init__(self):
        super().__init__(name="drone_toolkit", tools=[
            self.arm_drone, self.disarm_drone, self.takeoff, self.goto_waypoint,
            self.get_location, self.fly_over, self.change_altitude, self.adjust_altitude,
            self.move_direction, self.keep_moving, self.fly_until_distance, self.fly_circle,
            self.set_speed, self.set_flight_mode, self.return_to_launch, self.land,
            self.hold_position, self.resume_mission, self.watch_condition,
            self.clear_conditions, self.list_conditions,
            self.get_status, self.get_battery, self.get_position, self.get_flight_summary,
        ])

    def arm_drone(self) -> str:
        "Arms the drone and switches to GUIDED mode. Always call before takeoff."
        safe,reason=_safety_check("arm")
        if not safe: return f"[BLOCKED] {reason}"
        command_queue.put({"action":"arm"}); _navigation_queued.set()
        return "arm_drone queued. Continue with next step."

    def disarm_drone(self) -> str:
        "Disarms the drone. Only works on the ground."
        alt=vehicle.location.global_relative_frame.alt
        if alt is not None and alt>0.5: return f"[BLOCKED] Airborne at {round(alt,1)}m — land first."
        command_queue.put({"action":"disarm"}); return "disarm_drone queued."

    def takeoff(self, altitude: float) -> str:
        "Take off to altitude in meters (2-120)."
        safe,reason=_safety_check("takeoff",altitude)
        if not safe: return f"[BLOCKED] {reason}"
        command_queue.put({"action":"takeoff","altitude":altitude}); _navigation_queued.set()
        return f"takeoff({altitude}m) queued. Continue with next step."

    def get_location(self, name: str) -> str:
        "Look up a named preset location and return its coordinates."
        key=name.lower().strip(); matches=[k for k in PRESET_LOCATIONS if key in k or k in key]
        if not matches: return f"Unknown location '{name}'. Available: {', '.join(PRESET_LOCATIONS.keys())}"
        loc=PRESET_LOCATIONS[matches[0]]
        return f"{loc['description']}: lat={loc['lat']}, lon={loc['lon']}"

    def fly_over(self, location_name: str, altitude: float = 50, orbit_radius: float = 80) -> str:
        "Full flyover of a named location — approach two orbits. Use for: flyover, survey, scout, recon."
        key=location_name.lower().strip(); matches=[k for k in PRESET_LOCATIONS if key in k or k in key]
        if not matches: return f"Unknown location '{location_name}'."
        safe,reason=_safety_check("flyover",altitude)
        if not safe: return f"[BLOCKED] {reason}"
        loc=PRESET_LOCATIONS[matches[0]]
        command_queue.put({"action":"flyover","latitude":loc["lat"],"longitude":loc["lon"],
                           "altitude":altitude,"radius":orbit_radius,"location_name":matches[0]})
        _navigation_queued.set()
        return f"fly_over({matches[0]}, {altitude}m) queued. Continue with next step."

    def goto_waypoint(self, latitude: float, longitude: float, altitude: float) -> str:
        "Fly to GPS coordinates at altitude."
        safe,reason=_safety_check("goto",altitude)
        if not safe: return f"[BLOCKED] {reason}"
        command_queue.put({"action":"goto","latitude":latitude,"longitude":longitude,"altitude":altitude})
        _navigation_queued.set()
        return f"goto_waypoint({latitude:.5f},{longitude:.5f},{altitude}m) queued. Continue with next step."

    def change_altitude(self, altitude: float) -> str:
        "Change to an absolute altitude in meters."
        safe,reason=_safety_check("change_altitude",altitude)
        if not safe: return f"[BLOCKED] {reason}"
        command_queue.put({"action":"change_altitude","altitude":altitude}); _navigation_queued.set()
        return f"change_altitude({altitude}m) queued. Continue with next step."

    def adjust_altitude(self, change: float) -> str:
        "Relative altitude change. Positive = up, negative = down."
        cur=vehicle.location.global_relative_frame.alt; new=cur+change
        safe,reason=_safety_check("change_altitude",new)
        if not safe: return f"[BLOCKED] {reason}"
        command_queue.put({"action":"adjust_altitude","change":change}); _navigation_queued.set()
        return f"adjust_altitude({'+' if change>0 else ''}{change}m) queued. Continue with next step."

    def move_direction(self, direction: str, distance: float, altitude: float = None) -> str:
        "Move a set distance. Compass: north/south/east/west. Body-frame: forward/backward/left/right."
        safe,reason=_safety_check("move")
        if not safe: return f"[BLOCKED] {reason}"
        command_queue.put({"action":"move","direction":direction,"distance":distance,"altitude":altitude})
        _navigation_queued.set()
        return f"move_direction({direction}, {distance}m) queued. Continue with next step."

    def keep_moving(self, direction: str) -> str:
        "Fly continuously in a direction until told to stop. Supports compass and body-frame."
        safe,reason=_safety_check("move")
        if not safe: return f"[BLOCKED] {reason}"
        command_queue.put({"action":"keep_moving","direction":direction}); _navigation_queued.set()
        return f"keep_moving({direction}) queued. Flying until stop command."

    def fly_until_distance(self, direction: str, distance_m: float) -> str:
        "Fly until reaching distance_m from current position then auto-stop."
        safe,reason=_safety_check("move")
        if not safe: return f"[BLOCKED] {reason}"
        command_queue.put({"action":"fly_until_distance","direction":direction,"distance_m":distance_m})
        _navigation_queued.set()
        return f"fly_until_distance({direction}, {distance_m}m) queued. Will stop automatically."

    def fly_circle(self, radius: float = 20, altitude: float = None, points: int = 12) -> str:
        "Orbit current position. Use for: circle, orbit, fly around here."
        cur_alt=altitude if altitude is not None else vehicle.location.global_relative_frame.alt
        safe,reason=_safety_check("circle",cur_alt)
        if not safe: return f"[BLOCKED] {reason}"
        command_queue.put({"action":"circle","radius":radius,"altitude":cur_alt,"points":points})
        _navigation_queued.set()
        return f"fly_circle(r={radius}m, alt={cur_alt}m) queued. Continue with next step."

    def set_speed(self, speed: float) -> str:
        "Set groundspeed in m/s (0.5-30). Applied instantly — does not interrupt the mission."
        if speed<0.5 or speed>30: return f"[BLOCKED] Speed {speed} out of range (0.5-30 m/s)."
        applied=_set_speed_mavlink(speed)
        return f"Speed set to {applied} m/s via MAVLink — mission continues."

    def set_flight_mode(self, mode: str) -> str:
        "Switch flight mode: GUIDED/LOITER/AUTO/RTL/LAND/STABILIZE."
        mode=mode.upper(); ALLOWED=["GUIDED","LOITER","AUTO","RTL","LAND","STABILIZE","ALT_HOLD"]
        if mode not in ALLOWED: return f"[BLOCKED] '{mode}' not valid. Use: {', '.join(ALLOWED)}"
        command_queue.put({"action":"set_mode","mode":mode})
        return f"set_flight_mode({mode}) queued."

    def return_to_launch(self) -> str:
        "Fly home in GUIDED then land. Use for: return home, go back, RTL."
        command_queue.put({"action":"rtl"}); _navigation_queued.set()
        return "return_to_launch queued. Drone will fly home and land."

    def land(self) -> str:
        "Land at current position."
        command_queue.put({"action":"land"}); _navigation_queued.set()
        return "land queued."

    def hold_position(self) -> str:
        "Stop immediately and hold position. Use for: stop, hold, hover, freeze, pause."
        _clear_queue(); stop_flag.set(); command_queue.put({"action":"hold"})
        return "hold_position queued — drone will freeze."

    def resume_mission(self) -> str:
        "Resume last interrupted mission from where it left off."
        last=mission_state.get("last_command",{}); pending=mission_state.get("pending_mission",[])
        if not last and not pending: return "No interrupted mission to resume."
        # Drone stayed in GUIDED the whole time — just release the hold lock and requeue.
        _hold_active.clear()
        stop_flag.clear()
        queued=[]
        if last: command_queue.put(last); queued.append(last.get("action","?"))
        for cmd in pending: command_queue.put(cmd); queued.append(cmd.get("action","?"))
        mission_state["pending_mission"]=[]
        flog("info",f"RESUME: re-queued {len(queued)} command(s): {' -> '.join(queued)}")
        return f"Resuming mission: {' -> '.join(queued)}."

    def watch_condition(self, field: str, operator: str, value: float,
                        then_action: str, then_params: str = "") -> str:
        "Register a background condition watch. Fields: rel_alt/groundspeed/armed/mode/airborne/yaw"
        if field not in CONDITION_FIELDS: return f"[BLOCKED] Unknown field '{field}'."
        if operator not in _OPERATORS: return f"[BLOCKED] Unknown operator '{operator}'."
        params={}
        if then_params:
            try: params=json.loads(then_params)
            except: return "[BLOCKED] then_params must be valid JSON."
        label=f"{field} {operator} {value} -> {then_action}"
        condition_monitor.add_watch(ConditionWatch(field,operator,value,then_action,params,label))
        return f"watch_condition registered: {label}"

    def clear_conditions(self) -> str:
        "Remove all active condition watches."
        condition_monitor.clear_watches(); return "All condition watches cleared."

    def list_conditions(self) -> str:
        "List currently active condition watches."
        return condition_monitor.list_watches()

    def get_status(self) -> str:
        "Full live drone status."
        loc=vehicle.location.global_relative_frame; att=vehicle.attitude; batt=vehicle.battery
        return (f"Mode: {vehicle.mode.name} | Armed: {vehicle.armed}\n"
                f"Alt: {round(loc.alt,1)}m | lat: {round(loc.lat,6)} | lon: {round(loc.lon,6)}\n"
                f"Roll: {round(math.degrees(att.roll),1)} Pitch: {round(math.degrees(att.pitch),1)} Yaw: {round(math.degrees(att.yaw),1)}\n"
                f"Groundspeed: {round(vehicle.groundspeed,1)} m/s\nBattery: {batt.level}% | {batt.voltage}V")

    def get_battery(self) -> str:
        "Battery level, voltage, current."
        b=vehicle.battery; return f"Battery: {b.level}% | {b.voltage}V | {b.current}A"

    def get_position(self) -> str:
        "Current GPS and local-frame position."
        loc=vehicle.location.global_relative_frame; ll=vehicle.location.local_frame
        return (f"GPS: {round(loc.lat,6)}, {round(loc.lon,6)} @ {round(loc.alt,1)}m\n"
                f"Local: N={round(ll.north,2)}m E={round(ll.east,2)}m D={round(ll.down,2)}m")

    def get_flight_summary(self) -> str:
        "Full session flight log with timestamps, commands, altitudes."
        log=mission_state["flight_log"]
        if not log: return "No flight activity recorded yet."
        lines=["--- Flight Log ---"]
        for i,e in enumerate(log,1):
            det=", ".join(f"{k}={v}" for k,v in e["details"].items()) if e["details"] else ""
            lines.append(f"{i}. [{e['time']}] {e['action'].upper()} {det} @ {e['alt']}m")
        lines.append(f"Max altitude: {round(mission_state['max_altitude'],1)}m")
        lines.append(f"Total: {len(log)} commands")
        return "\n".join(lines)

# ---------------------------------------------------------------
# SESSION DB + MODEL
# ---------------------------------------------------------------
agent_db       = SqliteDb(db_file="drone_sessions.db")
_NO_THINK_BODY = {"chat_template_kwargs": {"enable_thinking": False}}
ACTIVE_MODEL   = OpenRouter(id="google/gemini-3.1-flash-lite", extra_body=_NO_THINK_BODY)
_all_agents: list = []

def _build_system_message():
    loc_list = "\n".join(
        f"  {name}: {data['description']} (lat={data['lat']}, lon={data['lon']})"
        for name, data in PRESET_LOCATIONS.items())
    return (
        "/nothink\n"
        "You control a real drone in a SITL simulation.\n\n"
        "CRITICAL RULES:\n"
        "1. Call EVERY required tool in order before responding. Never stop after one tool.\n"
        "2. Each tool queues a command. The drone executes in background automatically.\n"
        "3. After ALL tools are called, respond with ONE short sentence only.\n"
        "4. NEVER say you cannot do something. NEVER ask to give commands one at a time.\n"
        "5. Battery readings are FAKE in simulation. Ignore completely. Never mention it.\n"
        "6. Do NOT write long reasoning. Do NOT ask clarifying questions. Just execute.\n"
        "7. NEVER call get_status, get_position, or get_battery to monitor. DO NOT POLL.\n"
        "8. The queue is asynchronous — all commands queue instantly and execute in order.\n\n"
        "DIRECTION REFERENCE FRAMES:\n"
        "- north/south/east/west = absolute compass directions (world frame)\n"
        "- forward/backward/left/right = relative to drone CURRENT HEADING (body frame)\n\n"
        "MULTI-STEP EXAMPLES:\n"
        "- arm and take off to 20m then fly north 100m then return home\n"
        "  -> arm_drone(), takeoff(20), move_direction('north',100), return_to_launch()\n"
        "- fly east 200m then circle then go home\n"
        "  -> move_direction('east',200), fly_circle(30), return_to_launch()\n"
        "- do a flyover of the hospital then return\n"
        "  -> fly_over('hospital',50), return_to_launch()\n"
        "- keep going forward until I say stop -> keep_moving('forward')\n"
        "- fly north until 150 meters from here -> fly_until_distance('north',150)\n\n"
        "NEAREST BUILDING = residence 1 (lat=-35.35734, lon=149.170626)\n\n"
        f"NAMED LOCATIONS:\n{loc_list}\n\n"
        "TOOLS AVAILABLE:\n"
        "arm_drone() | takeoff(altitude) | goto_waypoint(lat,lon,alt) | get_location(name)\n"
        "fly_over(location,alt) | move_direction(dir,dist) | keep_moving(dir)\n"
        "fly_until_distance(dir,dist_m) | adjust_altitude(change) | change_altitude(alt)\n"
        "fly_circle(radius,alt) | set_speed(speed) | set_flight_mode(mode)\n"
        "hold_position() | resume_mission() | return_to_launch() | land() | disarm_drone()\n"
        "get_flight_summary() | watch_condition(field,op,val,action)\n"
        "clear_conditions() | list_conditions()\n"
        "list_missions() | read_mission(filename) | save_report(content,filename) | list_reports()"
    )

# ---------------------------------------------------------------
# LEARNING MACHINE
# ---------------------------------------------------------------
_learning_machine = LearningMachine(
    db=agent_db,
    user_profile=UserProfileConfig(
        mode=LearningMode.ALWAYS,
        additional_instructions=[
            "Extract drone operator preferences: preferred altitude, preferred speed, "
            "favourite locations, typical mission patterns.",
            "Do NOT store battery warnings — battery is fake in SITL.",
        ],
    ),
    user_memory=UserMemoryConfig(
        mode=LearningMode.ALWAYS,
        additional_instructions=[
            "Capture flight behaviour patterns, mission preferences, and corrections "
            "the operator made during the session.",
            "Ignore battery percentage — it is always fake in SITL.",
        ],
    ),
    session_context=SessionContextConfig(
        mode=LearningMode.ALWAYS,
        enable_planning=True,
        additional_instructions=[
            "Track the current drone mission: goal, planned steps, completed steps, "
            "and any interruptions or diversions.",
        ],
    ),
    decision_log=DecisionLogConfig(
        mode=LearningMode.AGENTIC,
        additional_instructions=[
            "Log decisions about tool selection, mission interpretation, "
            "and how you handled ambiguous commands.",
        ],
    ),
)

def _build_flight_agent():
    compression_mgr = CompressionManager(
        model=OpenRouter(id="google/gemini-2.0-flash-001", extra_body=_NO_THINK_BODY),
        compress_tool_results_limit=20,
        compress_tool_call_instructions=(
            "Summarize this drone tool result in one short line. "
            "Keep: action name, GPS coords, altitude, distance, mode, armed status, errors. "
            "Remove all boilerplate."),
    )
    import os as _os
    _skills_path = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "skills")
    _skills = None
    if _os.path.isdir(_skills_path):
        try: _skills = Skills(loaders=[LocalSkills(_skills_path)])
        except Exception as e: print(f"[SKILLS] Could not load: {e}")
    else: print("[SKILLS] No skills/ folder found — running without skills.")

    agent = Agent(
        name="Drone", model=ACTIVE_MODEL,
        tools=[DroneToolkit(), _filesystem_toolkit],
        db=agent_db, add_history_to_context=True, num_history_runs=10,
        tool_call_limit=50, compression_manager=compression_mgr, enable_agentic_state=False,
        learning=_learning_machine,
        add_learnings_to_context=True,
        system_message=_build_system_message(),
        markdown=False,
    )
    if _skills is not None: agent.skills = _skills
    return agent

def _set_active_model(model_id: str):
    global ACTIVE_MODEL, flight_agent
    model_id = model_id.replace("openrouter:","").strip()
    ACTIVE_MODEL = OpenRouter(id=model_id, extra_body=_NO_THINK_BODY)
    flight_agent = _build_flight_agent()
    for a in [safety_agent, summary_agent, _planner, _safety_validator, _summariser]:
        try: a.model = ACTIVE_MODEL
        except: pass
    print(f"[MODEL] Now using: {model_id}")
    flog("info", f"MODEL switched to {model_id}")

# ---------------------------------------------------------------
# AGENTS
# ---------------------------------------------------------------
safety_agent = Agent(
    name="Safety Agent", model=ACTIVE_MODEL, output_schema=SafetyAssessment,
    db=agent_db, add_history_to_context=True, num_history_runs=3,
    instructions=[
        "You are a UAV safety officer. Evaluate missions for compliance.",
        f"Regulations:\n{DRONE_KNOWLEDGE}",
        "Check altitude (max 120m). Do NOT flag battery — simulation only.",
        "Return SafetyAssessment with is_safe, risk_level, issues, recommendations, approved.",
    ], markdown=False)

flight_agent = _build_flight_agent()

summary_agent = Agent(
    name="Session Summary Agent", model=ACTIVE_MODEL, db=agent_db,
    add_history_to_context=True, num_history_runs=10,
    instructions=[
        "You produce flight session summaries from drone flight logs.",
        "Write a clear readable narrative in plain English.",
        "Include all commands in order, altitudes, locations, incidents, and duration.",
        "Write like a pilot debrief — professional, concise, easy to read.",
        "Do NOT output JSON, field names, or structured format. Prose only.",
    ], markdown=False)

_planner = Agent(
    name="Mission Planner", model=ACTIVE_MODEL, output_schema=MissionPlan, db=agent_db,
    instructions=[
        "Convert a mission description into a structured MissionPlan.",
        "Break into ordered DroneCommand steps.",
        "Estimate time: arm=5s, takeoff=15s, goto 100m=30s, circle=60s, flyover=120s, land=10s.",
        "Risk: LOW=simple under 50m, MEDIUM=complex, HIGH=near limits.",
        "Start with arm if drone needs to be armed.",
    ], markdown=False)

_safety_validator = Agent(
    name="Safety Validator", model=ACTIVE_MODEL, output_schema=SafetyAssessment, db=agent_db,
    instructions=[
        "Evaluate a MissionPlan for safety compliance.",
        f"Regulations:\n{DRONE_KNOWLEDGE[:600]}",
        "Return SafetyAssessment with is_safe, risk_level, issues, recommendations, approved.",
    ], markdown=False)

_summariser = Agent(
    name="Post-Flight Summariser", model=ACTIVE_MODEL, db=agent_db,
    instructions=[
        "Generate a plain English flight summary from the session log provided.",
        "Write flowing prose paragraphs like a pilot debrief.",
        "Include: all commands in order, altitudes, locations, incidents, duration.",
        "Do NOT output JSON, structured data, or field names. Plain text only.",
    ], markdown=False)

_all_agents.extend([safety_agent, flight_agent, summary_agent,
                    _planner, _safety_validator, _summariser])

# ---------------------------------------------------------------
# MISSION WORKFLOW
# ---------------------------------------------------------------
def run_mission(mission_description: str):
    SEP = "=" * 55
    print(f"\n{SEP}\n  MISSION: {mission_description}\n{SEP}")
    print("\n[STEP 1/4] Planning...")
    mission_state["phase"] = "planning"
    plan_resp = _planner.run(f"[DRONE STATE]\n{get_live_context()}\n\nPlan: {mission_description}")
    plan = plan_resp.content if isinstance(plan_resp.content, MissionPlan) else None
    if not plan:
        print("[STEP 1] Planning failed. Aborting."); mission_state["phase"]="idle"; return
    mission_state["current_mission"] = plan.model_dump()
    print(f"Plan: {plan.mission_name} | {len(plan.steps)} steps | Risk: {plan.risk_level}")
    print("\n[STEP 2/4] Safety check...")
    mission_state["phase"] = "safety_check"
    safety_resp = _safety_validator.run(
        f"[DRONE STATE]\n{get_live_context()}\n\nPlan:\n{json.dumps(mission_state['current_mission'], indent=2)}")
    assessment = safety_resp.content if isinstance(safety_resp.content, SafetyAssessment) else None
    if assessment:
        status = "APPROVED" if assessment.approved else "REJECTED"
        print(f"Safety: {status} | Risk: {assessment.risk_level}")
        if assessment.issues: print(f"Issues: {assessment.issues}")
        if not assessment.approved:
            mission_state["phase"]="idle"; print("MISSION ABORTED."); return
    else: print("[STEP 2] Safety check skipped — proceeding.")
    print("\n[STEP 3/4] Executing...")
    mission_state["phase"] = "executing"
    if mission_state.get("current_mission") and mission_state["current_mission"].get("steps"):
        steps_txt = "\n".join([
            f"- {s.get('action','?')}: "
            + ", ".join(f"{k}={v}" for k,v in s.items() if k not in ("action","reason") and v is not None)
            for s in mission_state["current_mission"]["steps"]])
        exec_prompt = f"Execute:\n{steps_txt}\nGoal: {plan.objective}"
    else: exec_prompt = f"Execute: {mission_description}"
    print("-" * 40)
    flight_agent.print_response(exec_prompt, session_id=SESSION_ID, stream=True)
    mission_state["phase"] = "complete"; print("-" * 40)
    print("\n[STEP 4/4] Generating report...")
    elapsed = int((datetime.datetime.now()-SESSION_START).total_seconds())
    log_txt = "\n".join([
        f"[{e['time']}] {e['action'].upper()} {e['details']} @ alt={e['alt']}m"
        for e in mission_state["flight_log"]]) or "No commands logged."
    print("-" * 40)
    _summariser.print_response(
        f"Session: {SESSION_ID} | Duration: {elapsed}s\n"
        f"Mission: {mission_description}\nLog:\n{log_txt}\n"
        f"Max alt: {round(mission_state['max_altitude'],1)}m", stream=True)
    print(f"\n{SEP}")

# ---------------------------------------------------------------
# ASYNC RUNNER
# ---------------------------------------------------------------
def _run_agent_async(fn, *args, **kwargs):
    # Do NOT auto-clear a held/stopped state — only the user can resume.
    # If the drone is not held, clear any stale stop and queue before the new agent call.
    if not _hold_active.is_set():
        if not command_queue.empty() or stop_flag.is_set():
            stop_flag.set(); _clear_queue()
    _navigation_queued.clear()
    def _target():
        try: fn(*args, **kwargs)
        except Exception as e:
            print(f"\n[AGENT ERROR] {e}"); flog("error", f"AGENT ERROR: {e}")
        # No auto-resume: mission only continues when the user explicitly says resume.
        print("\n>> ", end="", flush=True)
    threading.Thread(target=_target, daemon=True).start()

# ---------------------------------------------------------------
# STOP DETECTION HELPER
# ---------------------------------------------------------------
_STOP_PHRASES_EXACT = {
    "stop", "hold", "halt", "hold position", "hover", "freeze", "pause",
    "stop now", "hold on", "stop it", "hold it", "cut it", "abort",
}
_STOP_PHRASES_CONTAINS = (
    "stop", "halt", "freeze", "hold on", "hold it", "abort mission",
)

def _is_stop_command(low: str) -> bool:
    """
    Matches stop/hold phrases without eating legitimate commands like
    'stop at the hospital' or 'fly north and stop at 500m'.
    Rule: exact match always wins; fuzzy match only fires for short inputs (<=4 words).
    """
    if low in _STOP_PHRASES_EXACT:
        return True
    if len(low.split()) <= 4:
        return any(p in low for p in _STOP_PHRASES_CONTAINS)
    return False

# ---------------------------------------------------------------
# INPUT ROUTER
# ---------------------------------------------------------------
SUMMARY_KEYWORDS = (
    "summary","flight log","flight summary","what did","what have","what happened",
    "trip summary","session summary","recap","what did the drone do",
    "everything we did","what was done","show me the log","show log","history","what commands",)
MODEL_QUESTIONS = (
    "what model","which model","what ai","what llm","what are you using",
    "what version","are you gemini","are you gpt","are you qwen","are you grok",)

def _handle_input(user_input: str):
    low = user_input.lower().strip()

    # ── Slash commands ──────────────────────────────────────────
    if low.startswith("/model"):
        parts = user_input.split(maxsplit=1)
        if len(parts) < 2:
            print(f"Current model: {ACTIVE_MODEL.id}")
            print("Switch: /model <model_id>")
            print("  /model google/gemini-2.0-flash-001")
            print("  /model qwen/qwen3-235b-a22b")
            print("  /model x-ai/grok-3-mini-beta")
            print("  /model openai/gpt-4o-mini")
        else: _set_active_model(parts[1].strip())
        return

    if any(q in low for q in MODEL_QUESTIONS):
        print(f"Current model: {ACTIVE_MODEL.id} (via OpenRouter)")
        print("Switch with: /model <model_id>"); return

    if low in ("/status","status","what is the status","full status"):
        loc=vehicle.location.global_relative_frame; att=vehicle.attitude; batt=vehicle.battery
        print(f"Mode: {vehicle.mode.name} | Armed: {vehicle.armed}")
        print(f"Alt: {round(loc.alt,1)}m | lat: {round(loc.lat,6)} | lon: {round(loc.lon,6)}")
        print(f"Roll: {round(math.degrees(att.roll),1)} Pitch: {round(math.degrees(att.pitch),1)} Yaw: {round(math.degrees(att.yaw),1)}")
        print(f"Groundspeed: {round(vehicle.groundspeed,1)} m/s")
        print(f"Battery: {batt.level}% | {batt.voltage}V"); return

    if low in ("/battery","battery","battery level","what is the battery"):
        b=vehicle.battery; print(f"Battery: {b.level}% | {b.voltage}V | {b.current}A"); return

    if low in ("/position","position","where is the drone","where are we"):
        loc=vehicle.location.global_relative_frame; ll=vehicle.location.local_frame
        print(f"GPS: {round(loc.lat,6)}, {round(loc.lon,6)} @ {round(loc.alt,1)}m")
        print(f"Local: N={round(ll.north,2)}m E={round(ll.east,2)}m D={round(ll.down,2)}m"); return

    if low == "/mcp":
        ms=_filesystem_toolkit._missions_dir; rp=_filesystem_toolkit._reports_dir
        print("[FILES] Filesystem toolkit: ACTIVE (pure Python, no external dependencies)")
        print(f"  missions/ -> {ms}"); print(f"  reports/  -> {rp}")
        m=[f for f in os.listdir(ms) if f.endswith((".txt",".json"))]
        r=[f for f in os.listdir(rp) if f.endswith((".txt",".json",".md"))]
        print(f"  Mission files: {m or 'none'}"); print(f"  Report files: {r or 'none'}")
        print("  Say: list available missions OR read mission file patrol.txt"); return

    if low == "/memory":
        print("--- Learning Machine State ---")
        try:
            lm = _learning_machine
            if hasattr(lm, "user_profile_store") and lm.user_profile_store:
                print("\n[User Profile]")
                lm.user_profile_store.print(user_id="operator")
            if hasattr(lm, "user_memory_store") and lm.user_memory_store:
                print("\n[User Memory]")
                lm.user_memory_store.print(user_id="operator")
            if hasattr(lm, "session_context_store") and lm.session_context_store:
                print("\n[Session Context]")
                lm.session_context_store.print(session_id=SESSION_ID)
            if hasattr(lm, "decision_log_store") and lm.decision_log_store:
                print("\n[Decision Log (last 5)]")
                lm.decision_log_store.print(agent_id="Drone", limit=5)
        except Exception as e:
            print(f"[MEMORY] Could not read: {e}")
        return

    if low == "/state":
        printable={k: v for k,v in mission_state.items() if k!="current_mission"}
        print(json.dumps(printable, indent=2, default=str))
        print("\nActive condition watches:"); print(condition_monitor.list_watches())
        print(f"\nHold active: {_hold_active.is_set()} | Stop flag: {stop_flag.is_set()}")
        target = _get_active_target()
        print(f"Active nav target: {target}")
        try:
            print(f"WPNAV_SPEED: {vehicle.parameters.get('WPNAV_SPEED')} cm/s")
        except Exception:
            pass
        return

    if low == "/locations":
        print("Preset locations:")
        for name,data in PRESET_LOCATIONS.items():
            print(f"  {name}: {data['description']} ({data['lat']}, {data['lon']})"); return

    if low == "/report":
        elapsed=int((datetime.datetime.now()-SESSION_START).total_seconds())
        log_txt="\n".join([f"[{e['time']}] {e['action']} {e['details']}"
                             for e in mission_state["flight_log"]]) or "No commands."
        _run_agent_async(summary_agent.print_response,
            f"Session: {SESSION_ID}. Duration: {elapsed}s.\nLog:\n{log_txt}\nMax alt: {mission_state['max_altitude']}m",
            session_id=SESSION_ID, stream=True); return

    if low.startswith("/mission"):
        desc=user_input[8:].strip()
        if not desc: print("Usage: /mission <describe your mission>")
        else: _run_agent_async(run_mission, desc)
        return

    # ── Summary keywords ────────────────────────────────────────
    if any(kw in low for kw in SUMMARY_KEYWORDS):
        log=mission_state["flight_log"]
        if not log: print("No flight activity recorded yet."); return
        elapsed=int((datetime.datetime.now()-SESSION_START).total_seconds())
        log_txt="\n".join([
            f"[{e['time']}] {e['action'].upper()} "
            +(", ".join(f"{k}={v}" for k,v in e['details'].items()) if e['details'] else "")
            +f" @ {e['alt']}m" for e in log])
        _run_agent_async(summary_agent.print_response,
            f"Session: {SESSION_ID} | Duration: {elapsed}s\n"
            f"Max alt: {round(mission_state['max_altitude'],1)}m\n"
            f"Incidents: {mission_state['incidents'] or 'None'}\n\nLog:\n{log_txt}",
            session_id=SESSION_ID, stream=True); return

    # ── STOP — checked BEFORE any agent call ────────────────────
    if _is_stop_command(low):
        _hold_active.set()          # Lock: executor cannot clear stop_flag while this is set
        _clear_queue()              # Save pending commands into pending_mission
        stop_flag.set()
        command_queue.put({"action": "hold"})
        pending_count = len(mission_state["pending_mission"])
        print(f"[STOP] Holding in GUIDED — drone hovering at current position. "
              f"{pending_count} pending command(s) saved. Say 'resume' to continue.")
        flog("info", f"STOP '{user_input}': {pending_count} command(s) saved to pending_mission")
        return  # Hard return — never fall through to the agent

    # ── Speed modifiers — applied instantly without touching the queue ────────
    # These never interrupt the mission: the live queue and pending_mission are
    # left completely alone. _set_speed_mavlink writes WPNAV_SPEED, sends
    # DO_CHANGE_SPEED, and re-issues simple_goto() to the active nav target so
    # ArduPilot recomputes the velocity profile for the CURRENT leg immediately.
    _SPEED_MAX=("max speed","maximum speed","fly faster","go faster","move faster",
                "full speed","top speed","fastest","highest speed","increase speed","speed up")
    _SPEED_MIN=("slow down","slower","reduce speed","decrease speed","minimum speed")
    _SPEED_MED=("normal speed","medium speed","default speed")

    if any(p in low for p in _SPEED_MAX):
        applied=_set_speed_mavlink(30)
        print(f"[SPEED] Set to max {applied} m/s — mission unaffected.")
        flog("info","SPEED modifier: 30 m/s (max)"); return
    if any(p in low for p in _SPEED_MIN):
        applied=_set_speed_mavlink(3)
        print(f"[SPEED] Set to {applied} m/s — mission unaffected.")
        flog("info","SPEED modifier: 3 m/s (min)"); return
    if any(p in low for p in _SPEED_MED):
        applied=_set_speed_mavlink(7)
        print(f"[SPEED] Set to {applied} m/s — mission unaffected.")
        flog("info","SPEED modifier: 7 m/s (medium)"); return

    import re as _re
    _speed_match=_re.search(r"(?:speed|at)\s+(\d+(?:\.\d+)?)", low)
    if _speed_match and any(k in low for k in ("speed","m/s","meters per")):
        _spd=min(30.0,max(0.5,float(_speed_match.group(1))))
        applied=_set_speed_mavlink(_spd)
        print(f"[SPEED] Set to {applied} m/s — mission unaffected.")
        flog("info",f"SPEED modifier: {applied} m/s"); return

    _alt_up=_re.search(r"(?:go up|climb|ascend|higher|up)\s+(\d+)", low)
    _alt_down=_re.search(r"(?:go down|descend|lower|down)\s+(\d+)", low)
    if _alt_up and not any(k in low for k in ("north","south","east","west","fly")):
        _chg=float(_alt_up.group(1))
        command_queue.put({"action":"adjust_altitude","change":_chg})
        print(f"[ALTITUDE] Climbing {_chg}m — queued, mission continues.")
        flog("info",f"ALTITUDE modifier: +{_chg}m queued"); return
    if _alt_down and not any(k in low for k in ("north","south","east","west","fly")):
        _chg=float(_alt_down.group(1))
        command_queue.put({"action":"adjust_altitude","change":-_chg})
        print(f"[ALTITUDE] Descending {_chg}m — queued, mission continues.")
        flog("info",f"ALTITUDE modifier: -{_chg}m queued"); return

    # ── RESUME — explicit keyword shortcut ──────────────────────
    _RESUME_PHRASES = ("resume","continue","carry on","keep going","go on",
                       "resume mission","continue mission","pick up where")
    if any(low == p or low.startswith(p) for p in _RESUME_PHRASES):
        last    = mission_state.get("last_command", {})
        pending = mission_state.get("pending_mission", [])
        _MODIFIER_ACTIONS = ("set_speed","set_mode")
        if last.get("action") in _MODIFIER_ACTIONS:
            last = {}
        if not last and not pending:
            print("[RESUME] No interrupted mission to resume.")
            flog("warning","RESUME called but nothing to resume"); return

        _hold_active.clear()    # Release hold lock
        stop_flag.clear()       # Clear stop so executor processes nav commands
        # No mode switch needed — drone stayed in GUIDED throughout the hold

        queued = []
        if last:
            command_queue.put(last); queued.append(last.get("action","?"))
        for cmd in pending:
            command_queue.put(cmd); queued.append(cmd.get("action","?"))
        mission_state["pending_mission"] = []
        summary = " -> ".join(queued)
        print(f"[RESUME] Resuming: {summary}")
        flog("info",f"RESUME: re-queued {len(queued)} command(s): {summary}")
        return

    # ── Fall through to flight agent ────────────────────────────
    enriched = (f"[LIVE DRONE STATE]\n{get_live_context()}\n\n[USER COMMAND]\n{user_input}")
    flog("info", f"USER COMMAND: {user_input}")
    _run_agent_async(flight_agent.print_response, enriched,
                     user_id="operator", session_id=SESSION_ID, stream=True)

# ---------------------------------------------------------------
# CLI
# ---------------------------------------------------------------
print("\n" + "=" * 60)
print("  AGENTIC DRONE CONTROL — READY")
print("=" * 60)
print("  Natural language flight commands:")
print("  >> arm and take off to 20 meters")
print("  >> do a flyover of the hospital")
print("  >> do a surveillance pass over the prison at 60 meters")
print("  >> scout the area around camp a")
print("  >> fly to the airfield at 30 meters")
print("  >> go north 500m, circle with 50m radius, return home")
print("  >> if altitude exceeds 80 meters RTL")
print("  >> keep going north / stop / resume")
print("  >> switch to auto mode / return home / land")
print("  what is the status / where are we")
print("  (or use /status /battery /position for instant readout without AI)")
print("-" * 60)
print("  Slash commands:")
print("  /mission <desc>   4-step plan+safety+execute+report")
print("  /report           flight report")
print("  /state            mission state + condition watches + hold/stop flags")
print("  /locations        all preset named locations")
print("  /memory           show learning machine memory stores")
print("  /mcp              show filesystem toolkit status")
print("  /model            show current model")
print("  /model <id>       switch model e.g. /model qwen/qwen3-235b-a22b")
print("  exit")
print("=" * 60 + "\n")

while True:
    try: user_input = input(">> ").strip()
    except (KeyboardInterrupt, EOFError):
        print("\nShutting down.")
        flog("info", f"SESSION END — ID: {SESSION_ID} | Keyboard interrupt"); break
    if not user_input: continue
    if user_input.lower() in ("exit","quit"):
        print("Exiting.")
        flog("info", f"SESSION END — ID: {SESSION_ID} | User exit"); break
    _handle_input(user_input)

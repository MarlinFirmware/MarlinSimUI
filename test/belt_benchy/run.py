#!/usr/bin/env python3
"""
Belt printer Benchy: stream a real belt-sliced print through the simulator and check the machine did what the G-code says.

The G-code is 3DBenchy sliced by ideaMaker for a 45 degree belt printer (3DBenchy.gcode.gz, next to this script):
Marlin Y carries the height along the tilted gantry, Z is the belt feed, X is across the belt. 131,000 lines, 293 layers.

What is checked (ground truth is the simulated steppers, not Marlin's own position report):
  - every line is accepted (no `Unknown command`, no `Error:`, no halt / kill)
  - at each checkpoint (every CHECK_LAYERS layers, and the end of the print) the nozzle is where the G-code last sent it
    (X, Y within POS_TOL mm; Z measured from the belt position when `G92 Z0` ran, so the belt feed is proven too),
    and the E steppers have delivered the filament the G-code asked for (after M221 flow), within E_TOL
  - the end G-code runs (cool down, home, park at Y50)
  - (informational) the simulated time next to the slicer's estimate
Pictures of the finished print (several camera angles) are written to the output folder for a person to look at.

Usage:
  test/belt_benchy/run.py --sim <MarlinSimulator binary>            # starts its own sim on its own ports
  test/belt_benchy/run.py --attach --agent-port 8100 --serial-port 8099   # use a sim that is already running
See README.md for the Marlin configuration the simulator must be built with.
Needs only the Python 3 standard library.
"""
import argparse, gzip, json, os, re, socket, subprocess, sys, tempfile, time, urllib.request, urllib.error

HERE = os.path.dirname(os.path.abspath(__file__))
GCODE_GZ = os.path.join(HERE, "3DBenchy.gcode.gz")
CHECK_LAYERS = 30      # compare with the G-code every this many layers
POS_TOL = 0.05         # mm, X and Y and belt (Z)
E_TOL = 0.03           # fraction of the filament delivered so far (plus a small absolute allowance). Per-move rounding of flow-scaled
                       # E steps costs about +2.6% on this file; a planner that truncates instead loses about 13% and fails.


class Agent:
  def __init__(self, port):
    self.base = f"http://127.0.0.1:{port}"
  def req(self, method, path, body=None, timeout=30):
    data = json.dumps(body).encode() if body is not None else None
    r = urllib.request.Request(self.base + path, data=data, method=method, headers={"Content-Type": "application/json"})
    try:
      with urllib.request.urlopen(r, timeout=timeout) as f:
        return json.loads(f.read() or b"{}")
    except urllib.error.HTTPError as e:
      raise RuntimeError(f"{method} {path} -> {e.code}: {e.read().decode(errors='replace')}")
  def wait_up(self, secs=60):
    end = time.monotonic() + secs
    while time.monotonic() < end:
      try: return self.req("GET", "/ping", timeout=2)
      except Exception: time.sleep(0.5)
    raise TimeoutError("agent port not answering (start the sim with --agent-port)")
  def wait_idle(self, secs=1800):
    end = time.monotonic() + secs
    while time.monotonic() < end:
      if self.req("GET", "/idle").get("idle"): return
      time.sleep(0.1)
    raise TimeoutError("sim did not go idle")
  def kin(self):
    st = self.req("GET", "/state/CoreXY%20Kinematic%20System")["state"]
    eff = st["effectors"][0]["position"]
    return eff["x"], eff["y"], eff["z"], eff["e"]


class Serial:
  """Raw G-code socket with a small window of lines in flight (each `ok` frees a slot)."""
  def __init__(self, port, window=6):
    self.sock = socket.create_connection(("127.0.0.1", port), timeout=600)
    self.buf = b""
    self.window = window
    self.pending = []          # lines sent and not yet acknowledged
    self.problems = []
  def _read_one(self):
    """Process incoming lines until one `ok` has been seen."""
    while True:
      while b"\n" in self.buf:
        l, self.buf = self.buf.split(b"\n", 1)
        t = l.decode(errors="replace").strip()
        if t.startswith("ok"):
          if self.pending: self.pending.pop(0)
          return
        if "Unknown command" in t or t.startswith("Error:") or "Printer halted" in t or "kill()" in t:
          self.problems.append((self.pending[0] if self.pending else "?", t))
      chunk = self.sock.recv(65536)
      if not chunk: raise ConnectionError("serial socket closed (did the sim exit?)")
      self.buf += chunk
  def send(self, line):
    while len(self.pending) >= self.window: self._read_one()
    self.pending.append(line)
    self.sock.sendall((line + "\n").encode())
  def drain(self):
    while self.pending: self._read_one()


class Tracker:
  """What the G-code says the machine should have done, line by line (absolute XYZ, absolute or relative E, M221 flow)."""
  def __init__(self):
    self.x = self.y = self.z = None
    self.e_logical = 0.0          # G-code's E value
    self.e_delivered = 0.0        # net filament pushed, in mm, after flow (what the E steppers must have done)
    self.abs_xyz = True
    self.abs_e = True
    self.flow = 1.0
  def feed(self, line):
    code = line.split(";", 1)[0].strip()
    if not code: return
    w = code.split()
    cmd = w[0].upper()
    args = {}
    for p in w[1:]:
      if p[0].isalpha():
        try: args[p[0].upper()] = float(p[1:]) if len(p) > 1 else 0.0
        except ValueError: pass
    if cmd in ("G0", "G1"):
      for ax in "XYZ":
        if ax in args:
          cur = getattr(self, ax.lower())
          setattr(self, ax.lower(), args[ax] if self.abs_xyz else (cur or 0.0) + args[ax])
      if "E" in args:
        new = args["E"] if self.abs_e else self.e_logical + args["E"]
        self.e_delivered += (new - self.e_logical) * self.flow
        self.e_logical = new
    elif cmd == "G90": self.abs_xyz = True; self.abs_e = True
    elif cmd == "G91": self.abs_xyz = False; self.abs_e = False
    elif cmd == "M82": self.abs_e = True
    elif cmd == "M83": self.abs_e = False
    elif cmd == "G92":
      if "E" in args: self.e_logical = args["E"]
      if "Z" in args: self.z = args["Z"]
      if "X" in args: self.x = args["X"]
      if "Y" in args: self.y = args["Y"]
    elif cmd == "G28":
      self.x = self.y = self.z = None    # homed: unknown until the next absolute move
    elif cmd == "M221" and "S" in args:
      self.flow = args["S"] / 100.0


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--sim", help="MarlinSimulator binary built with the belt printer configuration (README.md)")
  ap.add_argument("--attach", action="store_true", help="use a sim that is already running")
  ap.add_argument("--agent-port", type=int, default=8130)
  ap.add_argument("--serial-port", type=int, default=8230)
  ap.add_argument("--scale", type=float, default=100, help="realtime_scale for the run (default 100)")
  ap.add_argument("--out", default=None, help="output folder for pictures and the report (default: a temp folder)")
  ap.add_argument("--max-lines", type=int, default=0, help="stop after this many lines (a quick run; the end checks are skipped)")
  ap.add_argument("--keep", action="store_true", help="leave the sim running afterwards")
  a = ap.parse_args()
  if not a.attach and not a.sim: ap.error("give --sim <binary>, or --attach")

  out = a.out or tempfile.mkdtemp(prefix="belt_benchy_")
  os.makedirs(out, exist_ok=True)
  with gzip.open(GCODE_GZ, "rt") as f: lines = f.read().splitlines()
  slicer_seconds = next((float(m.group(1)) for m in (re.match(r";REMAINING_TIME:\s*(\d+)", l) for l in lines) if m), None)

  proc = None
  if not a.attach:
    rundir = tempfile.mkdtemp(prefix="belt_benchy_run_")
    proc = subprocess.Popen([os.path.abspath(a.sim), "--agent-port", str(a.agent_port), "--serial-port", str(a.serial_port),
                             "--no-audio", "--machine", "belt"], cwd=rundir, stdout=open(os.path.join(out, "sim.log"), "w"), stderr=subprocess.STDOUT)
  failures = []
  def check(ok, what):
    print(("PASS  " if ok else "FAIL  ") + what, flush=True)
    if not ok: failures.append(what)

  try:
    agent = Agent(a.agent_port); agent.wait_up()
    agent.req("POST", "/kernel/control", {"realtime_scale": a.scale})
    ser = Serial(a.serial_port)
    trk = Tracker()
    z_origin = None; e_origin = None        # physical position at the first checkpoint after `G92 E0 Z0`
    checkpoints = []
    layers_since = 0; t0 = time.time(); last_report = t0
    end_start = next(i for i, l in enumerate(lines) if l.startswith("M104 T0 S0"))   # the end G-code
    n = len(lines) if not a.max_lines else min(a.max_lines, len(lines))

    def checkpoint(tag):
      nonlocal z_origin, e_origin
      ser.send("M400"); ser.drain(); agent.wait_idle()
      x, y, z, e = agent.kin()
      if z_origin is None:
        z_origin, e_origin = z - (trk.z or 0.0), e - trk.e_delivered     # `G92 Z0` just ran: physical belt position of Z0
        print(f"      first checkpoint: belt origin {z_origin:.3f}, E origin {e_origin:.3f}")
        return
      dx, dy, dz = x - trk.x, y - trk.y, (z - z_origin) - trk.z
      e_got, e_want = e - e_origin, trk.e_delivered
      ok_pos = max(abs(dx), abs(dy), abs(dz)) <= POS_TOL
      ok_e = abs(e_got - e_want) <= E_TOL * max(abs(e_want), 1.0) + 0.05
      checkpoints.append((tag, dx, dy, dz, e_got - e_want))
      check(ok_pos and ok_e, f"{tag}: nozzle off by ({dx:+.3f}, {dy:+.3f}, {dz:+.3f}) mm; E delivered {e_got:.2f} of {e_want:.2f} mm")

    for i in range(n):
      line = lines[i]
      m = re.match(r";LAYER:(\d+)", line)
      if m and int(m.group(1)) > 0 and int(m.group(1)) % CHECK_LAYERS == 0 and trk.x is not None and z_origin is not None:
        checkpoint(f"layer {m.group(1)}")
      if i == end_start and z_origin is not None and not a.max_lines:
        checkpoint("end of print")
      code = line.split(";", 1)[0].strip()
      if code:
        ser.send(code)
        trk.feed(code)
        if z_origin is None and code.startswith("G92") and "Z" in code and trk.z is not None:
          checkpoint("start")
      if time.time() - last_report > 20:
        last_report = time.time()
        print(f"      {i}/{n} lines, sim time {agent.req('GET','/kernel')['sim_seconds']:.0f} s, wall {time.time()-t0:.0f} s", flush=True)
    ser.send("M400"); ser.drain(); agent.wait_idle()
    sim_seconds = agent.req("GET", "/kernel")["sim_seconds"]

    check(not ser.problems, "every line accepted" + ("" if not ser.problems else f": {ser.problems[:3]}"))
    check(len(checkpoints) >= (1 if a.max_lines else 8), f"{len(checkpoints)} checkpoints compared with the G-code")
    if not a.max_lines:
      x, y, z, e = agent.kin()
      check(abs(y - 50.0) <= POS_TOL, f"parked at Y50 after the end G-code (Y {y:.3f})")
      if slicer_seconds:
        ratio = sim_seconds / slicer_seconds
        print(f"INFO  sim time {sim_seconds:.0f} s (includes heating and cooling) vs slicer estimate {slicer_seconds:.0f} s (x{ratio:.2f})")
      for name, view in (("front", dict(preset="front")), ("side", dict(yaw=90, pitch=10)), ("iso", dict(yaw=35, pitch=30)), ("top", dict(pitch=89, yaw=0))):
        # The print sits on the belt near where the last layer was made; aim at the last known nozzle height range, not a fixed point
        agent.req("POST", "/view", {**view, "distance": 160})
        time.sleep(1.5)
        agent.req("POST", "/capture/viewport", {"path": os.path.join(out, f"benchy_{name}.png")})
    print(f"\nwall time {time.time()-t0:.0f} s; output in {out}")
  except Exception as ex:
    failures.append(f"exception: {ex!r}")
    print("FAIL  exception:", repr(ex))
  finally:
    if proc and not a.keep:
      proc.terminate()
      try: proc.wait(10)
      except Exception: proc.kill()
  print("\nRESULT:", "PASS" if not failures else f"FAIL ({len(failures)})")
  return 0 if not failures else 1


if __name__ == "__main__":
  sys.exit(main())

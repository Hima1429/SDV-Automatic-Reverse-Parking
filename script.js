'use strict';
/* =====================================================================
   Automatic Reverse Parking System — SDV simulation  (units: cm, rad, s)
   SENSE -> ANALYZE -> DECIDE -> CONTROL -> PARK
   Modules: CFG · Geometry · World · Vehicle · Sensors · Slot · Safety ·
            Ctl (state machine + steering) · Manual · Result · App · View/UI
   ===================================================================== */
const KMH = 27.78; // cm/s per km/h
const CFG = {
  L: 450, W: 180, WB: 270, RO: 90,               // length, width, wheelbase, rear overhang
  MAX_STEER: 45 * Math.PI / 180, STEER_RATE: 0.9, // rad, rad/s (real steering takes time)
  SAFETY: 50, INSET: 10, REAR_GAP: 16,            // margin per end, boundary inset, target rear gap
  EMERGENCY: 10, HARD: 4, WARN: 25, CAUTION: 60, SENSOR_MAX: 250,
  WORLD_W: 2300, WORLD_H: 800, LANE_Y: 450, BAY_Y: 230, WALL_Y: 85, CURB_Y: 740, A_X: 450,
  SLOTS: { small: 520, tight: 600, medium: 650, large: 760 },
  V_SEARCH: 5 * KMH, V_PARK: 4 * KMH, V_POS: 3 * KMH, ACC: 90, DEC: 160
};
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const deg = r => r * 180 / Math.PI;

/* ---------- Geometry ---------- */
// Ray vs axis-aligned rectangle (slab method) -> distance or Infinity
function rayRect(ox, oy, dx, dy, r) {
  let t0 = 0, t1 = 1e9;
  for (const [o, d, mn, mx] of [[ox, dx, r.x, r.x + r.w], [oy, dy, r.y, r.y + r.h]]) {
    if (Math.abs(d) < 1e-9) { if (o < mn || o > mx) return Infinity; }
    else { let a = (mn - o) / d, b = (mx - o) / d; if (a > b) [a, b] = [b, a]; t0 = Math.max(t0, a); t1 = Math.min(t1, b); if (t0 > t1) return Infinity; }
  }
  return t0;
}
// Oriented car hull (inflated by m) vs rectangle — separating axis test
function hits(c, r, m) {
  const co = Math.cos(c.th), si = Math.sin(c.th), off = CFG.L / 2 - CFG.RO;
  const cx = c.x + co * off, cy = c.y + si * off, hl = CFG.L / 2 + m, hw = CFG.W / 2 + m;
  const dx = r.x + r.w / 2 - cx, dy = r.y + r.h / 2 - cy;
  if (Math.abs(dx) > Math.abs(co) * hl + Math.abs(si) * hw + r.w / 2) return false;
  if (Math.abs(dy) > Math.abs(si) * hl + Math.abs(co) * hw + r.h / 2) return false;
  const pf = Math.abs(dx * co + dy * si), pl = Math.abs(dx * si - dy * co);
  return pf <= hl + r.w / 2 * Math.abs(co) + r.h / 2 * Math.abs(si) &&
         pl <= hw + r.w / 2 * Math.abs(si) + r.h / 2 * Math.abs(co);
}

/* ---------- World ---------- */
const World = {
  cars: [], walls: [], dynamic: [], slotLen: 600, slotStart: 0,
  build(len) {
    const W = CFG.WORLD_W, H = CFG.WORLD_H, aF = CFG.A_X + CFG.L, y = CFG.BAY_Y - CFG.W / 2;
    this.slotLen = len; this.slotStart = aF; this.dynamic = [];
    this.cars = [{ x: CFG.A_X, y, w: CFG.L, h: CFG.W, color: '#5b7cfa' }, { x: aF + len, y, w: CFG.L, h: CFG.W, color: '#f0883e' }];
    this.walls = [{ x: -300, y: -300, w: W + 600, h: 300 + CFG.WALL_Y }, { x: -300, y: CFG.CURB_Y, w: W + 600, h: 300 },
                  { x: -300, y: -300, w: 300, h: H + 600 }, { x: W, y: -300, w: 300, h: H + 600 }];
  },
  all() { return [...this.cars, ...this.walls, ...this.dynamic]; }
};

/* ---------- Vehicle (kinematic bicycle model, reference = rear axle) ---------- */
const Vehicle = {
  create() { return { x: 140, y: CFG.LANE_Y, th: 0, v: 0, delta: 0, gear: 'P', shift: 0, odo: 0 }; },
  dir(c) { return { fx: Math.cos(c.th), fy: Math.sin(c.th), lx: Math.sin(c.th), ly: -Math.cos(c.th) }; },
  pt(c, a, b) { const d = this.dir(c); return { x: c.x + a * d.fx + b * d.lx, y: c.y + a * d.fy + b * d.ly }; },
  apply(c, cmd, dt) {
    let speed = cmd.speed;
    if (cmd.gear !== c.gear) {                      // gear shifts only when stopped
      speed = 0;
      if (Math.abs(c.v) < 2) { c.shift += dt; if (c.shift > 0.5) { c.gear = cmd.gear; c.shift = 0; } }
    } else c.shift = 0;
    const tgt = clamp(cmd.steer, -CFG.MAX_STEER, CFG.MAX_STEER);   // rate-limited steering actuator
    c.delta += clamp(tgt - c.delta, -CFG.STEER_RATE * dt, CFG.STEER_RATE * dt);
    if (!cmd.free && Math.abs(tgt - c.delta) > 0.12) speed = 0;     // steer first, then roll
    const vT = c.gear === 'D' ? speed : c.gear === 'R' ? -speed : 0;
    const a = cmd.brake ? 1500 : (Math.abs(vT) < Math.abs(c.v) || vT * c.v < 0 ? CFG.DEC : CFG.ACC);
    c.v += clamp(vT - c.v, -a * dt, a * dt);
    c.th += c.v / CFG.WB * Math.tan(c.delta) * dt;
    c.x += c.v * Math.cos(c.th) * dt; c.y += c.v * Math.sin(c.th) * dt; c.odo += Math.abs(c.v) * dt;
  }
};

/* ---------- Four virtual distance sensors (ray-cast fans) ---------- */
const SENSORS = [
  { id: 'FL', name: 'Front Left',  a: CFG.L - CFG.RO, b:  CFG.W / 2, base: -0.35, rev: false },
  { id: 'FR', name: 'Front Right', a: CFG.L - CFG.RO, b: -CFG.W / 2, base:  0.35, rev: false },
  { id: 'RL', name: 'Rear Left',   a: -CFG.RO,        b:  CFG.W / 2, base:  0.35, rev: true },
  { id: 'RR', name: 'Rear Right',  a: -CFG.RO,        b: -CFG.W / 2, base: -0.35, rev: true }];
const FAN = [-0.5, -0.25, 0, 0.25, 0.5];
const Sensors = {
  cast(ox, oy, ang, max) {
    let m = max;
    for (const r of World.all()) { const t = rayRect(ox, oy, Math.cos(ang), Math.sin(ang), r); if (t < m) m = t; }
    return m;
  },
  read(c) {
    return SENSORS.map(s => {
      const p = Vehicle.pt(c, s.a, s.b), a0 = c.th + (s.rev ? Math.PI : 0) + s.base;
      const rays = FAN.map(f => ({ a: a0 + f, d: this.cast(p.x, p.y, a0 + f, CFG.SENSOR_MAX) }));
      const best = rays.reduce((m, r) => r.d < m.d ? r : m, rays[0]);
      return { ...s, x: p.x, y: p.y, d: best.d, ang: best.a, rays };
    });
  },
  level(d) { return d < CFG.WARN ? 2 : d < CFG.CAUTION ? 1 : 0; }
};

/* ---------- Parking-slot detection (side scan + odometry) & safe boundary ---------- */
const Slot = {
  scan: null,
  reset() { this.scan = { seenCar: false, inGap: false, start: 0, found: null, ray: null }; },
  update(c) {
    const p = Vehicle.pt(c, CFG.L / 2 - CFG.RO, CFG.W / 2), s = this.scan;
    const r = Sensors.cast(p.x, p.y, c.th - Math.PI / 2, 400); s.ray = { p, r };
    if (r < 120) { if (s.inGap) { s.inGap = false; s.found = { start: s.start, end: p.x, len: p.x - s.start }; } else s.seenCar = true; }
    else if (s.seenCar && !s.inGap) { s.inGap = true; s.start = p.x; }
  },
  evaluate(f) {
    const req = CFG.L + 2 * CFG.SAFETY;
    return { ...f, req, ok: f.len >= req,
      b: { x0: f.start + CFG.INSET, x1: f.end - CFG.INSET, y0: CFG.BAY_Y - 100, y1: CFG.BAY_Y + 100 } }; // safe boundary
  }
};

/* ---------- Collision / emergency logic ---------- */
const Safety = {
  check(c, sens, parked) {
    const fwd = c.v > 1, rev = c.v < -1;
    const near = sens.find(s => ((s.rev && rev) || (!s.rev && fwd)) && s.d < CFG.EMERGENCY);
    let hard = false, warn = false;
    for (const r of World.all()) { if (hits(c, r, CFG.HARD)) hard = true; else if (!parked && hits(c, r, CFG.WARN)) warn = true; }
    if (near) return { lvl: 2, msg: near.name + ' sensor = ' + Math.round(near.d) + ' cm' };
    if (hard) { const w = sens.filter(s => (s.rev && rev) || (!s.rev && fwd)).sort((a, b) => a.d - b.d)[0];
      return { lvl: 2, msg: w && w.d < 40 ? w.name + ' sensor = ' + Math.round(w.d) + ' cm' : 'Collision envelope violated' }; }
    return { lvl: warn ? 1 : 0, msg: warn ? 'Close to obstacle' : 'SAFE' };
  }
};

/* ---------- Controller: state machine + steering ---------- */
const STATE_TEXT = {
  SEARCHING: 'Searching for a parking slot…', SPACE_DETECTED: 'Parking slot detected — analysing',
  POSITIONING: 'Slot suitable — safe boundary calculated, positioning vehicle', REVERSE: 'Reverse mode activated — steering into slot',
  TURNING: 'Counter-steering — straightening while reversing', ALIGNING: 'Aligning inside safe boundary',
  PARKED: 'Parking completed', EMERGENCY_STOP: 'EMERGENCY STOP', SLOT_TOO_SMALL: 'Slot too small — manoeuvre aborted'
};
const Ctl = {
  state: 'IDLE', t: 0, ev: null, pl: null, msg: '', R: CFG.WB / Math.tan(CFG.MAX_STEER),
  reset() { this.state = 'IDLE'; this.t = 0; this.ev = null; this.pl = null; this.msg = ''; },
  to(s) { this.state = s; this.t = 0; App.addLog(STATE_TEXT[s] || s); },
  start() { Slot.reset(); this.ev = null; this.pl = null; this.msg = ''; this.to('SEARCHING'); },
  emergency(msg) { this.msg = msg; this.to('EMERGENCY_STOP'); },
  arc(th) { return this.R * (1 - Math.cos(th)); },            // lateral shift left in a full-lock arc
  // heading + lateral hold: s = +1 forward, -1 reverse
  hold(c, s, ey) { return clamp(-0.0044 * (c.y - ey) - s * 2.2 * c.th, -0.4, 0.4); },
  // DECIDE: arc geometry from measured lateral offset -> where the reverse must start
  plan(c) {
    const gy = CFG.BAY_Y, d = Math.max(0, c.y - gy), th = Math.acos(clamp(1 - d / (2 * this.R), -1, 1));
    const goalX = this.ev.start + CFG.REAR_GAP + CFG.RO;
    return { gy, goalX, th, startX: goalX + 2 * this.R * Math.sin(th) };
  },
  update(c, dt, sens) {
    this.t += dt;
    const sd = id => sens.find(s => s.id === id).d, M = CFG.MAX_STEER;
    const cmd = (gear, speed, steer, brake) => ({ gear, speed, steer, brake });
    switch (this.state) {
      case 'SEARCHING':
        Slot.update(c);
        if (Slot.scan.found) {
          this.ev = Slot.evaluate(Slot.scan.found);
          App.addLog('Measured slot ' + Math.round(this.ev.len) + ' cm, required ' + this.ev.req + ' cm');
          this.to('SPACE_DETECTED');
        }
        return cmd('D', CFG.V_SEARCH, this.hold(c, 1, CFG.LANE_Y));
      case 'SPACE_DETECTED':
        if (this.t > 1.5 && Math.abs(c.v) < 2) { if (!this.ev.ok) this.to('SLOT_TOO_SMALL'); else { this.pl = this.plan(c); this.to('POSITIONING'); } }
        return cmd('D', 0, this.hold(c, 1, CFG.LANE_Y));
      case 'POSITIONING': {
        const dx = this.pl.startX - c.x, s = dx > 0 ? 1 : -1;
        if (Math.abs(dx) < 2.5) { if (Math.abs(c.v) < 2) this.to('REVERSE'); return cmd(c.gear, 0, 0); }
        return cmd(s > 0 ? 'D' : 'R', clamp(Math.sqrt(2 * 90 * Math.abs(dx)), 10, CFG.V_POS), this.hold(c, s, CFG.LANE_Y));
      }
      case 'REVERSE': {      // arc 1: full lock toward the bay until the remaining shift equals one full-lock arc
        const e = c.y - this.pl.gy, ths = c.th + c.v * c.v / (2 * CFG.DEC) / this.R;   // heading once the car has stopped
        const eStop = e - this.R * (Math.cos(c.th) - Math.cos(ths));                    // lateral error left after braking
        if (eStop <= this.arc(ths) || Math.min(sd('RL'), sd('RR')) < 30) this.to('TURNING');
        return cmd('R', CFG.V_PARK, -M);
      }
      case 'TURNING':        // arc 2: opposite lock, straightens the car as it settles into the bay
        if (c.th <= c.v * c.v / (2 * CFG.DEC) / this.R + 0.005) this.to('ALIGNING');   // stop so the car coasts to heading 0
        return cmd('R', CFG.V_PARK * clamp(c.th / 0.35, 0.4, 1), M);
      case 'ALIGNING': {     // centre fore/aft inside the safe boundary, trim heading
        const b = this.ev.b, dx = (b.x0 + b.x1) / 2 - (CFG.L / 2 - CFG.RO) - c.x, s = dx > 0 ? 1 : -1;
        if (Math.abs(dx) < 3) { if (Math.abs(c.v) < 2) this.to('PARKED'); return cmd(c.gear, 0, this.hold(c, s, this.pl.gy)); }
        return cmd(s > 0 ? 'D' : 'R', clamp(Math.sqrt(2 * 60 * Math.abs(dx)), 10, 3 * KMH), this.hold(c, s, this.pl.gy));
      }
      case 'PARKED': return cmd('P', 0, 0);
      case 'EMERGENCY_STOP': return cmd(c.gear, 0, c.delta, true);
      default: return cmd(c.gear === 'P' ? 'P' : c.gear, 0, 0);
    }
  }
};

/* ---------- Manual drive ---------- */
const Manual = {
  keys: {},
  cmd(c) {
    const k = this.keys, up = k.ArrowUp || k.w, dn = k.ArrowDown || k.s;
    let gear = c.gear, speed = 0;
    if (up) { gear = 'D'; speed = 12 * KMH; } else if (dn) { gear = 'R'; speed = 6 * KMH; }
    if (k[' ']) speed = 0;
    const st = ((k.ArrowRight || k.d) ? 1 : 0) - ((k.ArrowLeft || k.a) ? 1 : 0);
    return { gear, speed, steer: st * 0.7, brake: !!k[' '], free: true };
  }
};

/* ---------- Parking result ---------- */
function computeResult(c, ev) {
  const rear = (c.x - CFG.RO) - ev.start, front = ev.end - (c.x + CFG.L - CFG.RO), curb = c.y - CFG.W / 2 - CFG.WALL_Y;
  const th = Math.abs(deg(c.th)), lat = Math.abs(c.y - CFG.BAY_Y);
  return { rear, front, curb, th, lat,
    acc: clamp(100 - lat * 0.7 - Math.abs(rear - front) * 0.1 - th * 2.5, 0, 100),
    align: clamp(100 - th * 6 - lat * 0.3, 0, 100) };
}

/* ---------- App: simulation loop owner ---------- */
const App = {
  slotKey: 'medium', mode: 'auto', paused: false, car: null, trail: [], sens: [], safety: { lvl: 0, msg: 'SAFE' },
  result: null, demo: false, dropped: false, revDist: 0, log: [], notice: '', noticeT: 0, onLog: null,
  reset(key) {
    if (key) this.slotKey = key;
    World.build(CFG.SLOTS[this.slotKey]); this.car = Vehicle.create(); Slot.reset(); Ctl.reset();
    Object.assign(this, { mode: 'auto', paused: false, trail: [], result: null, demo: false, dropped: false, revDist: 0, log: [], notice: '' });
    this.sens = Sensors.read(this.car); this.safety = { lvl: 0, msg: 'SAFE' };
    this.addLog('Scene ready — slot ' + CFG.SLOTS[this.slotKey] + ' cm, vehicle ' + CFG.L + '×' + CFG.W + ' cm');
  },
  addLog(m) { this.log.unshift(m); this.log.length = Math.min(this.log.length, 8); if (this.onLog) this.onLog(); },
  startAuto(demo) {
    const k = this.slotKey; this.reset(k); this.demo = !!demo; Ctl.start();
    if (demo) this.addLog('Emergency demo: obstacle will appear during reverse');
  },
  setManual(on) {
    if (on) { Ctl.reset(); this.mode = 'manual'; this.addLog('Manual drive: ↑/W drive · ↓/S reverse · ←/→ or A/D steer · Space brake'); }
    else { this.mode = 'auto'; Manual.keys = {}; this.addLog('Manual drive off'); }
  },
  estop() {
    if (this.mode === 'manual') { this.car.v = 0; this.notice = 'EMERGENCY STOP'; this.noticeT = 2; }
    else if (!['IDLE', 'PARKED', 'EMERGENCY_STOP'].includes(Ctl.state)) Ctl.emergency('Driver pressed EMERGENCY STOP');
  },
  dropObstacle() {   // virtual obstacle behind the rear-left corner
    const c = this.car, d = Vehicle.dir(c), p = Vehicle.pt(c, -CFG.RO, CFG.W / 2), g = 105;
    const cx = p.x - d.fx * g + d.lx * 25, cy = p.y - d.fy * g + d.ly * 25;      // behind the rear-left sensor
    World.dynamic = [{ x: cx - 40, y: cy - 40, w: 80, h: 80, obstacle: true }];
    this.addLog('Virtual obstacle placed behind the vehicle');
  },
  step(dt) {
    if (this.paused) return;
    const c = this.car, manual = this.mode === 'manual';
    if (this.noticeT > 0) { this.noticeT -= dt; if (this.noticeT <= 0) this.notice = ''; }
    const cmd = manual ? Manual.cmd(c) : Ctl.update(c, dt, this.sens);
    const prev = { x: c.x, y: c.y, th: c.th };
    const was = Ctl.state;
    Vehicle.apply(c, cmd, dt);
    if (manual) {
      if (c.v > 5) { if (!Slot.scan) Slot.reset(); Slot.update(c); if (Slot.scan.found) { Ctl.ev = Slot.evaluate(Slot.scan.found); Slot.scan.found = null; } }
      if (World.all().some(r => hits(c, r, 2))) { Object.assign(c, prev); c.v = 0; this.notice = 'EMERGENCY BRAKE — obstacle'; this.noticeT = 1.5; }
    }
    this.sens = Sensors.read(c);
    this.safety = Safety.check(c, this.sens, Ctl.state === 'PARKED');
    if (!manual && this.safety.lvl === 2 && !['EMERGENCY_STOP', 'PARKED', 'IDLE', 'SLOT_TOO_SMALL'].includes(Ctl.state)) Ctl.emergency(this.safety.msg);
    if (Ctl.state === 'PARKED' && !this.result) { this.result = computeResult(c, Ctl.ev); this.addLog('Accuracy ' + Math.round(this.result.acc) + '%'); }
    if (this.demo && !this.dropped) {
      if (c.v < 0) this.revDist += -c.v * dt;
      if (this.revDist > 120) { this.dropped = true; this.dropObstacle(); }
    }
    const l = this.trail[this.trail.length - 1];
    if (!l || Math.hypot(l.x - c.x, l.y - c.y) > 6) this.trail.push({ x: c.x, y: c.y });
  }
};

if (typeof module !== 'undefined') module.exports = { Manual, App, Ctl, CFG, World, Vehicle, Sensors, Slot, hits };

/* =====================================================================
   View + UI (browser only)
   ===================================================================== */
if (typeof document !== 'undefined') {
  const $ = id => document.getElementById(id), cv = $('sim'), ctx = cv.getContext('2d');
  const COL = ['#2ee6a6', '#ffd24a', '#ff4d6d'], SC = cv.width / CFG.WORLD_W;

  function rr(x, y, w, h, r) { ctx.beginPath(); ctx.roundRect(x, y, w, h, r); }
  function drawCar(x, y, th, delta, color, o = {}) {
    ctx.save(); ctx.translate(x, y); ctx.rotate(th);
    if (o.ghost) { ctx.setLineDash([14, 10]); ctx.strokeStyle = 'rgba(46,230,166,.8)'; ctx.lineWidth = 3; rr(-CFG.RO, -90, CFG.L, 180, 30); ctx.stroke(); ctx.restore(); return; }
    ctx.fillStyle = '#05080c';
    for (const [wx, st] of [[0, 0], [CFG.WB, delta]]) for (const wy of [-80, 80]) { ctx.save(); ctx.translate(wx, wy); ctx.rotate(st); rr(-35, -13, 70, 26, 6); ctx.fill(); ctx.restore(); }
    const g = ctx.createLinearGradient(0, -90, 0, 90); g.addColorStop(0, color); g.addColorStop(1, 'rgba(0,0,0,.45)');
    ctx.fillStyle = color; rr(-CFG.RO, -88, CFG.L, 176, 34); ctx.fill(); ctx.fillStyle = g; ctx.fill();
    ctx.fillStyle = 'rgba(8,14,22,.78)'; rr(95, -68, 150, 136, 26); ctx.fill();            // cabin
    ctx.fillStyle = 'rgba(190,225,255,.25)'; rr(235, -62, 38, 124, 12); ctx.fill();          // windscreen
    ctx.fillStyle = '#fff6c8'; for (const y2 of [-62, 62]) { rr(342, y2 - 14, 16, 28, 6); ctx.fill(); }
    ctx.fillStyle = o.rev ? '#ffffff' : '#ff2e4d'; for (const y2 of [-62, 62]) { rr(-86, y2 - 12, 12, 24, 5); ctx.fill(); }
    ctx.restore();
  }

  function draw() {
    const c = App.car, ev = Ctl.ev; ctx.setTransform(SC, 0, 0, SC, 0, 0);
    ctx.fillStyle = '#10161f'; ctx.fillRect(0, 0, CFG.WORLD_W, CFG.WORLD_H);
    ctx.fillStyle = '#2a3342'; ctx.fillRect(0, 0, CFG.WORLD_W, CFG.WALL_Y); ctx.fillRect(0, CFG.CURB_Y, CFG.WORLD_W, 60);
    ctx.fillStyle = '#171e2a'; ctx.fillRect(0, CFG.WALL_Y, CFG.WORLD_W, 320 - CFG.WALL_Y);
    ctx.strokeStyle = 'rgba(255,255,255,.55)'; ctx.lineWidth = 5; ctx.setLineDash([]);
    const sx = World.slotStart, se = sx + World.slotLen;
    for (const x of [CFG.A_X, sx, se, se + CFG.L]) { ctx.beginPath(); ctx.moveTo(x, 140); ctx.lineTo(x, 320); ctx.stroke(); }
    ctx.strokeStyle = 'rgba(255,210,74,.5)'; ctx.setLineDash([40, 30]); ctx.beginPath(); ctx.moveTo(0, 590); ctx.lineTo(CFG.WORLD_W, 590); ctx.stroke(); ctx.setLineDash([]);
    if (ev) { ctx.fillStyle = ev.ok ? 'rgba(46,230,166,.12)' : 'rgba(255,77,109,.16)'; ctx.fillRect(ev.start, 140, ev.len, 180);
      ctx.fillStyle = ev.ok ? '#2ee6a6' : '#ff4d6d'; ctx.font = '600 34px system-ui'; ctx.textAlign = 'center';
      ctx.fillText(Math.round(ev.len) + ' cm — ' + (ev.ok ? 'SUITABLE' : 'TOO SMALL'), ev.start + ev.len / 2, 128 + 0);
      if (ev.ok) { ctx.strokeStyle = '#2ee6a6'; ctx.lineWidth = 4; ctx.setLineDash([18, 10]); ctx.strokeRect(ev.b.x0, 140 + 5, ev.b.x1 - ev.b.x0, 170); ctx.setLineDash([]);
        if (Ctl.pl) drawCar(Ctl.pl.goalX, CFG.BAY_Y, 0, 0, '', { ghost: true }); } }
    for (const r of World.cars) drawCar(r.x + CFG.RO, r.y + 90, 0, 0, r.color);
    for (const r of World.dynamic) { ctx.fillStyle = '#ff9f1a'; ctx.fillRect(r.x, r.y, r.w, r.h); ctx.fillStyle = '#111'; ctx.fillRect(r.x + 10, r.y + 10, r.w - 20, r.h - 20);
      ctx.fillStyle = '#ff9f1a'; ctx.font = '700 22px system-ui'; ctx.textAlign = 'center'; ctx.fillText('OBST', r.x + r.w / 2, r.y + r.h / 2 + 8); }
    ctx.strokeStyle = 'rgba(56,208,255,.35)'; ctx.lineWidth = 6; ctx.beginPath(); App.trail.forEach((p, i) => i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)); ctx.stroke();
    const sc = Slot.scan; if (Ctl.state === 'SEARCHING' && sc && sc.ray) { ctx.strokeStyle = 'rgba(56,208,255,.7)'; ctx.setLineDash([8, 8]); ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(sc.ray.p.x, sc.ray.p.y); ctx.lineTo(sc.ray.p.x - Math.sin(c.th) * 0 + Math.sin(c.th) * sc.ray.r, sc.ray.p.y - Math.cos(c.th) * sc.ray.r); ctx.stroke(); ctx.setLineDash([]); }
    for (const s of App.sens) {   // sensor cones + hit markers
      const lv = Sensors.level(s.d); ctx.fillStyle = COL[lv] + '2e'; ctx.beginPath(); ctx.moveTo(s.x, s.y);
      for (const r of s.rays) ctx.lineTo(s.x + Math.cos(r.a) * r.d, s.y + Math.sin(r.a) * r.d); ctx.closePath(); ctx.fill();
      ctx.strokeStyle = COL[lv]; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(s.x, s.y); ctx.lineTo(s.x + Math.cos(s.ang) * s.d, s.y + Math.sin(s.ang) * s.d); ctx.stroke();
      const hx = s.x + Math.cos(s.ang) * s.d, hy = s.y + Math.sin(s.ang) * s.d;
      ctx.fillStyle = COL[lv]; ctx.beginPath(); ctx.arc(hx, hy, 9, 0, 7); ctx.fill();
      if (s.d < CFG.SENSOR_MAX) { ctx.font = '700 30px system-ui'; ctx.textAlign = 'center'; ctx.fillStyle = '#fff'; ctx.fillText(Math.round(s.d), hx, hy - 16); } }
    drawCar(c.x, c.y, c.th, c.delta, '#38d0ff', { rev: c.gear === 'R' });
    ctx.fillStyle = '#e8f0ff'; ctx.font = '700 28px system-ui'; ctx.textAlign = 'center';
    for (const s of App.sens) { ctx.fillStyle = COL[Sensors.level(s.d)]; ctx.beginPath(); ctx.arc(s.x, s.y, 12, 0, 7); ctx.fill(); }
  }

  const STAGE = { SEARCHING: 0, SPACE_DETECTED: 1, POSITIONING: 2, REVERSE: 3, TURNING: 3, ALIGNING: 3, PARKED: 4 };
  const set = (id, t) => { const e = $(id); if (e.textContent !== String(t)) e.textContent = t; };
  function ui() {
    const c = App.car, st = Ctl.state, ev = Ctl.ev, man = App.mode === 'manual';
    set('vSpeed', Math.abs(c.v / KMH).toFixed(1) + ' km/h'); set('vGear', c.gear);
    const sd = deg(c.delta); set('vSteer', Math.round(sd) + '°'); set('vSteerTxt', Math.abs(sd) < 2 ? 'Straight' : sd < 0 ? 'Left Turn' : 'Right Turn');
    set('vState', man ? 'MANUAL' : st); $('needle').style.transform = 'rotate(' + clamp(sd, -45, 45) * 2 + 'deg)';
    for (const s of App.sens) { const lv = Sensors.level(s.d); set('s' + s.id, s.d >= CFG.SENSOR_MAX ? '250+ cm' : Math.round(s.d) + ' cm');
      const b = $('b' + s.id); b.style.width = clamp(s.d / CFG.SENSOR_MAX * 100, 3, 100) + '%'; b.style.background = COL[lv]; $('s' + s.id).style.color = COL[lv]; }
    set('pLen', ev ? Math.round(ev.len) + ' cm' : '— scanning —'); set('pReq', (CFG.L + 2 * CFG.SAFETY) + ' cm'); set('pMargin', CFG.SAFETY + ' cm');
    set('pStatus', ev ? (ev.ok ? 'AVAILABLE' : 'TOO SMALL') : 'UNKNOWN'); $('pStatus').className = ev ? (ev.ok ? 'ok' : 'bad') : '';
    const hold = App.paused ? 'PAUSED' : st === 'EMERGENCY_STOP' ? 'HALTED' : 'ACTIVE';
    set('sysActive', hold); set('sysAuto', !man && st !== 'IDLE' && st !== 'PARKED' ? 'ON' : 'OFF');
    const lv = App.safety.lvl; set('sysCol', st === 'EMERGENCY_STOP' ? 'EMERGENCY' : ['SAFE', 'WARNING', 'EMERGENCY'][lv]); $('sysCol').style.color = COL[st === 'EMERGENCY_STOP' ? 2 : lv];
    document.querySelectorAll('#pipe li').forEach((li, i) => li.className = i === STAGE[st] ? 'on' : i < (STAGE[st] ?? -1) ? 'done' : '');
    const bn = $('banner'); let cls = '', html = '';
    if (st === 'EMERGENCY_STOP') { cls = 'bad'; html = '⚠ ' + (/sensor/.test(Ctl.msg) ? 'OBSTACLE DETECTED' : 'EMERGENCY') + '<small>EMERGENCY STOP — ' + Ctl.msg + '</small>'; }
    else if (st === 'PARKED') { cls = 'ok'; html = '✓ PARKING COMPLETED<small>Vehicle is safely positioned</small>'; }
    else if (st === 'SLOT_TOO_SMALL') { cls = 'warn'; html = 'SLOT TOO SMALL<small>Required ' + ev.req + ' cm, measured ' + Math.round(ev.len) + ' cm</small>'; }
    else if (App.notice) { cls = 'bad'; html = '⚠ ' + App.notice; }
    else if (lv === 1 && st !== 'IDLE') { cls = 'warn'; html = '⚠ WARNING<small>' + App.safety.msg + '</small>'; }
    bn.className = cls; if (bn.innerHTML !== html) bn.innerHTML = html;
    const r = App.result, rc = $('result'); rc.hidden = !r;
    if (r) { set('rAcc', Math.round(r.acc) + '%'); set('rAli', Math.round(r.align) + '%'); set('rRear', Math.round(r.rear) + ' cm'); set('rFront', Math.round(r.front) + ' cm'); set('rCurb', Math.round(r.curb) + ' cm'); }
    $('btnPause').disabled = App.paused; $('btnResume').disabled = !App.paused; $('btnManual').classList.toggle('active', man); $('btnManual').textContent = man ? 'MANUAL: ON' : 'MANUAL DRIVE';
    $('pad').hidden = !man;
  }
  App.onLog = () => { $('log').innerHTML = App.log.map(m => '<li>' + m + '</li>').join(''); };

  $('btnStart').onclick = () => App.startAuto(false); $('btnDemo').onclick = () => App.startAuto(true);
  $('btnPause').onclick = () => App.paused = true; $('btnResume').onclick = () => App.paused = false;
  $('btnEstop').onclick = () => App.estop(); $('btnReset').onclick = () => App.reset();
  $('btnManual').onclick = () => App.setManual(App.mode !== 'manual'); $('btnObs').onclick = () => App.dropObstacle();
  document.querySelectorAll('[data-slot]').forEach(b => b.onclick = () => { App.reset(b.dataset.slot); document.querySelectorAll('[data-slot]').forEach(x => x.classList.toggle('active', x === b)); });
  addEventListener('keydown', e => { const k = e.key.length === 1 ? e.key.toLowerCase() : e.key; if (App.mode === 'manual' && (k.startsWith('Arrow') || k === ' ' || 'wasd'.includes(k))) { Manual.keys[k] = true; e.preventDefault(); } });
  addEventListener('keyup', e => { const k = e.key.length === 1 ? e.key.toLowerCase() : e.key; Manual.keys[k] = false; });
  document.querySelectorAll('#pad button').forEach(b => { const k = b.dataset.k; b.onpointerdown = () => Manual.keys[k] = true; b.onpointerup = b.onpointerleave = () => Manual.keys[k] = false; });

  App.reset('medium'); document.querySelector('[data-slot="medium"]').classList.add('active');
  let last = performance.now(), acc = 0;
  (function loop(now) {
    acc += Math.min(0.1, (now - last) / 1000); last = now;
    while (acc >= 1 / 120) { App.step(1 / 120); acc -= 1 / 120; }
    draw(); ui(); requestAnimationFrame(loop);
  })(last);
}

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filename: plan_gui.py
import pygame, json, math, time
from pathlib import Path

global CM_PER_PX, FLIGHT_SPEED, FLIGHT_HEIGHT
global GRID_W_M, GRID_H_M, GRID_CELL_M
# -------------------- Window & UI --------------------
W, H     = 1000, 720
PANEL_W  = 240
FPS      = 60
TITLE    = "Drone Planner"

BG_COLOR    = (245,245,245)
GRID_COLOR  = (220,220,220)
AXIS_COLOR  = (160,160,160)
CENTER_DOT  = (40,110,255)
PATH_COLOR  = (255,60,60)
POINT_COLOR = (0,0,0)
TEXT_COLOR  = (30,30,30)
GHOST_COLOR = (40,40,40)

BG_STEP_PX = 50            # background grid spacing (px)

# -------------------- Flight / Map Settings --------------------
CM_PER_PX      = 3.0       # 1 px = 3 cm (edit in Settings)
FLIGHT_SPEED   = 30        # cm/s (written to JSON meta -> used by main.py)
FLIGHT_HEIGHT  = 90        # cm  (written to JSON meta -> used by main.py)

# -------------------- Grid Defaults (meters) --------------------
GRID_W_M       = 3.0
GRID_H_M       = 2.0
GRID_CELL_M    = 0.6
GRID_ORIENT    = "X"       # "X" = horizontal sweeps; "Y" = vertical sweeps

# Click blink effect
BLINK_MS       = 400

# -------------------- Save/Load base dir --------------------
BASE_DIR = Path(__file__).resolve().parent
SAVE_DIR = BASE_DIR / "plans"
SAVE_DIR.mkdir(parents=True, exist_ok=True)  # ensure plans/ exists

# ---------------------------------------------------------------
pygame.init()
screen  = pygame.display.set_mode((W, H))
pygame.display.set_caption(TITLE)
clock   = pygame.time.Clock()
font    = pygame.font.SysFont(None, 20)
bigfont = pygame.font.SysFont(None, 28)
tiny    = pygame.font.SysFont(None, 16)

CENTER = ((W - PANEL_W)//2, H//2)

MODE_FREE = 0
MODE_GRID = 1
mode = MODE_FREE

points      = []     # free mode points (absolute px, no CENTER inside)
redo_stack  = []
grid_path   = []     # grid mode points (absolute px, includes CENTER at [0])
blink_fx    = []     # [{"pos":(x,y), "t0":ms}, ...]
active_popup = None
info_msg     = ""
mouse_pos    = (0,0)

# -------------------- Math helpers --------------------
def cm_per_px(): return CM_PER_PX
def px_to_cm(px): return px * cm_per_px()

def dist_cm_px(p0, p1):
    dx = p1[0]-p0[0]; dy = p1[1]-p0[1]
    d_px = math.hypot(dx, dy)
    return int(round(px_to_cm(d_px))), d_px

# angle (0..180) between lines (pos0->posref) and (pos1->posref)
def get_angle_btw_line(pos0, pos1, posref):
    ax = posref[0] - pos0[0]; ay = posref[1] - pos0[1]
    bx = posref[0] - pos1[0]; by = posref[1] - pos1[1]
    magA = math.hypot(ax, ay); magB = math.hypot(bx, by)
    if magA == 0 or magB == 0: return 0
    dot = (ax*bx + ay*by) / (magA*magB)
    dot = max(-1.0, min(1.0, dot))
    return int(round(math.degrees(math.acos(dot))))

# signed turn at posref from (pos0->posref) to (pos1->posref); left/CCW positive
def get_signed_angle_btw_line(pos0, pos1, posref):
    ax = posref[0] - pos0[0]; ay = posref[1] - pos0[1]
    bx = posref[0] - pos1[0]; by = posref[1] - pos1[1]
    magA = math.hypot(ax, ay); magB = math.hypot(bx, by)
    if magA == 0 or magB == 0: return 0
    dot = (ax*bx + ay*by) / (magA*magB)
    dot = max(-1.0, min(1.0, dot))
    unsigned = math.degrees(math.acos(dot))
    cross = ax*by - ay*bx
    return int(round(unsigned if cross > 0 else (-unsigned if cross < 0 else 0)))

def points_equal(a,b): return a[0]==b[0] and a[1]==b[1]

def total_distance_cm(path_points):
    if len(path_points) < 2: return 0
    s = 0
    for i in range(1, len(path_points)):
        dcm,_ = dist_cm_px(path_points[i-1], path_points[i]); s += dcm
    return s

def format_time(sec):
    if sec < 60: return f"{sec:.1f}s"
    m = int(sec // 60); s = sec - 60*m
    return f"{m}m {s:.0f}s"

# -------------------- Drawing --------------------
def draw_centered_grid(surface):
    surface.fill(BG_COLOR)
    # axes through CENTER
    pygame.draw.line(surface, AXIS_COLOR, (CENTER[0], 0), (CENTER[0], H), 1)
    pygame.draw.line(surface, AXIS_COLOR, (0, CENTER[1]), (W-PANEL_W, CENTER[1]), 1)

    # grid lines centered on CENTER
    step = BG_STEP_PX
    x = CENTER[0]
    while x >= 0:
        pygame.draw.line(surface, GRID_COLOR, (x, 0), (x, H), 1); x -= step
    x = CENTER[0] + step
    while x <= W-PANEL_W:
        pygame.draw.line(surface, GRID_COLOR, (x, 0), (x, H), 1); x += step

    y = CENTER[1]
    while y >= 0:
        pygame.draw.line(surface, GRID_COLOR, (0, y), (W-PANEL_W, y), 1); y -= step
    y = CENTER[1] + step
    while y <= H:
        pygame.draw.line(surface, GRID_COLOR, (0, y), (W-PANEL_W, y), 1); y += step

    pygame.draw.circle(surface, CENTER_DOT, CENTER, 5)

def add_blink(pos):
    blink_fx.append({"pos": pos, "t0": pygame.time.get_ticks()})

def draw_blinks(surf):
    now = pygame.time.get_ticks()
    i = 0
    while i < len(blink_fx):
        fx = blink_fx[i]
        dt = now - fx["t0"]
        if dt >= BLINK_MS: blink_fx.pop(i); continue
        prog = dt / BLINK_MS
        radius = int(4 + 18*prog)
        alpha  = max(0, int(220 * (1 - prog)))
        ring = pygame.Surface((radius*2+4, radius*2+4), pygame.SRCALPHA)
        pygame.draw.circle(ring, (40,110,255,alpha), (radius+2, radius+2), radius, width=3)
        surf.blit(ring, (fx["pos"][0]-radius-2, fx["pos"][1]-radius-2))
        i += 1

# -------------------- UI --------------------
class Button:
    def __init__(self, rect, text):
        self.rect = pygame.Rect(rect); self.text = text
    def draw(self, surf):
        pygame.draw.rect(surf, (250,250,250), self.rect, border_radius=8)
        pygame.draw.rect(surf, (200,200,200), self.rect, 1, border_radius=8)
        surf.blit(font.render(self.text, True, TEXT_COLOR), (self.rect.x+10, self.rect.y+8))
    def hit(self, pos): return self.rect.collidepoint(pos)

buttons = [
    Button((W-PANEL_W+20,  20, 200, 32), "Mode: Free"),
    Button((W-PANEL_W+20,  62, 200, 32), "Gen Grid"),
    Button((W-PANEL_W+20, 104, 200, 32), "Grid Dir: X→Y"),
    Button((W-PANEL_W+20, 146, 200, 32), "Undo (Z)"),
    Button((W-PANEL_W+20, 188, 200, 32), "Redo (Y)"),
    Button((W-PANEL_W+20, 230, 200, 32), "Clear"),
    Button((W-PANEL_W+20, 272, 200, 32), "Settings"),
    Button((W-PANEL_W+20, 314, 200, 32), "Save JSON"),
    Button((W-PANEL_W+20, 356, 200, 32), "Load JSON (L)"),
    Button((W-PANEL_W+20, 398, 200, 32), "Back to Base (H)"),
]

class PopupInput:
    def __init__(self, title, fields):
        self.title  = title
        self.fields = fields[:]  # [(label, str), ...]
        self.active = 0
        self.done   = False
        self.ok     = False
        self.rect   = pygame.Rect(120, 120, 520, 260)
    def draw(self, surf):
        pygame.draw.rect(surf, (255,255,255), self.rect, border_radius=10)
        pygame.draw.rect(surf, (60,60,60), self.rect, 2, border_radius=10)
        surf.blit(bigfont.render(self.title, True, (10,10,10)), (self.rect.x+16, self.rect.y+12))
        y = self.rect.y + 60
        for i,(lab,val) in enumerate(self.fields):
            surf.blit(font.render(lab, True, (20,20,20)), (self.rect.x+16, y+3))
            box = pygame.Rect(self.rect.x+260, y, 220, 28)
            pygame.draw.rect(surf, (250,250,250), box, border_radius=6)
            pygame.draw.rect(surf, (120,120,120), box, 1, border_radius=6)
            if i == self.active:
                pygame.draw.rect(surf, (80,160,255), box, 2, border_radius=6)
            surf.blit(font.render(val, True, (0,0,0)), (box.x+8, box.y+6))
            y += 40
        okb  = pygame.Rect(self.rect.x+300, self.rect.y+self.rect.h-46, 80, 30)
        canb = pygame.Rect(self.rect.x+390, self.rect.y+self.rect.h-46, 80, 30)
        for r,c1,c2,t in [(okb,(230,245,230),(200,200,200),"OK"), (canb,(245,230,230),(200,200,200),"Cancel")]:
            pygame.draw.rect(surf, c1, r, border_radius=6)
            pygame.draw.rect(surf, c2, r, 1, border_radius=6)
            surf.blit(font.render(t, True, (0,80,0) if t=="OK" else (120,0,0)),
                      (r.x+18 if t=="Cancel" else r.x+28, r.y+7))
        return okb, canb
    def handle_event(self, e):
        if e.type == pygame.KEYDOWN:
            if   e.key == pygame.K_TAB:    self.active = (self.active+1) % len(self.fields)
            elif e.key == pygame.K_RETURN: self.done = True; self.ok = True
            elif e.key == pygame.K_ESCAPE: self.done = True; self.ok = False
            elif e.key == pygame.K_BACKSPACE:
                lab,val = self.fields[self.active]; self.fields[self.active] = (lab, val[:-1])
            else:
                ch = e.unicode
                if ch and (ch.isdigit() or ch in ".-"):
                    lab,val = self.fields[self.active]; self.fields[self.active] = (lab, val + ch)
        elif e.type == pygame.MOUSEBUTTONDOWN:
            okb, canb = self.draw(screen)
            if   okb.collidepoint(e.pos):  self.done = True; self.ok = True
            elif canb.collidepoint(e.pos): self.done = True; self.ok = False

# -------------------- Grid gen --------------------
def gen_lawnmower(center, w_m, h_m, cell_m, orient="X"):
    w_px    = (w_m*100.0) / cm_per_px()
    h_px    = (h_m*100.0) / cm_per_px()
    cell_px = max(1.0, (cell_m*100.0) / cm_per_px())

    cols = max(1, int(math.floor(w_px / cell_px)) + 1)
    rows = max(1, int(math.floor(h_px / cell_px)) + 1)

    x0 = center[0] - w_px/2
    y0 = center[1] - h_px/2

    pts = []
    direction = 1

    if orient == "X":
        y = 0.0
        for _ in range(rows):
            xs_idx = (range(cols) if direction==1 else range(cols-1, -1, -1))
            for ci in xs_idx:
                x = x0 + ci*cell_px
                yabs = y0 + y
                pts.append((int(round(x)), int(round(yabs))))
            y += cell_px
            direction *= -1
            if y > h_px + 1e-6: break
    else:
        x = 0.0
        for _ in range(cols):
            ys_idx = (range(rows) if direction==1 else range(rows-1, -1, -1))
            for ri in ys_idx:
                y = y0 + ri*cell_px
                xabs = x0 + x
                pts.append((int(round(xabs)), int(round(y))))
            x += cell_px
            direction *= -1
            if x > w_px + 1e-6: break

    pts.insert(0, center)  # start from center
    return pts

# -------------------- Path → Segments --------------------
def compute_segments(path_points):
    n = len(path_points)
    if n < 2: return [], [], [], []
    dcm, dpx, ang_u, ang_s = [], [], [0], [0]
    for i in range(1, n):
        cm, px = dist_cm_px(path_points[i-1], path_points[i])
        dcm.append(int(cm)); dpx.append(int(round(px)))
    for i in range(1, n-1):
        au = get_angle_btw_line(path_points[i-1], path_points[i+1], path_points[i])
        asg= get_signed_angle_btw_line(path_points[i-1], path_points[i+1], path_points[i])
        ang_u.append(int(au)); ang_s.append(int(asg))
    return dcm, dpx, ang_u, ang_s

# -------------------- Save / Load JSON (always in plans/) --------------------
def save_json_core(path_points, meta, filename="beta_waypoint.json"):
    dcm, dpx, ang_u, ang_s = compute_segments(path_points)
    beta_waypoints = []
    for i in range(len(dcm)):
        beta_waypoints.append({
            "dist_cm": int(dcm[i]),
            "dist_px": int(dpx[i]),
            "angle_deg": int(ang_u[i]) if i < len(ang_u) else 0,
            "turn_signed_deg": int(ang_s[i]) if i < len(ang_s) else 0
        })
    data = {"wp": beta_waypoints, "pos": path_points, "meta": meta}
    out_path = (SAVE_DIR / filename)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)
    return len(beta_waypoints), str(out_path)

def save_json(path_points, meta):
    nseg, outp = save_json_core(path_points, meta, "beta_waypoint.json")
    print(f"[Saved] {outp} with {nseg} segments")

def save_json_timestamp(path_points, meta):
    ts = time.strftime("%Y%m%d_%H%M%S")
    fname = f"beta_waypoint_{ts}.json"
    nseg, outp = save_json_core(path_points, meta, fname)
    print(f"[Saved] {outp} with {nseg} segments")

def find_latest_beta_waypoint_json():
    cands = []
    cands += list(SAVE_DIR.glob("beta_waypoint.json"))
    cands += list(SAVE_DIR.glob("beta_waypoint_*.json"))
    cands += list(SAVE_DIR.glob("beta_waypoint-*.json"))
    if not cands:
        return None
    return max(cands, key=lambda p: p.stat().st_mtime)

def load_json(path=None):
    """Loads last path + meta from plans/; updates CENTER, cm/px, speed/height, mode, and path arrays."""
    global points, grid_path, mode, CM_PER_PX, FLIGHT_SPEED, FLIGHT_HEIGHT, CENTER, info_msg
    try:
        if path is None:
            p = find_latest_beta_waypoint_json()
            path = p if p is not None else (SAVE_DIR / "beta_waypoint.json")
        else:
            path = Path(path)
            if not path.is_absolute():
                path = SAVE_DIR / path

        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        pos  = data.get("pos", [])
        meta = data.get("meta", {})
        if not pos:
            info_msg = f"No 'pos' in {path.name}"
            return

        # update CENTER
        cpx = meta.get("center_px")
        if isinstance(cpx, (list, tuple)) and len(cpx) == 2:
            CENTER = (int(cpx[0]), int(cpx[1]))

        # update map/flight settings
        CM_PER_PX     = float(meta.get("cm_per_px", CM_PER_PX))
        FLIGHT_SPEED  = int(float(meta.get("speed_cm_s", FLIGHT_SPEED)))
        FLIGHT_HEIGHT = int(float(meta.get("height_cm",  FLIGHT_HEIGHT)))

        # restore mode & path
        m = meta.get("mode", "FREE").upper()
        if m == "GRID":
            mode = MODE_GRID
            grid_path = [(int(x),int(y)) for x,y in pos]
            points.clear()
        else:
            mode = MODE_FREE
            points = [(int(x),int(y)) for x,y in pos[1:]] if len(pos) >= 1 else []
            grid_path.clear()

        info_msg = f"Loaded {path.name} | mode={m} | points={len(pos)}"
        add_blink(CENTER)
    except FileNotFoundError:
        info_msg = f"No beta_waypoint file found in {SAVE_DIR}"
    except Exception as e:
        info_msg = f"Load error: {e!r}"

# -------------------- Back to base --------------------
def back_to_base():
    global points, grid_path, mode, info_msg
    if mode == MODE_FREE:
        path = [CENTER] + points
        last = path[-1]
        if not points_equal(last, CENTER):
            points.append(CENTER); add_blink(CENTER)
            info_msg = "Added return-to-base segment."
        else:
            info_msg = "Already at base."
    else:
        if not grid_path:
            grid_path = [CENTER]; add_blink(CENTER)
            info_msg = "Path empty; base set."
        else:
            last = grid_path[-1]
            if not points_equal(last, CENTER):
                grid_path.append(CENTER); add_blink(CENTER)
                info_msg = "Added return-to-base segment."
            else:
                info_msg = "Already at base."

# -------------------- Main Loop --------------------
running = True
while running:
    draw_centered_grid(screen)

    # build current path
    if mode == MODE_FREE:
        path = [CENTER] + points
    else:
        path = grid_path[:] if grid_path else [CENTER]

    # draw path
    if len(path) >= 2:
        pygame.draw.lines(screen, PATH_COLOR, False, path, 2)
    for p in path:
        pygame.draw.circle(screen, POINT_COLOR, p, 3)

    # per-segment labels
    if len(path) >= 2:
        for i in range(1, len(path)):
            cm,_ = dist_cm_px(path[i-1], path[i])
            mx = (path[i-1][0] + path[i][0])//2
            my = (path[i-1][1] + path[i][1])//2
            screen.blit(tiny.render(f"{cm} cm", True, (60,60,60)), (mx+6, my+6))

    # live mouse preview (FREE mode)
    if active_popup is None and mode == MODE_FREE:
        mx, my = mouse_pos
        if mx < W-PANEL_W:
            anchor = path[-1]
            pygame.draw.line(screen, GHOST_COLOR, anchor, (mx,my), 1)
            pygame.draw.circle(screen, (30,30,30), (mx,my), 3)
            cm,_ = dist_cm_px(anchor, (mx,my))
            eta  = cm / max(1e-6, FLIGHT_SPEED)
            tip  = f"{cm} cm ({cm/100:.2f} m), ETA {format_time(eta)}"
            tiptxt = font.render(tip, True, (20,20,20))
            tw, th = tiptxt.get_size()
            bx, by = mx+12, my-10-th
            pygame.draw.rect(screen, (255,255,255), (bx-6, by-4, tw+12, th+8), border_radius=6)
            pygame.draw.rect(screen, (180,180,180), (bx-6, by-4, tw+12, th+8), 1, border_radius=6)
            screen.blit(tiptxt, (bx, by))

    # HUD (top left)
    mode_str = "FREE" if mode == MODE_FREE else "GRID"
    pts_cnt  = (len(points) if mode==MODE_FREE else (len(grid_path)-1 if grid_path else 0))
    hud1 = f"Mode: {mode_str} | Points: {pts_cnt} | Speed={FLIGHT_SPEED}cm/s  Height={FLIGHT_HEIGHT}cm"
    screen.blit(font.render(hud1, True, TEXT_COLOR), (10, 10))

    bg_m = (BG_STEP_PX * CM_PER_PX) / 100.0
    total_cm = total_distance_cm(path)
    total_eta = total_cm / max(1e-6, FLIGHT_SPEED)
    screen.blit(font.render(f"BG grid: {BG_STEP_PX}px approx. {bg_m:.2f} m", True, TEXT_COLOR), (10, 32))
    screen.blit(font.render(f"Total: {total_cm} cm ({total_cm/100:.2f} m)  ETA: {format_time(total_eta)}", True, TEXT_COLOR), (10, 52))

    if mode == MODE_GRID and grid_path:
        screen.blit(font.render(f"Grid: {GRID_W_M:.2f}×{GRID_H_M:.2f} m, cell {GRID_CELL_M:.2f} m, orient {GRID_ORIENT}", True, TEXT_COLOR), (10, 72))

    if info_msg:
        screen.blit(font.render(info_msg, True, (120,0,0)), (10, 94))

    # right panel & buttons
    pygame.draw.rect(screen, (250,250,250), (W-PANEL_W, 0, PANEL_W, H))
    pygame.draw.line(screen, (200,200,200), (W-PANEL_W, 0), (W-PANEL_W, H), 1)
    buttons[0].text = "Mode: Free" if mode==MODE_FREE else "Mode: Grid"
    buttons[2].text = f"Grid Dir: {'X→Y' if GRID_ORIENT=='X' else 'Y→X'}"
    for b in buttons: b.draw(screen)

    draw_blinks(screen)
    if active_popup: active_popup.draw(screen)

    pygame.display.flip()

    # -------------------- Events --------------------
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False

        elif e.type == pygame.MOUSEMOTION:
            mouse_pos = e.pos
        
        elif active_popup:
            active_popup.handle_event(e)
            if active_popup.done:
                if active_popup.ok:
                    if active_popup.title.startswith("Settings"):
                        try:
                            
                            FLIGHT_SPEED  = int(float(active_popup.fields[0][1]))
                            FLIGHT_HEIGHT = int(float(active_popup.fields[1][1]))
                            CM_PER_PX     = float(active_popup.fields[2][1])
                            info_msg = "Settings updated."
                        except Exception:
                            info_msg = "Invalid settings input."
                    else:
                        # Grid params popup
                        try:
                            
                            GW = float(active_popup.fields[0][1])
                            GH = float(active_popup.fields[1][1])
                            GC = float(active_popup.fields[2][1])
                            if GW<=0 or GH<=0 or GC<=0: raise ValueError
                            GRID_W_M, GRID_H_M, GRID_CELL_M = GW, GH, GC
                            grid_path = gen_lawnmower(CENTER, GRID_W_M, GRID_H_M, GRID_CELL_M, GRID_ORIENT)
                            mode = MODE_GRID
                            info_msg = f"Generated grid {GRID_W_M}x{GRID_H_M}m, cell {GRID_CELL_M}m, orient {GRID_ORIENT}."
                            add_blink(CENTER)
                        except Exception:
                            info_msg = "Invalid grid input."
                active_popup = None

        else:
            if e.type == pygame.MOUSEBUTTONDOWN:
                mx, my = e.pos
                if mx < W-PANEL_W:
                    if mode == MODE_FREE:
                        pt = (mx, my)
                        points.append(pt); redo_stack.clear(); add_blink(pt)
                # buttons
                for i, b in enumerate(buttons):
                    if b.hit((mx,my)):
                        if   i == 0:  # Mode toggle
                            mode = MODE_FREE if mode==MODE_GRID else MODE_GRID
                            info_msg = f"Mode set to {'FREE' if mode==MODE_FREE else 'GRID'}."
                        elif i == 1:  # Gen Grid (popup)
                            active_popup = PopupInput(
                                "Grid Params (meters)",
                                [("Width (m)", f"{GRID_W_M}"),
                                 ("Height (m)", f"{GRID_H_M}"),
                                 ("Cell (m)", f"{GRID_CELL_M}")]
                            )
                        elif i == 2:  # Grid orientation toggle
                            GRID_ORIENT = "Y" if GRID_ORIENT=="X" else "X"
                            if mode == MODE_GRID and GRID_W_M>0 and GRID_H_M>0 and GRID_CELL_M>0:
                                grid_path = gen_lawnmower(CENTER, GRID_W_M, GRID_H_M, GRID_CELL_M, GRID_ORIENT)
                            info_msg = f"Grid orientation → {GRID_ORIENT}"
                        elif i == 3:  # Undo
                            if mode == MODE_FREE and points:
                                redo_stack.append(points.pop())
                        elif i == 4:  # Redo
                            if mode == MODE_FREE and redo_stack:
                                points.append(redo_stack.pop())
                        elif i == 5:  # Clear
                            if mode == MODE_FREE: points.clear(); redo_stack.clear()
                            else: grid_path.clear()
                        elif i == 6:  # Settings (flight & map)
                            active_popup = PopupInput(
                                "Settings (flight/map)",
                                [("Speed (cm/s)", f"{FLIGHT_SPEED}"),
                                 ("Height (cm)", f"{FLIGHT_HEIGHT}"),
                                 ("cm per px", f"{CM_PER_PX}")]
                            )
                        elif i == 7:  # Save JSON (to plans/beta_waypoint.json)
                            path_points = ([CENTER] + points) if mode==MODE_FREE else (grid_path[:] if grid_path else [CENTER])
                            meta = {
                                "mode": "FREE" if mode==MODE_FREE else "GRID",
                                "speed_cm_s": FLIGHT_SPEED,
                                "height_cm": FLIGHT_HEIGHT,
                                "cm_per_px": CM_PER_PX,
                                "center_px": CENTER
                            }
                            save_json(path_points, meta)
                        elif i == 8:  # Load JSON (from plans/)
                            load_json(None)
                        elif i == 9:  # Back to Base
                            back_to_base()

            elif e.type == pygame.KEYDOWN:
                if   e.key == pygame.K_z:  # undo
                    if mode == MODE_FREE and points: redo_stack.append(points.pop())
                elif e.key == pygame.K_y:  # redo
                    if mode == MODE_FREE and redo_stack: points.append(redo_stack.pop())
                elif e.key == pygame.K_h:  # hotkey: back to base
                    back_to_base()
                elif e.key == pygame.K_l:  # hotkey: load json
                    load_json(None)

    clock.tick(FPS)

pygame.quit()

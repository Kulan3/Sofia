#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pygame, json, math

# -------------------- Config --------------------
W, H = 900, 700          # window size
PANEL_W = 200            # right control panel width
FPS = 60
BG_COLOR = (245,245,245)
BG_STEP_PX = 50
GRID_COLOR = (220,220,220)
AXIS_COLOR = (160,160,160)
CENTER_DOT = (40,110,255)
PATH_COLOR = (255,60,60)
POINT_COLOR = (0,0,0)
TEXT_COLOR = (30,30,30)

# Map scale: cm per pixel (e.g., 1 px = 3.0 cm)
CM_PER_PX = 3.0

# Flight settings (will be saved to JSON)
FLIGHT_SPEED  = 30   # cm/s
FLIGHT_HEIGHT = 90   # cm

# Grid defaults (meters)
GRID_W_M = 3.0
GRID_H_M = 2.0
GRID_CELL_M = 0.6

# Click blink effect
BLINK_MS = 400  # total duration in ms
# ------------------------------------------------

pygame.init()
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("Drone sonic boom bomm")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)
bigfont = pygame.font.SysFont(None, 28)

CENTER = ((W - PANEL_W)//2, H//2)

# ---------- Helpers ----------
def draw_grid(surface):
    surface.fill(BG_COLOR)
    # axes at CENTER
    pygame.draw.line(surface, AXIS_COLOR, (CENTER[0], 0), (CENTER[0], H), 1)
    pygame.draw.line(surface, AXIS_COLOR, (0, CENTER[1]), (W-PANEL_W, CENTER[1]), 1)
    # background grid
    step = BG_STEP_PX
    for x in range(0, W-PANEL_W, step):
        pygame.draw.line(surface, GRID_COLOR, (x,0), (x,H), 1)
    for y in range(0, H, step):
        pygame.draw.line(surface, GRID_COLOR, (0,y), (W-PANEL_W,y), 1)
    # center dot
    pygame.draw.circle(surface, CENTER_DOT, CENTER, 5)

def cm_per_px():
    return CM_PER_PX

def px_to_cm(px):
    return px * cm_per_px()

def dist_cm(p0, p1):
    dx = p1[0]-p0[0]
    dy = p1[1]-p0[1]
    d_px = math.hypot(dx, dy)
    return int(round(px_to_cm(d_px))), int(round(d_px))

# ---- Your angle (0..180°) ----
def get_angle_btw_line(pos0, pos1, posref):
    """
    Angle (0..180) between segments (pos0->posref) and (pos1->posref),
    i.e., the turn at 'posref' between the incoming and outgoing legs.
    """
    ax = posref[0] - pos0[0]
    ay = posref[1] - pos0[1]
    bx = posref[0] - pos1[0]
    by = posref[1] - pos1[1]

    magA = math.hypot(ax, ay)
    magB = math.hypot(bx, by)
    if magA == 0 or magB == 0:
        return 0

    dot = (ax * bx + ay * by) / (magA * magB)
    dot = max(-1.0, min(1.0, dot))          # clamp for safety
    angle_deg = math.degrees(math.acos(dot)) # 0..180
    return int(round(angle_deg))

# ---- Signed version: left +, right - ([-180, 180]) ----
def get_signed_angle_btw_line(pos0, pos1, posref):
    """
    Signed turn angle at 'posref' from (pos0->posref) to (pos1->posref).
    Positive = left (CCW), Negative = right (CW).
    """
    ax = posref[0] - pos0[0]
    ay = posref[1] - pos0[1]
    bx = posref[0] - pos1[0]
    by = posref[1] - pos1[1]

    magA = math.hypot(ax, ay)
    magB = math.hypot(bx, by)
    if magA == 0 or magB == 0:
        return 0

    dot = (ax * bx + ay * by) / (magA * magB)
    dot = max(-1.0, min(1.0, dot))
    unsigned = math.degrees(math.acos(dot))

    # cross product z-component (2D)
    cross = ax * by - ay * bx
    signed = unsigned if cross > 0 else (-unsigned if cross < 0 else 0)
    return int(round(signed))

def points_equal(p, q):
    return p[0] == q[0] and p[1] == q[1]

# Back-to-base helper
def back_to_base():
    global points, grid_path, mode, info_msg
    if mode == MODE_FREE:
        last = ( [CENTER] + points )[-1] if points else CENTER
        if not points_equal(last, CENTER):
            points.append(CENTER)
            add_blink(CENTER)
            info_msg = "Added return-to-base segment."
        else:
            info_msg = "Already at base."
    else:
        if not grid_path:
            grid_path = [CENTER]
            info_msg = "Path empty; set base as only point."
            add_blink(CENTER)
            return
        last = grid_path[-1]
        if not points_equal(last, CENTER):
            grid_path.append(CENTER)
            add_blink(CENTER)
            info_msg = "Added return-to-base segment."
        else:
            info_msg = "Already at base."

# ---------- UI ----------
class Button:
    def __init__(self, rect, text, key=None):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.key = key
    def draw(self, surf):
        pygame.draw.rect(surf, (250,250,250), self.rect, border_radius=8)
        pygame.draw.rect(surf, (200,200,200), self.rect, 1, border_radius=8)
        txt = font.render(self.text, True, TEXT_COLOR)
        surf.blit(txt, (self.rect.x+10, self.rect.y+8))
    def hit(self, pos):
        return self.rect.collidepoint(pos)

# Right panel buttons (added "Back to Base")
buttons = [
    Button((W-PANEL_W+20,  20, 160, 32), "Mode: Free"),
    Button((W-PANEL_W+20,  62, 160, 32), "Gen Grid"),
    Button((W-PANEL_W+20, 104, 160, 32), "Undo (Z)"),
    Button((W-PANEL_W+20, 146, 160, 32), "Redo (Y)"),
    Button((W-PANEL_W+20, 188, 160, 32), "Clear"),
    Button((W-PANEL_W+20, 230, 160, 32), "Settings"),
    Button((W-PANEL_W+20, 272, 160, 32), "Save JSON"),
    Button((W-PANEL_W+20, 314, 160, 32), "Back to Base (H)"),
]

MODE_FREE = 0
MODE_GRID = 1
mode = MODE_FREE

# data
points = []         # user points (Free mode)
redo_stack = []     # for redo in Free mode
grid_path = []      # generated path in Grid mode

# blink effects
blink_fx = []       # list of dicts: {"pos": (x,y), "t0": ms}
def add_blink(pos):
    blink_fx.append({"pos": pos, "t0": pygame.time.get_ticks()})

# popups (simple text input)
class PopupInput:
    def __init__(self, title, fields):
        self.title = title
        self.fields = fields[:]  # list of (label, value_str)
        self.active = 0
        self.done = False
        self.ok = False
        self.rect = pygame.Rect(120, 120, 500, 240)
    def draw(self, surf):
        pygame.draw.rect(surf, (255,255,255), self.rect, border_radius=10)
        pygame.draw.rect(surf, (60,60,60), self.rect, 2, border_radius=10)
        title = bigfont.render(self.title, True, (10,10,10))
        surf.blit(title, (self.rect.x+16, self.rect.y+12))
        # fields
        y = self.rect.y + 60
        for i,(lab,val) in enumerate(self.fields):
            lbl = font.render(lab, True, (20,20,20))
            surf.blit(lbl, (self.rect.x+16, y+3))
            box = pygame.Rect(self.rect.x+220, y, 220, 28)
            pygame.draw.rect(surf, (250,250,250), box, border_radius=6)
            pygame.draw.rect(surf, (120,120,120), box, 1, border_radius=6)
            if i == self.active:
                pygame.draw.rect(surf, (80,160,255), box, 2, border_radius=6)
            valtxt = font.render(val, True, (0,0,0))
            surf.blit(valtxt, (box.x+8, box.y+6))
            y += 40
        # buttons
        okb = pygame.Rect(self.rect.x+280, self.rect.y+self.rect.h-46, 80, 30)
        canb= pygame.Rect(self.rect.x+370, self.rect.y+self.rect.h-46, 80, 30)
        pygame.draw.rect(surf, (230,245,230), okb, border_radius=6)
        pygame.draw.rect(surf, (200,200,200), okb, 1, border_radius=6)
        pygame.draw.rect(surf, (245,230,230), canb, border_radius=6)
        pygame.draw.rect(surf, (200,200,200), canb, 1, border_radius=6)
        surf.blit(font.render("OK", True, (0,80,0)), (okb.x+28, okb.y+7))
        surf.blit(font.render("Cancel", True, (120,0,0)), (canb.x+18, canb.y+7))
        return okb, canb
    def handle_event(self, e):
        if e.type == pygame.KEYDOWN:
            if e.key == pygame.K_TAB:
                self.active = (self.active + 1) % len(self.fields)
            elif e.key == pygame.K_RETURN:
                self.done = True; self.ok = True
            elif e.key == pygame.K_ESCAPE:
                self.done = True; self.ok = False
            elif e.key == pygame.K_BACKSPACE:
                lab, val = self.fields[self.active]
                self.fields[self.active] = (lab, val[:-1])
            else:
                ch = e.unicode
                if ch and (ch.isdigit() or ch in ".-"):
                    lab, val = self.fields[self.active]
                    self.fields[self.active] = (lab, val + ch)
        elif e.type == pygame.MOUSEBUTTONDOWN:
            okb = pygame.Rect(self.rect.x+280, self.rect.y+self.rect.h-46, 80, 30)
            canb= pygame.Rect(self.rect.x+370, self.rect.y+self.rect.h-46, 80, 30)
            if okb.collidepoint(e.pos):
                self.done = True; self.ok = True
            elif canb.collidepoint(e.pos):
                self.done = True; self.ok = False

def gen_lawnmower(center, w_m, h_m, cell_m):
    """Generate absolute pixel points starting at center, lawnmower pattern."""
    w_px   = (w_m*100.0) / cm_per_px()
    h_px   = (h_m*100.0) / cm_per_px()
    cell_px= (cell_m*100.0) / cm_per_px()
    if cell_px <= 0:  # safety
        cell_px = 1.0

    # floor to avoid overshoot; +1 to include boundary
    cols = max(1, int(math.floor(w_px / cell_px)) + 1)
    rows = max(1, int(math.floor(h_px / cell_px)) + 1)

    x0 = center[0] - w_px/2
    y0 = center[1] - h_px/2

    pts = []
    direction = 1
    y = 0.0
    for _ in range(rows):
        xs = [i*cell_px for i in (range(cols) if direction==1 else range(cols-1,-1,-1))]
        for xv in xs:
            x = x0 + xv
            yabs = y0 + y
            pts.append((int(round(x)), int(round(yabs))))
        y += cell_px
        direction *= -1
        if y > h_px + 1e-6:
            break

    # start from center
    pts.insert(0, center)
    return pts

def compute_segments(all_points):
    """
    Input: absolute pixel points, first = start (center)
    Returns:
        dist_cm_list, dist_px_list, angles_unsigned, angles_signed
      - angles_*[0] = 0 (no turn before first move)
    """
    if len(all_points) < 2:
        return [], [], [], []
    dist_cm_list, dist_px_list = [], []
    angles_unsigned, angles_signed = [0], [0]  # first turn = 0

    # distances
    for i in range(1, len(all_points)):
        dcm, dpx = dist_cm(all_points[i-1], all_points[i])
        dist_cm_list.append(dcm)
        dist_px_list.append(dpx)

    # angles at each intermediate vertex
    for i in range(1, len(all_points)-1):
        a_unsigned = get_angle_btw_line(all_points[i-1], all_points[i+1], all_points[i])
        a_signed   = get_signed_angle_btw_line(all_points[i-1], all_points[i+1], all_points[i])
        angles_unsigned.append(a_unsigned)
        angles_signed.append(a_signed)

    return dist_cm_list, dist_px_list, angles_unsigned, angles_signed

def save_json(path_points, meta):
    """
    path_points: list of absolute points in px; saved as 'pos' (px)
    meta: dict includes speed/height/mode
    """
    dcm, dpx, ang_u, ang_s = compute_segments(path_points)
    path_wp       = path_points
    path_dist_cm  = dcm
    path_dist_px  = dpx
    path_angle    = ang_s   # use ang_u if you want 0..180 instead

    # Print out the information.
    print('path_wp: {}'.format(path_wp))
    print('path_dist_cm: {}'.format(path_dist_cm))
    print('path_dist_px: {}'.format(path_dist_px))
    print('path_angle: {}'.format(path_angle))
    # (optional) also show unsigned for reference:
    # print('path_angle_unsigned: {}'.format(ang_u))
    # ---------------------------------------------


    waypoints = []
    for i in range(len(dcm)):
        waypoints.append({
            "dist_cm": int(dcm[i]),
            "dist_px": int(dpx[i]),
            "angle_deg": int(ang_u[i]) if i < len(ang_u) else 0,           # 0..180
            "turn_signed_deg": int(ang_s[i]) if i < len(ang_s) else 0       # -180..180 (left +)
        })
    data = {"wp": waypoints, "pos": path_points, "meta": meta}
    with open("waypoint.json", "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)
    print("[Saved] waypoint.json with", len(waypoints), "segments")

# ---------- Main loop ----------
MODE_FREE = 0
MODE_GRID = 1
mode = MODE_FREE

points = []
redo_stack = []
grid_path = []
active_popup = None
info_msg = ""
blink_fx = []

running = True
while running:
    # draw map
    draw_grid(screen)

    # current path
    if mode == MODE_FREE:
        path = [CENTER] + points
    else:
        path = grid_path[:] if grid_path else [CENTER]

    # path lines & points
    if len(path) >= 2:
        pygame.draw.lines(screen, PATH_COLOR, False, path, 2)
    for p in path:
        pygame.draw.circle(screen, POINT_COLOR, p, 3)

    # blink effects (expanding, fading ring)
    now = pygame.time.get_ticks()
    i = 0
    while i < len(blink_fx):
        fx = blink_fx[i]
        dt = now - fx["t0"]
        if dt >= BLINK_MS:
            blink_fx.pop(i)
            continue
        prog = dt / BLINK_MS
        radius = int(4 + 18*prog)
        alpha  = max(0, int(220 * (1 - prog)))
        ring_surf = pygame.Surface((radius*2+4, radius*2+4), pygame.SRCALPHA)
        pygame.draw.circle(ring_surf, (40,110,255, alpha), (radius+2, radius+2), radius, width=3)
        screen.blit(ring_surf, (fx["pos"][0]-radius-2, fx["pos"][1]-radius-2))
        i += 1

    # left info
    mode_str = "FREE" if mode == MODE_FREE else "GRID"
    pts_count = (len(points) if mode == MODE_FREE
                 else (len(grid_path) - 1 if grid_path else 0))
    info = f"Mode: {mode_str} | Points: {pts_count} | Speed={FLIGHT_SPEED}cm/s  Height={FLIGHT_HEIGHT}cm"
    screen.blit(font.render(info, True, TEXT_COLOR), (10, 10))
    if info_msg:
        screen.blit(font.render(info_msg, True, (120,0,0)), (10, 34))

    # right panel
    pygame.draw.rect(screen, (250,250,250), (W-PANEL_W, 0, PANEL_W, H))
    pygame.draw.line(screen, (200,200,200), (W-PANEL_W, 0), (W-PANEL_W, H), 1)
    buttons[0].text = "Mode: Free" if mode == MODE_FREE else "Mode: Grid"
    for b in buttons:
        b.draw(screen)

    # popup
    if active_popup:
        active_popup.draw(screen)

    pygame.display.flip()

    # events
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False

        elif active_popup:
            active_popup.handle_event(e)
            if active_popup.done:
                if active_popup.ok:
                    if active_popup.title.startswith("Settings"):
                        try:
                            FLIGHT_SPEED  = int(float(active_popup.fields[0][1]))
                            FLIGHT_HEIGHT = int(float(active_popup.fields[1][1]))
                            info_msg = "Settings updated."
                        except Exception:
                            info_msg = "Invalid settings input."
                    else:
                        # Grid params
                        try:
                            GW = float(active_popup.fields[0][1])
                            GH = float(active_popup.fields[1][1])
                            GC = float(active_popup.fields[2][1])
                            if GW <= 0 or GH <= 0 or GC <= 0:
                                raise ValueError("nonpositive")
                            GRID_W_M, GRID_H_M, GRID_CELL_M = GW, GH, GC
                            grid_path = gen_lawnmower(CENTER, GRID_W_M, GRID_H_M, GRID_CELL_M)
                            mode = MODE_GRID
                            info_msg = f"Generated grid {GRID_W_M}x{GRID_H_M}m cell {GRID_CELL_M}m."
                            add_blink(CENTER)
                        except Exception:
                            info_msg = "Invalid grid input."
                active_popup = None

        else:
            if e.type == pygame.MOUSEBUTTONDOWN:
                mx, my = e.pos
                # clicks inside map area only
                if mx < W-PANEL_W:
                    if mode == MODE_FREE:
                        pt = (mx, my)
                        points.append(pt)
                        redo_stack.clear()
                        add_blink(pt)
                # buttons
                for i, b in enumerate(buttons):
                    if b.hit((mx,my)):
                        if i == 0:  # Mode toggle
                            mode = MODE_FREE if mode == MODE_GRID else MODE_GRID
                            info_msg = f"Mode set to {'FREE' if mode==MODE_FREE else 'GRID'}."
                        elif i == 1:  # Gen Grid
                            active_popup = PopupInput(
                                "Grid Params (meters)",
                                [("Width (m)", f"{GRID_W_M}"),
                                 ("Height (m)", f"{GRID_H_M}"),
                                 ("Cell (m)", f"{GRID_CELL_M}")]
                            )
                        elif i == 2:  # Undo
                            if mode == MODE_FREE and points:
                                redo_stack.append(points.pop())
                        elif i == 3:  # Redo
                            if mode == MODE_FREE and redo_stack:
                                points.append(redo_stack.pop())
                        elif i == 4:  # Clear
                            if mode == MODE_FREE:
                                points.clear(); redo_stack.clear()
                            else:
                                grid_path.clear()
                        elif i == 5:  # Settings
                            active_popup = PopupInput(
                                "Settings (flight)",
                                [("Speed (cm/s)", f"{FLIGHT_SPEED}"),
                                 ("Height (cm)", f"{FLIGHT_HEIGHT}")]
                            )
                        elif i == 6:  # Save JSON
                            if mode == MODE_FREE:
                                path_points = [CENTER] + points
                            else:
                                path_points = grid_path[:] if grid_path else [CENTER]
                            meta = {
                                "mode": "FREE" if mode==MODE_FREE else "GRID",
                                "speed_cm_s": FLIGHT_SPEED,
                                "height_cm": FLIGHT_HEIGHT,
                                "cm_per_px": CM_PER_PX,
                                "center_px": CENTER
                            }
                            save_json(path_points, meta)
                        elif i == 7:  # Back to Base
                            back_to_base()

            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_z:  # undo
                    if mode == MODE_FREE and points:
                        redo_stack.append(points.pop())
                elif e.key == pygame.K_y:  # redo
                    if mode == MODE_FREE and redo_stack:
                        points.append(redo_stack.pop())
                elif e.key == pygame.K_h:  # back to base
                    back_to_base()
    
    clock.tick(FPS)

pygame.quit()

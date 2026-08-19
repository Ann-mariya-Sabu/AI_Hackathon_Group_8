"""
Emergency Response Ambulance -- Interactive Dynamic A* Replanning
Track 4: Emergency Response Ambulance

An ambulance plans a route to the hospital with A*, starts driving,
hits a traffic blockage, detects it, replans with A* from its current
position, and finishes the trip -- all in one continuous run.

This version adds an interactive dark-themed UI:
  - Edit the grid before launch: draw/erase obstacles, drag the start
    and hospital anywhere, randomize the map, see a live A* preview
    (path + cost + nodes) as you edit so you can hand-craft a more
    optimal layout.
  - While the ambulance drives, click the grid to drop your own
    traffic blockages anywhere (or leave "auto blockage" on for a
    hands-off demo), pause/resume, and restart at any time.

Run with:
    python main.py
"""

import heapq
import random
import time
import sys
import pygame  # type: ignore[import-not-found]

# ----------------------------------------------------------------------
# 1. CONFIGURATION
# ----------------------------------------------------------------------

GRID_COLS = 20
GRID_ROWS = 14
CELL = 40

GRID_W = GRID_COLS * CELL
GRID_H = GRID_ROWS * CELL
PANEL_W = 380
WINDOW_W = GRID_W + PANEL_W
WINDOW_H = GRID_H

DEFAULT_START = (7, 1)      # (row, col)
DEFAULT_HOSPITAL = (7, 18)

PATH_PREVIEW_MS = 900    # show the initial plan before the ambulance moves
MOVE_INTERVAL_MS = 220   # ms per grid cell while moving
SPAWN_AFTER_STEPS = 5    # auto-spawn the blockage after this many moves
BLOCKAGE_PAUSE_MS = 600  # visible "stop and think" pause while replanning

# Columns/rows that make up a narrow static-obstacle corridor near the
# middle of the map. Auto-spawn never drops a blockage in this zone so
# the ambulance is never sealed off with no possible detour.
NARROW_ZONE = lambda r, c: 9 <= c <= 11 and 5 <= r <= 9

# ----------------------------------------------------------------------
# DARK THEME PALETTE
# ----------------------------------------------------------------------

COL_BG = (16, 18, 24)
COL_GRID_LINE = (36, 40, 50)
COL_GRID_LINE_SOFT = (26, 29, 37)
COL_STATIC = (64, 70, 86)
COL_STATIC_BORDER = (90, 98, 118)
COL_INITIAL_PATH = (70, 140, 255)
COL_NEW_PATH = (255, 150, 55)
COL_PREVIEW_PATH = (110, 220, 180)
COL_TRAVELLED = (70, 210, 140)
COL_BLOCKAGE = (230, 70, 70)
COL_BLOCKAGE_DARK = (110, 20, 20)
COL_HOVER = (255, 255, 255)

COL_PANEL_BG = (12, 13, 18)
COL_PANEL_BG_ALT = (18, 20, 27)
COL_PANEL_TEXT = (230, 232, 238)
COL_PANEL_DIM = (135, 140, 155)
COL_WARN = (255, 95, 95)
COL_SUCCESS = (95, 230, 150)
COL_ACCENT = (95, 170, 255)

COL_BTN = (30, 33, 42)
COL_BTN_HOVER = (44, 48, 62)
COL_BTN_ACTIVE = (60, 110, 200)
COL_BTN_BORDER = (55, 60, 74)
COL_BTN_TEXT = (225, 228, 235)


# ----------------------------------------------------------------------
# 2. A* SEARCH
# ----------------------------------------------------------------------

def heuristic(a, b):
    """Manhattan distance: h(n) = |x1-x2| + |y1-y2|."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(cell, obstacles):
    r, c = cell
    result = []
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nr, nc = r + dr, c + dc
        if 0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS and (nr, nc) not in obstacles:
            result.append((nr, nc))
    return result


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def a_star(start, goal, obstacles):
    """Real A* search. Returns (path, nodes_expanded, path_cost).

    path_cost is the number of moves (g-score of the goal).
    Returns (None, nodes_expanded, None) if no path exists.
    """
    if start == goal:
        return [start], 0, 0

    open_set = [(heuristic(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}
    closed = set()
    nodes_expanded = 0

    while open_set:
        f, g, current = heapq.heappop(open_set)
        if current in closed:
            continue
        closed.add(current)
        nodes_expanded += 1

        if current == goal:
            return reconstruct_path(came_from, current), nodes_expanded, g

        for neighbor in get_neighbors(current, obstacles):
            tentative_g = g + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))

    return None, nodes_expanded, None


# ----------------------------------------------------------------------
# 3. STATIC MAP
# ----------------------------------------------------------------------

def make_static_obstacles():
    obstacles = set()

    # A small wall near the middle that forces the initial route to jog
    # up or down instead of driving straight across.
    for r in (6, 7, 8):
        obstacles.add((r, 10))

    # Decorative city blocks (kept away from the main corridor so the
    # grid always has an open detour available).
    decorative = [
        (2, 2), (2, 3), (3, 2), (3, 3),
        (11, 3), (11, 4), (12, 3),
        (4, 7), (4, 8), (5, 7),
        (9, 14), (9, 15), (10, 14),
        (1, 16), (1, 17), (2, 16),
        (12, 17), (12, 18), (11, 18),
    ]
    for cell in decorative:
        obstacles.add(cell)

    return obstacles


def random_obstacles(start, hospital, density=0.16):
    """Generate a random obstacle field that still leaves a path open."""
    for _ in range(200):
        obstacles = set()
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                cell = (r, c)
                if cell in (start, hospital):
                    continue
                if random.random() < density:
                    obstacles.add(cell)
        path, _, _ = a_star(start, hospital, obstacles)
        if path:
            return obstacles
    return set()


# ----------------------------------------------------------------------
# 4. SIMULATION STATE
# ----------------------------------------------------------------------

def log(sim, msg):
    print(msg)
    sim["messages"].append(msg)
    sim["messages"] = sim["messages"][-14:]


def recompute_preview(sim):
    """Live A* preview used while editing the map (before launch)."""
    path, nodes, cost = a_star(sim["start"], sim["hospital"], sim["obstacles"])
    sim["preview_path"] = path
    sim["preview_cost"] = cost
    sim["preview_nodes"] = nodes


def new_sim(start=None, hospital=None, obstacles=None):
    now = pygame.time.get_ticks()
    start = start or DEFAULT_START
    hospital = hospital or DEFAULT_HOSPITAL
    obstacles = obstacles if obstacles is not None else make_static_obstacles()
    obstacles.discard(start)
    obstacles.discard(hospital)

    sim = {
        "phase": "EDIT",              # "EDIT" -> "SIM"
        "edit_mode": "select",        # select | set_start | set_hospital
        "sim_mode": "none",           # none | add_blockage
        "auto_blockage": True,

        "start": start,
        "hospital": hospital,
        "obstacles": obstacles,
        "dynamic_obstacles": set(),

        "initial_path": None,
        "current_path": None,
        "position": start,
        "path_index": 0,
        "travelled": [start],
        "steps_taken": 0,
        "replans": 0,
        "nodes_expanded_total": 0,
        "initial_cost": None,
        "final_cost": None,

        "state": "EDIT",
        "next_move_time": now,
        "blockage_spawned_auto": False,
        "start_time": None,
        "exec_time": None,
        "messages": [],
        "paused": False,

        "preview_path": None,
        "preview_cost": None,
        "preview_nodes": None,
    }

    log(sim, "[INIT] Emergency Ambulance Agent ready")
    log(sim, "[EDIT] Edit the map, then press START SIMULATION")
    recompute_preview(sim)
    return sim


def launch_sim(sim):
    """Move from EDIT phase into a running simulation."""
    sim["phase"] = "SIM"
    sim["state"] = "INIT_DELAY"
    sim["position"] = sim["start"]
    sim["path_index"] = 0
    sim["travelled"] = [sim["start"]]
    sim["steps_taken"] = 0
    sim["replans"] = 0
    sim["dynamic_obstacles"] = set()
    sim["blockage_spawned_auto"] = False
    sim["start_time"] = time.time()
    sim["exec_time"] = None
    sim["next_move_time"] = pygame.time.get_ticks() + PATH_PREVIEW_MS

    log(sim, "[A*] Searching for optimal path...")
    path, nodes, cost = a_star(sim["start"], sim["hospital"], sim["obstacles"])
    sim["initial_path"] = path
    sim["current_path"] = path
    sim["initial_cost"] = cost
    sim["final_cost"] = cost
    sim["nodes_expanded_total"] = nodes

    if path:
        log(sim, "[A*] Initial path found")
        log(sim, f"[A*] Path cost: {cost}")
        log(sim, f"[A*] Nodes expanded: {nodes}")
    else:
        log(sim, "[A*] No route available from start to hospital!")


def all_obstacles(sim):
    return sim["obstacles"] | sim["dynamic_obstacles"]


def add_blockage(sim, cell, manual=True):
    if cell in (sim["position"], sim["hospital"]):
        return
    if cell in sim["obstacles"] or cell in sim["dynamic_obstacles"]:
        return

    sim["dynamic_obstacles"].add(cell)
    tag = "manually placed" if manual else "detected"
    log(sim, "[EVENT] Traffic blockage " + tag + "!")
    log(sim, f"[EVENT] Blocked cell: {cell}")

    if sim["state"] == "MOVING" and cell in sim["current_path"][sim["path_index"]:]:
        log(sim, "[AGENT] Current route is invalid")
        replan(sim)


def auto_spawn_blockage(sim):
    remaining = sim["current_path"][sim["path_index"]:]
    if len(remaining) < 4:
        sim["blockage_spawned_auto"] = True
        return

    lo = max(1, int(len(remaining) * 0.30))
    hi = min(len(remaining) - 1, int(len(remaining) * 0.50))
    if hi < lo:
        hi = lo

    def valid(cell):
        r, c = cell
        if cell in (sim["start"], sim["hospital"], sim["position"]):
            return False
        if cell in sim["obstacles"] or cell in sim["dynamic_obstacles"]:
            return False
        if NARROW_ZONE(r, c):
            return False
        return True

    candidates = [remaining[i] for i in range(lo, hi + 1) if valid(remaining[i])]
    if not candidates:
        candidates = [cell for cell in remaining[1:] if valid(cell)]
    if not candidates:
        sim["blockage_spawned_auto"] = True
        return

    blockage = candidates[len(candidates) // 2]
    sim["blockage_spawned_auto"] = True
    add_blockage(sim, blockage, manual=False)


def replan(sim):
    log(sim, "[A*] Dynamic replanning started...")
    path, nodes, cost = a_star(sim["position"], sim["hospital"], all_obstacles(sim))
    sim["nodes_expanded_total"] += nodes

    if path:
        sim["current_path"] = path
        sim["path_index"] = 0
        sim["replans"] += 1
        sim["final_cost"] = cost

        log(sim, "[A*] New path found")
        log(sim, f"[A*] New path cost: {cost}")
        log(sim, f"[A*] Nodes expanded: {nodes}")
        log(sim, "[AGENT] Resuming movement using replanned route...")

        sim["state"] = "BLOCKED_PAUSE"
        sim["next_move_time"] = pygame.time.get_ticks() + BLOCKAGE_PAUSE_MS
    else:
        log(sim, "[A*] No route available. Ambulance stuck.")
        sim["state"] = "STUCK"


def finish(sim):
    sim["state"] = "ARRIVED"
    sim["exec_time"] = time.time() - sim["start_time"]

    log(sim, "[GOAL] Hospital reached!")
    log(sim, "[RESULT] Simulation completed successfully")
    log(sim, f"[RESULT] Replans: {sim['replans']}")
    log(sim, f"[RESULT] Final path cost: {sim['final_cost']}")
    log(sim, f"[RESULT] Total nodes expanded: {sim['nodes_expanded_total']}")
    log(sim, f"[RESULT] Execution time: {sim['exec_time']:.2f} seconds")


def update(sim):
    if sim["phase"] != "SIM":
        return
    if sim["paused"] or sim["state"] in ("ARRIVED", "STUCK"):
        return

    now = pygame.time.get_ticks()

    if sim["state"] == "INIT_DELAY":
        if now >= sim["next_move_time"]:
            sim["state"] = "MOVING"
            sim["next_move_time"] = now + MOVE_INTERVAL_MS
            log(sim, "[AGENT] Ambulance moving...")
        return

    if sim["state"] == "BLOCKED_PAUSE":
        if now >= sim["next_move_time"]:
            sim["state"] = "MOVING"
            sim["next_move_time"] = now + MOVE_INTERVAL_MS
        return

    if sim["state"] == "MOVING":
        if now >= sim["next_move_time"]:
            path = sim["current_path"]
            if sim["path_index"] < len(path) - 1:
                sim["path_index"] += 1
                sim["position"] = path[sim["path_index"]]
                sim["travelled"].append(sim["position"])
                sim["steps_taken"] += 1
                log(sim, f"[AGENT] Current position: {sim['position']}")
            sim["next_move_time"] = now + MOVE_INTERVAL_MS

        if (sim["auto_blockage"] and not sim["blockage_spawned_auto"]
                and sim["steps_taken"] >= SPAWN_AFTER_STEPS):
            auto_spawn_blockage(sim)

        if sim["position"] == sim["hospital"]:
            finish(sim)


# ----------------------------------------------------------------------
# 5. DRAWING -- GRID
# ----------------------------------------------------------------------

def cell_rect(cell, pad=0):
    r, c = cell
    return pygame.Rect(c * CELL + pad, r * CELL + pad, CELL - 2 * pad, CELL - 2 * pad)


def draw_soft_cell(surface, cell, color, alpha=180, size_ratio=0.55):
    r, c = cell
    pad = int(CELL * (1 - size_ratio) / 2)
    s = pygame.Surface((CELL, CELL), pygame.SRCALPHA)
    pygame.draw.rect(s, (*color, alpha), (pad, pad, CELL - 2 * pad, CELL - 2 * pad), border_radius=5)
    surface.blit(s, (c * CELL, r * CELL))


def draw_grid(surface):
    surface.fill(COL_BG, pygame.Rect(0, 0, GRID_W, GRID_H))
    for c in range(GRID_COLS + 1):
        x = c * CELL
        pygame.draw.line(surface, COL_GRID_LINE_SOFT, (x, 0), (x, GRID_H))
    for r in range(GRID_ROWS + 1):
        y = r * CELL
        pygame.draw.line(surface, COL_GRID_LINE_SOFT, (0, y), (GRID_W, y))


def draw_static_obstacles(surface, sim):
    for cell in sim["obstacles"]:
        rect = cell_rect(cell, pad=2)
        pygame.draw.rect(surface, COL_STATIC, rect, border_radius=4)
        pygame.draw.rect(surface, COL_STATIC_BORDER, rect, width=1, border_radius=4)


def draw_ambulance(surface, cell):
    rect = cell_rect(cell)
    body = pygame.Rect(rect.x + 6, rect.y + 10, rect.w - 12, rect.h - 18)
    pygame.draw.rect(surface, (235, 238, 244), body, border_radius=6)
    pygame.draw.rect(surface, (30, 32, 40), body, width=2, border_radius=6)

    cx, cy = body.center
    pygame.draw.rect(surface, (225, 45, 45), (cx - 8, cy - 3, 16, 6))
    pygame.draw.rect(surface, (225, 45, 45), (cx - 3, cy - 8, 6, 16))

    pygame.draw.circle(surface, (225, 45, 45), (rect.x + 10, rect.y + 8), 3)
    pygame.draw.circle(surface, (70, 120, 230), (rect.x + rect.w - 10, rect.y + 8), 3)


def draw_hospital(surface, cell):
    rect = cell_rect(cell, pad=4)
    pygame.draw.rect(surface, (235, 238, 244), rect, border_radius=4)
    pygame.draw.rect(surface, (60, 140, 230), rect, width=2, border_radius=4)
    cx, cy = rect.center
    pygame.draw.rect(surface, (60, 140, 230), (cx - 2, cy - 10, 4, 20))
    pygame.draw.rect(surface, (60, 140, 230), (cx - 10, cy - 2, 20, 4))


def draw_start_marker(surface, cell):
    rect = cell_rect(cell, pad=6)
    pygame.draw.rect(surface, (70, 210, 140), rect, width=2, border_radius=8)


def draw_blockage(surface, cell):
    rect = cell_rect(cell, pad=2)
    pygame.draw.rect(surface, COL_BLOCKAGE, rect, border_radius=3)
    pygame.draw.line(surface, (255, 255, 255), rect.topleft, rect.bottomright, 4)
    pygame.draw.line(surface, (255, 255, 255), rect.topright, rect.bottomleft, 4)
    pygame.draw.rect(surface, COL_BLOCKAGE_DARK, rect, width=2, border_radius=3)


def draw_hover_highlight(surface, cell, color=COL_HOVER):
    rect = cell_rect(cell, pad=1)
    pygame.draw.rect(surface, color, rect, width=2, border_radius=4)


def draw_scene(surface, sim, hover_cell):
    draw_grid(surface)
    draw_static_obstacles(surface, sim)

    if sim["phase"] == "EDIT":
        if sim["preview_path"]:
            for cell in sim["preview_path"]:
                draw_soft_cell(surface, cell, COL_PREVIEW_PATH, alpha=140, size_ratio=0.4)
        draw_start_marker(surface, sim["start"])
        draw_hospital(surface, sim["hospital"])
        if hover_cell is not None:
            highlight_color = COL_HOVER
            if sim["edit_mode"] == "set_start":
                highlight_color = (70, 210, 140)
            elif sim["edit_mode"] == "set_hospital":
                highlight_color = (60, 140, 230)
            draw_hover_highlight(surface, hover_cell, highlight_color)
    else:
        if sim["initial_path"]:
            for cell in sim["initial_path"]:
                draw_soft_cell(surface, cell, COL_INITIAL_PATH, alpha=130, size_ratio=0.42)

        if sim["replans"] > 0 and sim["current_path"]:
            for cell in sim["current_path"][sim["path_index"]:]:
                draw_soft_cell(surface, cell, COL_NEW_PATH, alpha=190, size_ratio=0.5)

        for cell in sim["travelled"]:
            draw_soft_cell(surface, cell, COL_TRAVELLED, alpha=210, size_ratio=0.34)

        for cell in sim["dynamic_obstacles"]:
            draw_blockage(surface, cell)

        draw_hospital(surface, sim["hospital"])
        draw_ambulance(surface, sim["position"])

        if sim["sim_mode"] == "add_blockage" and hover_cell is not None:
            draw_hover_highlight(surface, hover_cell, COL_WARN)


# ----------------------------------------------------------------------
# 6. BUTTONS
# ----------------------------------------------------------------------

class Button:
    def __init__(self, rect, label, action, active=False, disabled=False):
        self.rect = rect
        self.label = label
        self.action = action
        self.active = active
        self.disabled = disabled

    def draw(self, surface, font, hovered):
        if self.disabled:
            bg = COL_PANEL_BG_ALT
            fg = COL_PANEL_DIM
        elif self.active:
            bg = COL_BTN_ACTIVE
            fg = (255, 255, 255)
        elif hovered:
            bg = COL_BTN_HOVER
            fg = COL_BTN_TEXT
        else:
            bg = COL_BTN
            fg = COL_BTN_TEXT

        pygame.draw.rect(surface, bg, self.rect, border_radius=6)
        pygame.draw.rect(surface, COL_BTN_BORDER, self.rect, width=1, border_radius=6)
        txt = font.render(self.label, True, fg)
        tx = self.rect.x + (self.rect.w - txt.get_width()) // 2
        ty = self.rect.y + (self.rect.h - txt.get_height()) // 2
        surface.blit(txt, (tx, ty))


def build_buttons(sim):
    x = GRID_W + 20
    w = PANEL_W - 40
    half = (w - 10) // 2
    buttons = []

    if sim["phase"] == "EDIT":
        y = 132
        buttons.append(Button(pygame.Rect(x, y, w, 34), "\u25b6  START SIMULATION", "start_sim"))
        y += 44
        buttons.append(Button(pygame.Rect(x, y, half, 30), "SET START",
                               "set_start", active=(sim["edit_mode"] == "set_start")))
        buttons.append(Button(pygame.Rect(x + half + 10, y, half, 30), "SET HOSPITAL",
                               "set_hospital", active=(sim["edit_mode"] == "set_hospital")))
        y += 40
        buttons.append(Button(pygame.Rect(x, y, half, 30), "RANDOM MAP", "random_obs"))
        buttons.append(Button(pygame.Rect(x + half + 10, y, half, 30), "CLEAR OBSTACLES", "clear_obs"))
        y += 40
        buttons.append(Button(pygame.Rect(x, y, w, 30), "RESET GRID", "reset_edit"))
    else:
        y = 132
        pause_label = "\u25b6  RESUME" if sim["paused"] else "\u23f8  PAUSE"
        buttons.append(Button(pygame.Rect(x, y, half, 32), pause_label, "pause"))
        buttons.append(Button(pygame.Rect(x + half + 10, y, half, 32), "\u27f2 RESTART", "restart"))
        y += 42
        buttons.append(Button(pygame.Rect(x, y, w, 30), "\U0001f6a7 CLICK GRID TO ADD BLOCKAGE",
                               "add_blockage_mode", active=(sim["sim_mode"] == "add_blockage")))
        y += 40
        auto_label = "AUTO BLOCKAGE: ON" if sim["auto_blockage"] else "AUTO BLOCKAGE: OFF"
        buttons.append(Button(pygame.Rect(x, y, w, 30), auto_label, "toggle_auto",
                               active=sim["auto_blockage"]))
        y += 40
        buttons.append(Button(pygame.Rect(x, y, w, 30), "\u2b05 BACK TO MAP EDITOR", "back_to_edit"))

    return buttons


def handle_button(sim, action):
    if action == "start_sim":
        if sim["preview_path"] is not None:
            launch_sim(sim)
        else:
            log(sim, "[EDIT] No route exists between start and hospital!")
    elif action == "set_start":
        sim["edit_mode"] = "none" if sim["edit_mode"] == "set_start" else "set_start"
    elif action == "set_hospital":
        sim["edit_mode"] = "none" if sim["edit_mode"] == "set_hospital" else "set_hospital"
    elif action == "random_obs":
        sim["obstacles"] = random_obstacles(sim["start"], sim["hospital"])
        recompute_preview(sim)
        log(sim, "[EDIT] Random obstacle field generated")
    elif action == "clear_obs":
        sim["obstacles"] = set()
        recompute_preview(sim)
        log(sim, "[EDIT] Obstacles cleared")
    elif action == "reset_edit":
        sim["start"] = DEFAULT_START
        sim["hospital"] = DEFAULT_HOSPITAL
        sim["obstacles"] = make_static_obstacles()
        recompute_preview(sim)
        log(sim, "[EDIT] Grid reset to default layout")
    elif action == "pause":
        sim["paused"] = not sim["paused"]
    elif action == "restart":
        launch_sim(sim)
    elif action == "add_blockage_mode":
        sim["sim_mode"] = "none" if sim["sim_mode"] == "add_blockage" else "add_blockage"
    elif action == "toggle_auto":
        sim["auto_blockage"] = not sim["auto_blockage"]
    elif action == "back_to_edit":
        sim["phase"] = "EDIT"
        sim["state"] = "EDIT"
        sim["edit_mode"] = "select"
        sim["sim_mode"] = "none"
        recompute_preview(sim)


# ----------------------------------------------------------------------
# 7. INFO PANEL
# ----------------------------------------------------------------------

def status_text(sim):
    if sim["phase"] == "EDIT":
        return "EDITING MAP", COL_ACCENT
    if sim["state"] == "INIT_DELAY":
        return "PLANNING", COL_ACCENT
    if sim["state"] == "BLOCKED_PAUSE":
        return "REPLANNING", COL_WARN
    if sim["state"] == "ARRIVED":
        return "ARRIVED", COL_SUCCESS
    if sim["state"] == "STUCK":
        return "NO ROUTE", COL_WARN
    if sim["paused"]:
        return "PAUSED", COL_PANEL_DIM
    return "MOVING", COL_ACCENT


def draw_panel(surface, sim, fonts, buttons, hovered_btn):
    f_title, f_label, f_bold, f_mono, f_banner = fonts

    panel = pygame.Rect(GRID_W, 0, PANEL_W, WINDOW_H)
    surface.fill(COL_PANEL_BG, panel)

    x = GRID_W + 20
    y = 18

    surface.blit(f_title.render("EMERGENCY RESPONSE SYSTEM", True, COL_PANEL_TEXT), (x, y))
    y += 26
    surface.blit(f_label.render("A* pathfinding \u00b7 dynamic replanning", True, COL_PANEL_DIM), (x, y))
    y += 26

    status, status_color = status_text(sim)
    surface.blit(f_label.render("Status:", True, COL_PANEL_DIM), (x, y))
    surface.blit(f_bold.render(status, True, status_color), (x + 90, y))
    y += 24

    # --- buttons ---
    for b in buttons:
        b.draw(surface, f_bold, hovered_btn is b)
    y = buttons[-1].rect.bottom + 18

    pygame.draw.line(surface, COL_BTN_BORDER, (x, y), (x + PANEL_W - 40, y))
    y += 14

    if sim["phase"] == "EDIT":
        surface.blit(f_label.render("Left-click a cell to toggle an obstacle.", True, COL_PANEL_DIM), (x, y))
        y += 18
        surface.blit(f_label.render("SET START / SET HOSPITAL then click a cell.", True, COL_PANEL_DIM), (x, y))
        y += 24

        stats = [
            ("Start", str(sim["start"])),
            ("Hospital", str(sim["hospital"])),
            ("Preview Cost", str(sim["preview_cost"]) if sim["preview_cost"] is not None else "NO PATH"),
            ("Preview Nodes", str(sim["preview_nodes"])),
        ]
        for label, value in stats:
            surface.blit(f_label.render(f"{label}:", True, COL_PANEL_DIM), (x, y))
            surface.blit(f_bold.render(value, True, COL_PANEL_TEXT), (x + 150, y))
            y += 22
    else:
        stats = [
            ("Position", str(sim["position"])),
            ("Path Cost", str(sim["final_cost"])),
            ("Nodes Expanded", str(sim["nodes_expanded_total"])),
            ("Replans", str(sim["replans"])),
            ("Steps Taken", str(sim["steps_taken"])),
        ]
        for label, value in stats:
            surface.blit(f_label.render(f"{label}:", True, COL_PANEL_DIM), (x, y))
            surface.blit(f_bold.render(value, True, COL_PANEL_TEXT), (x + 150, y))
            y += 22

    y += 6

    banner_bg = banner_lines = None
    if sim["phase"] == "SIM":
        if sim["state"] == "ARRIVED":
            banner_bg, banner_lines = COL_SUCCESS, ["EMERGENCY RESOLVED", "HOSPITAL REACHED"]
        elif sim["state"] == "STUCK":
            banner_bg, banner_lines = COL_WARN, ["NO ROUTE AVAILABLE", "AMBULANCE BLOCKED IN"]
        elif sim["state"] == "BLOCKED_PAUSE":
            banner_bg, banner_lines = COL_WARN, ["TRAFFIC BLOCKAGE!", "REPLANNING..."]
        elif sim["replans"] > 0:
            banner_bg, banner_lines = COL_SUCCESS, ["NEW ROUTE FOUND", "CONTINUING TO HOSPITAL"]

    if banner_lines:
        banner_rect = pygame.Rect(x, y, PANEL_W - 40, 46)
        pygame.draw.rect(surface, banner_bg, banner_rect, border_radius=6)
        ty = y + 6
        for line in banner_lines:
            txt = f_banner.render(line, True, (20, 20, 20))
            surface.blit(txt, (banner_rect.x + 10, ty))
            ty += 18
        y += 56
    else:
        y += 8

    if sim["phase"] == "SIM" and sim["state"] == "ARRIVED":
        pygame.draw.line(surface, COL_BTN_BORDER, (x, y), (x + PANEL_W - 40, y))
        y += 14
        surface.blit(f_bold.render("MISSION COMPLETE", True, COL_SUCCESS), (x, y))
        y += 24
        metrics = [
            ("Initial Path Cost", sim["initial_cost"]),
            ("Final Path Cost", sim["final_cost"]),
            ("Replans", sim["replans"]),
            ("Nodes Expanded", sim["nodes_expanded_total"]),
            ("Steps Taken", sim["steps_taken"]),
            ("Execution Time", f"{sim['exec_time']:.2f} s"),
        ]
        for label, value in metrics:
            surface.blit(f_label.render(f"{label}:", True, COL_PANEL_DIM), (x, y))
            surface.blit(f_bold.render(str(value), True, COL_PANEL_TEXT), (x + 160, y))
            y += 20
        y += 8

    pygame.draw.line(surface, COL_BTN_BORDER, (x, y), (x + PANEL_W - 40, y))
    y += 10
    surface.blit(f_label.render("LIVE LOG", True, COL_PANEL_DIM), (x, y))
    y += 18

    max_y = WINDOW_H - 34
    for msg in sim["messages"][-9:]:
        if y > max_y:
            break
        clipped = msg if len(msg) <= 48 else msg[:45] + "..."
        surface.blit(f_mono.render(clipped, True, (150, 210, 190)), (x, y))
        y += 15

    hint = "SPACE pause   R restart   ESC quit"
    surface.blit(f_mono.render(hint, True, COL_PANEL_DIM), (x, WINDOW_H - 20))


# ----------------------------------------------------------------------
# 8. MAIN LOOP
# ----------------------------------------------------------------------

def run_simulation():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Emergency Response Ambulance - Dynamic A* Replanning")
    clock = pygame.time.Clock()

    f_title = pygame.font.SysFont("segoeui,arial", 18, bold=True)
    f_label = pygame.font.SysFont("segoeui,arial", 14)
    f_bold = pygame.font.SysFont("segoeui,arial", 14, bold=True)
    f_banner = pygame.font.SysFont("segoeui,arial", 14, bold=True)
    f_mono = pygame.font.SysFont("consolas,couriernew,monospace", 12)
    fonts = (f_title, f_label, f_bold, f_mono, f_banner)

    sim = new_sim()
    buttons = build_buttons(sim)
    running = True

    while running:
        clock.tick(60)
        mouse_pos = pygame.mouse.get_pos()
        hovered_btn = None
        for b in buttons:
            if b.rect.collidepoint(mouse_pos) and not b.disabled:
                hovered_btn = b
                break

        hover_cell = None
        if mouse_pos[0] < GRID_W:
            col = mouse_pos[0] // CELL
            row = mouse_pos[1] // CELL
            if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
                hover_cell = (row, col)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE and sim["phase"] == "SIM":
                    sim["paused"] = not sim["paused"]
                elif event.key == pygame.K_r:
                    sim = new_sim()
                    buttons = build_buttons(sim)

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                clicked_button = False
                for b in buttons:
                    if b.rect.collidepoint((mx, my)) and not b.disabled:
                        handle_button(sim, b.action)
                        clicked_button = True
                        break

                if not clicked_button and mx < GRID_W:
                    col = mx // CELL
                    row = my // CELL
                    if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
                        cell = (row, col)
                        if sim["phase"] == "EDIT":
                            if sim["edit_mode"] == "set_start":
                                if cell != sim["hospital"] and cell not in sim["obstacles"]:
                                    sim["start"] = cell
                                    sim["edit_mode"] = "select"
                                    recompute_preview(sim)
                            elif sim["edit_mode"] == "set_hospital":
                                if cell != sim["start"] and cell not in sim["obstacles"]:
                                    sim["hospital"] = cell
                                    sim["edit_mode"] = "select"
                                    recompute_preview(sim)
                            else:
                                if cell not in (sim["start"], sim["hospital"]):
                                    if cell in sim["obstacles"]:
                                        sim["obstacles"].discard(cell)
                                    else:
                                        sim["obstacles"].add(cell)
                                    recompute_preview(sim)
                        elif sim["phase"] == "SIM" and sim["sim_mode"] == "add_blockage":
                            if sim["state"] in ("MOVING", "BLOCKED_PAUSE"):
                                add_blockage(sim, cell, manual=True)

                buttons = build_buttons(sim)

        update(sim)
        buttons = build_buttons(sim)

        screen.fill(COL_PANEL_BG)
        draw_scene(screen, sim, hover_cell)
        draw_panel(screen, sim, fonts, buttons, hovered_btn)
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    run_simulation()
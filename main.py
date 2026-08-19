"""
Emergency Response Ambulance -- Dynamic A* Replanning
Track 4: Emergency Response Ambulance

An ambulance plans a route to the hospital with A*, starts driving,
hits a traffic blockage that suddenly appears on its route, detects it,
replans with A* from its current position, and finishes the trip --
all in one continuous run, no restart required.

Run with:
    python main.py
"""

import heapq
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
PANEL_W = 340
WINDOW_W = GRID_W + PANEL_W
WINDOW_H = GRID_H

START = (7, 1)      # (row, col)
HOSPITAL = (7, 18)

PATH_PREVIEW_MS = 1500   # show the initial plan before the ambulance moves
MOVE_INTERVAL_MS = 250   # ms per grid cell while moving
SPAWN_AFTER_STEPS = 5    # spawn the blockage after this many moves
BLOCKAGE_PAUSE_MS = 700  # visible "stop and think" pause while replanning

# Columns/rows that make up the narrow static-obstacle corridor near the
# middle of the map. We never drop the dynamic blockage in this zone so
# the ambulance is never sealed off with no possible detour.
NARROW_ZONE = lambda r, c: 9 <= c <= 11 and 5 <= r <= 9

# Colors
COL_BG = (235, 238, 242)
COL_GRID_LINE = (208, 212, 218)
COL_STATIC = (55, 60, 70)
COL_INITIAL_PATH = (140, 185, 255)
COL_NEW_PATH = (255, 165, 60)
COL_TRAVELLED = (95, 200, 130)
COL_BLOCKAGE = (210, 55, 55)
COL_BLOCKAGE_DARK = (120, 20, 20)
COL_PANEL_BG = (24, 27, 34)
COL_PANEL_TEXT = (225, 228, 234)
COL_PANEL_DIM = (150, 155, 165)
COL_WARN = (255, 90, 90)
COL_SUCCESS = (90, 225, 140)
COL_ACCENT = (90, 160, 255)


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

    obstacles.discard(START)
    obstacles.discard(HOSPITAL)
    return obstacles


# ----------------------------------------------------------------------
# 4. SIMULATION STATE
# ----------------------------------------------------------------------

def log(sim, msg):
    print(msg)
    sim["messages"].append(msg)
    sim["messages"] = sim["messages"][-12:]


def new_sim():
    obstacles = make_static_obstacles()
    now = pygame.time.get_ticks()

    sim = {
        "obstacles": obstacles,
        "dynamic_obstacle": None,
        "initial_path": None,
        "current_path": None,
        "position": START,
        "path_index": 0,
        "travelled": [START],
        "steps_taken": 0,
        "replans": 0,
        "nodes_expanded_total": 0,
        "initial_cost": None,
        "final_cost": None,
        "state": "INIT_DELAY",
        "next_move_time": now + PATH_PREVIEW_MS,
        "blockage_spawned": False,
        "start_time": time.time(),
        "exec_time": None,
        "messages": [],
        "paused": False,
    }

    log(sim, "[INIT] Emergency Ambulance Agent started")
    log(sim, "[GRID] Environment initialized")
    log(sim, "[A*] Searching for optimal path...")

    path, nodes, cost = a_star(START, HOSPITAL, obstacles)
    sim["initial_path"] = path
    sim["current_path"] = path
    sim["initial_cost"] = cost
    sim["final_cost"] = cost
    sim["nodes_expanded_total"] = nodes

    log(sim, "[A*] Initial path found")
    log(sim, f"[A*] Path cost: {cost}")
    log(sim, f"[A*] Nodes expanded: {nodes}")

    return sim


def spawn_blockage(sim):
    remaining = sim["current_path"][sim["path_index"]:]
    if len(remaining) < 4:
        # Too close to the hospital to safely place a blockage -- skip.
        sim["blockage_spawned"] = True
        return

    lo = max(1, int(len(remaining) * 0.30))
    hi = min(len(remaining) - 1, int(len(remaining) * 0.50))
    if hi < lo:
        hi = lo

    def valid(cell):
        r, c = cell
        if cell in (START, HOSPITAL):
            return False
        if cell in sim["obstacles"]:
            return False
        if NARROW_ZONE(r, c):
            return False
        return True

    candidates = [remaining[i] for i in range(lo, hi + 1) if valid(remaining[i])]
    if not candidates:
        candidates = [cell for cell in remaining[1:] if valid(cell)]
    if not candidates:
        sim["blockage_spawned"] = True
        return

    blockage = candidates[len(candidates) // 2]
    sim["dynamic_obstacle"] = blockage
    sim["blockage_spawned"] = True

    log(sim, "[EVENT] Traffic blockage detected!")
    log(sim, f"[EVENT] Blocked cell: {blockage}")

    if blockage in sim["current_path"][sim["path_index"]:]:
        log(sim, "[AGENT] Current route is invalid")
        replan(sim)


def replan(sim):
    log(sim, "[A*] Dynamic replanning started...")
    obstacles = sim["obstacles"] | {sim["dynamic_obstacle"]}
    new_path, nodes, cost = a_star(sim["position"], HOSPITAL, obstacles)
    sim["nodes_expanded_total"] += nodes

    if new_path:
        sim["current_path"] = new_path
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
        log(sim, "[A*] No route available.")


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
    if sim["paused"] or sim["state"] == "ARRIVED":
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

        if not sim["blockage_spawned"] and sim["steps_taken"] >= SPAWN_AFTER_STEPS:
            spawn_blockage(sim)

        if sim["position"] == HOSPITAL:
            finish(sim)


# ----------------------------------------------------------------------
# 5. DRAWING
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
        pygame.draw.line(surface, COL_GRID_LINE, (x, 0), (x, GRID_H))
    for r in range(GRID_ROWS + 1):
        y = r * CELL
        pygame.draw.line(surface, COL_GRID_LINE, (0, y), (GRID_W, y))


def draw_static_obstacles(surface, sim):
    for cell in sim["obstacles"]:
        pygame.draw.rect(surface, COL_STATIC, cell_rect(cell, pad=2), border_radius=4)


def draw_ambulance(surface, cell):
    rect = cell_rect(cell)
    body = pygame.Rect(rect.x + 6, rect.y + 10, rect.w - 12, rect.h - 18)
    pygame.draw.rect(surface, (255, 255, 255), body, border_radius=6)
    pygame.draw.rect(surface, (40, 40, 40), body, width=2, border_radius=6)

    cx, cy = body.center
    pygame.draw.rect(surface, (220, 40, 40), (cx - 8, cy - 3, 16, 6))
    pygame.draw.rect(surface, (220, 40, 40), (cx - 3, cy - 8, 6, 16))

    pygame.draw.circle(surface, (220, 40, 40), (rect.x + 10, rect.y + 8), 3)
    pygame.draw.circle(surface, (60, 100, 220), (rect.x + rect.w - 10, rect.y + 8), 3)


def draw_hospital(surface, cell):
    rect = cell_rect(cell, pad=4)
    pygame.draw.rect(surface, (250, 250, 250), rect, border_radius=4)
    pygame.draw.rect(surface, (40, 120, 220), rect, width=2, border_radius=4)
    cx, cy = rect.center
    pygame.draw.rect(surface, (40, 120, 220), (cx - 2, cy - 10, 4, 20))
    pygame.draw.rect(surface, (40, 120, 220), (cx - 10, cy - 2, 20, 4))


def draw_blockage(surface, cell):
    rect = cell_rect(cell, pad=2)
    pygame.draw.rect(surface, COL_BLOCKAGE, rect, border_radius=3)
    pygame.draw.line(surface, (255, 255, 255), rect.topleft, rect.bottomright, 4)
    pygame.draw.line(surface, (255, 255, 255), rect.topright, rect.bottomleft, 4)
    pygame.draw.rect(surface, COL_BLOCKAGE_DARK, rect, width=2, border_radius=3)


def draw_scene(surface, sim):
    draw_grid(surface)
    draw_static_obstacles(surface, sim)

    for cell in sim["initial_path"]:
        draw_soft_cell(surface, cell, COL_INITIAL_PATH, alpha=140, size_ratio=0.42)

    if sim["replans"] > 0:
        for cell in sim["current_path"][sim["path_index"]:]:
            draw_soft_cell(surface, cell, COL_NEW_PATH, alpha=190, size_ratio=0.5)

    for cell in sim["travelled"]:
        draw_soft_cell(surface, cell, COL_TRAVELLED, alpha=210, size_ratio=0.34)

    if sim["dynamic_obstacle"] is not None:
        draw_blockage(surface, sim["dynamic_obstacle"])

    draw_hospital(surface, HOSPITAL)
    draw_ambulance(surface, sim["position"])


# ----------------------------------------------------------------------
# 6. INFO PANEL
# ----------------------------------------------------------------------

def status_text(sim):
    if sim["state"] == "INIT_DELAY":
        return "PLANNING", COL_ACCENT
    if sim["state"] == "BLOCKED_PAUSE":
        return "REPLANNING", COL_WARN
    if sim["state"] == "ARRIVED":
        return "ARRIVED", COL_SUCCESS
    return "MOVING", COL_ACCENT


def draw_panel(surface, sim, fonts):
    f_title, f_label, f_bold, f_mono, f_banner = fonts

    panel = pygame.Rect(GRID_W, 0, PANEL_W, WINDOW_H)
    surface.fill(COL_PANEL_BG, panel)

    x = GRID_W + 20
    y = 18

    surface.blit(f_title.render("EMERGENCY RESPONSE SYSTEM", True, COL_PANEL_TEXT), (x, y))
    y += 34

    lines = [
        ("Agent", "AMBULANCE"),
        ("Algorithm", "A*"),
        ("Mode", "DYNAMIC REPLANNING"),
    ]
    for label, value in lines:
        surface.blit(f_label.render(f"{label}:", True, COL_PANEL_DIM), (x, y))
        surface.blit(f_bold.render(value, True, COL_PANEL_TEXT), (x + 100, y))
        y += 22

    y += 8
    pygame.draw.line(surface, (60, 64, 74), (x, y), (x + PANEL_W - 40, y))
    y += 14

    status, status_color = status_text(sim)
    surface.blit(f_label.render("Status:", True, COL_PANEL_DIM), (x, y))
    surface.blit(f_bold.render(status, True, status_color), (x + 100, y))
    y += 22

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

    y += 10

    # Status banner
    if sim["state"] == "ARRIVED":
        banner_bg, banner_lines = COL_SUCCESS, ["EMERGENCY RESOLVED", "HOSPITAL REACHED"]
    elif sim["dynamic_obstacle"] is not None and sim["replans"] == 0:
        banner_bg, banner_lines = COL_WARN, ["WARNING: TRAFFIC BLOCKAGE", "REPLANNING..."]
    elif sim["replans"] > 0:
        banner_bg, banner_lines = COL_SUCCESS, ["NEW ROUTE FOUND", "CONTINUING TO HOSPITAL"]
    else:
        banner_bg, banner_lines = None, None

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
        y += 10

    # Mission complete metrics block
    if sim["state"] == "ARRIVED":
        y += 4
        pygame.draw.line(surface, (60, 64, 74), (x, y), (x + PANEL_W - 40, y))
        y += 14
        surface.blit(f_bold.render("MISSION COMPLETE", True, COL_SUCCESS), (x, y))
        y += 26
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
        y += 10

    # Live log
    pygame.draw.line(surface, (60, 64, 74), (x, y), (x + PANEL_W - 40, y))
    y += 10
    surface.blit(f_label.render("LIVE LOG", True, COL_PANEL_DIM), (x, y))
    y += 18

    max_y = WINDOW_H - 34
    for msg in sim["messages"][-9:]:
        if y > max_y:
            break
        clipped = msg if len(msg) <= 44 else msg[:41] + "..."
        surface.blit(f_mono.render(clipped, True, (170, 210, 190)), (x, y))
        y += 15

    hint = "SPACE pause/resume   R restart   ESC quit"
    surface.blit(f_mono.render(hint, True, COL_PANEL_DIM), (x, WINDOW_H - 20))


# ----------------------------------------------------------------------
# 7. MAIN LOOP
# ----------------------------------------------------------------------

def run_simulation():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Emergency Response Ambulance - Dynamic A* Replanning")
    clock = pygame.time.Clock()

    f_title = pygame.font.SysFont("arial", 17, bold=True)
    f_label = pygame.font.SysFont("arial", 14)
    f_bold = pygame.font.SysFont("arial", 14, bold=True)
    f_banner = pygame.font.SysFont("arial", 14, bold=True)
    f_mono = pygame.font.SysFont("couriernew,consolas,monospace", 12)
    fonts = (f_title, f_label, f_bold, f_mono, f_banner)

    sim = new_sim()
    running = True

    while running:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    sim["paused"] = not sim["paused"]
                elif event.key == pygame.K_r:
                    sim = new_sim()

        update(sim)

        screen.fill(COL_PANEL_BG)
        draw_scene(screen, sim)
        draw_panel(screen, sim, fonts)
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    run_simulation()
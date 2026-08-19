# Emergency Response Ambulance — Interactive Dynamic A* Replanning

## Track
drive link:https://drive.google.com/file/d/1N8k6J8n5OsTKTQBpK5a9CrvNDj5F6TQH/view?usp=drive_link
Track 4 — Emergency Response Ambulance

## Description

An ambulance navigates a dynamic grid toward a hospital using A* search.
While it is en route, a traffic blockage appears on its planned path
(auto-generated, or dropped in by you). The agent detects that its route
is no longer valid, replans a new path with A* from its current
position, and continues to the hospital — all in a single continuous
run, with no restart required.

This version is fully interactive and dark-themed: you design the map
before launch and steer events during the run instead of just watching
a scripted demo.

## Features

- Real A* pathfinding (not scripted or hardcoded)
- **Map editor** before every run:
  - Click any cell to add/remove a static obstacle
  - `SET START` / `SET HOSPITAL` then click a cell to relocate either one
  - `RANDOM MAP` generates a random (always-solvable) obstacle field
  - `CLEAR OBSTACLES` / `RESET GRID` for a clean slate
  - A **live A* preview** (path, cost, nodes expanded) updates instantly
    as you edit, so you can see how "optimal" your layout is before
    starting — great for hand-crafting harder or shorter routes
- **Interactive run**:
  - Click the grid at any time to drop your own traffic blockage
    exactly where you want it (toggle `ADD BLOCKAGE` mode)
  - `AUTO BLOCKAGE` toggle keeps the original hands-off auto-spawn
    behavior if you'd rather just watch
  - Pause / resume and restart at any time without leaving the run
  - `BACK TO MAP EDITOR` returns to editing without closing the app
- Visual path tracing: initial plan, replanned route, and travelled
  trail are all shown in distinct colors simultaneously
- Live decision-making logs printed to the console and shown on screen
- On-screen performance metrics (path cost, nodes expanded, replans,
  steps, execution time)
- Dark theme throughout, hover highlighting, and toggle/active button
  states

## Algorithm

```
f(n) = g(n) + h(n)
h(n) = |x1 - x2| + |y1 - y2|      (Manhattan distance)
```

- **State:** a grid cell `(row, column)`
- **Actions:** move Up, Down, Left, Right (no diagonals)
- **Step cost:** 1 per move
- **Goal test:** current cell == hospital cell

When a new obstacle appears on the ambulance's remaining route, A* is
re-run from the ambulance's *current* cell to the hospital, treating the
new obstacle (and all previously placed ones) as permanent walls. The
result replaces the active plan and the ambulance resumes driving
immediately.

## How to Run

```bash
pip install -r requirements.txt
python main.py
```

## Controls

**Map editor (before launch):**
- Left-click a cell — toggle a static obstacle
- `SET START` / `SET HOSPITAL` buttons, then click a cell to place it
- `RANDOM MAP` — generate a new random obstacle field
- `CLEAR OBSTACLES` — remove all obstacles
- `RESET GRID` — restore the default layout
- `START SIMULATION` — launch the run with the current map

**During the run:**
- `ADD BLOCKAGE` toggle, then click any cell to drop a blockage there
- `AUTO BLOCKAGE` toggle — turn the automatic random blockage on/off
- `PAUSE` / `RESUME`
- `RESTART` — relaunch the same map from scratch
- `BACK TO MAP EDITOR` — stop and go edit the map again

**Keyboard (always available):**
- `SPACE` — pause / resume (during a run)
- `R` — full reset back to the default map editor
- `ESC` — quit

## Demo Sequence

1. The map editor opens with a default layout and a live A* preview
   path shown in teal, along with its cost and nodes expanded.
2. Optionally reshape the map: add/remove obstacles, move the start or
   hospital, or randomize the whole grid — the preview updates live so
   you can aim for a shorter, more optimal route.
3. Click `START SIMULATION`. A* computes the initial route (light
   blue) and the ambulance starts driving, leaving a green trail.
4. A traffic blockage appears — either automatically, or because you
   clicked it in with `ADD BLOCKAGE` mode.
5. The console and on-screen panel report the blockage and
   `REPLANNING...`.
6. A* recalculates a new path (orange) from the ambulance's current
   position and the ambulance follows it to the hospital.
7. A "MISSION COMPLETE" panel displays the final metrics: initial path
   cost, final path cost, replans, nodes expanded, steps taken, and
   execution time.
8. Use `BACK TO MAP EDITOR` or `R` to design a new scenario and go again.

#!/usr/bin/env python3
"""
Maze-solving robot simulator.

- Loads a maze PNG (white background, black wall lines, square cells).
- The robot only learns walls through 4 axis-aligned ToFs (N/E/S/W, robot never
  rotates, mecanum).  Each ToF is a ray from the cell centre: it reveals the
  open edges it passes through and the first wall it hits, up to --range metres.
- Solver: optimistic flood fill toward the goal (unknown walls = open),
  re-flooded after every new wall.  Single run, no map beforehand.
- Output: PNGs with the route in red (full route incl. dead ends, and the
  loop-erased net route), optional GIF of the run, and timing stats.

Usage:
    python maze_sim.py maze.png                  # auto-detect grid size
    python maze_sim.py maze.png --cells 13 --start bl
    python maze_sim.py --demo --seed 3           # generate a random test maze
"""
import argparse
import math
import os
import random
from collections import deque

import numpy as np
from PIL import Image, ImageChops, ImageDraw

DR = (-1, 0, 1, 0)  # N E S W
DC = (0, 1, 0, -1)
OPP = (2, 3, 0, 1)
NAME = "NESW"
UNREACH = 10 ** 9


# --------------------------------------------------------------------------
# Ground-truth maze
# --------------------------------------------------------------------------
class Maze:
    def __init__(self, R, C):
        self.R, self.C = R, C
        self.w = np.zeros((R, C, 4), bool)  # w[r,c,d] = wall on side d of cell

    def inb(self, r, c):
        return 0 <= r < self.R and 0 <= c < self.C

    def set_wall(self, r, c, d, v=True):
        self.w[r, c, d] = v
        nr, nc = r + DR[d], c + DC[d]
        if self.inb(nr, nc):
            self.w[nr, nc, OPP[d]] = v

    def add_perimeter(self):
        self.w[0, :, 0] = True
        self.w[self.R - 1, :, 2] = True
        self.w[:, 0, 3] = True
        self.w[:, self.C - 1, 1] = True


def random_maze(n, loops=0.08, seed=None):
    """Recursive-backtracker maze, then punch some extra holes to make loops."""
    rng = random.Random(seed)
    m = Maze(n, n)
    m.w[:] = True
    seen, stack = {(0, 0)}, [(0, 0)]
    while stack:
        r, c = stack[-1]
        opts = [d for d in range(4)
                if m.inb(r + DR[d], c + DC[d]) and (r + DR[d], c + DC[d]) not in seen]
        if not opts:
            stack.pop()
            continue
        d = rng.choice(opts)
        nr, nc = r + DR[d], c + DC[d]
        m.set_wall(r, c, d, False)
        seen.add((nr, nc))
        stack.append((nr, nc))
    inner = [(r, c, d) for r in range(n) for c in range(n) for d in (1, 2)
             if m.inb(r + DR[d], c + DC[d]) and m.w[r, c, d]]
    rng.shuffle(inner)
    for r, c, d in inner[:int(loops * len(inner))]:
        m.set_wall(r, c, d, False)
    return m


def render_maze_png(m, path, cell=40, margin=24, t=4):
    W, H = m.C * cell + 2 * margin, m.R * cell + 2 * margin
    img = Image.new("L", (W, H), 255)
    dr = ImageDraw.Draw(img)
    h = t // 2
    for r in range(m.R):
        for c in range(m.C):
            x, y = margin + c * cell, margin + r * cell
            if m.w[r, c, 0]:
                dr.rectangle([x - h, y - h, x + cell - h + t - 1, y - h + t - 1], fill=0)
            if m.w[r, c, 2]:
                y2 = y + cell
                dr.rectangle([x - h, y2 - h, x + cell - h + t - 1, y2 - h + t - 1], fill=0)
            if m.w[r, c, 3]:
                dr.rectangle([x - h, y - h, x - h + t - 1, y + cell - h + t - 1], fill=0)
            if m.w[r, c, 1]:
                x2 = x + cell
                dr.rectangle([x2 - h, y - h, x2 - h + t - 1, y + cell - h + t - 1], fill=0)
    img.save(path)


# --------------------------------------------------------------------------
# PNG -> Maze
# --------------------------------------------------------------------------
def load_png(path, cells=None, thresh=128):
    im = Image.open(path)
    if im.mode in ("RGBA", "LA", "P"):
        im = im.convert("RGBA")
        bg = Image.new("RGBA", im.size, (255, 255, 255, 255))
        im = Image.alpha_composite(bg, im)
    gray = np.array(im.convert("L"))
    dark_full = gray < thresh
    ys, xs = np.nonzero(dark_full)
    if len(xs) == 0:
        raise SystemExit("No dark pixels found in image.")
    y0, y1, x0, x1 = ys.min(), ys.max(), xs.min(), xs.max()
    sub = dark_full[y0:y1 + 1, x0:x1 + 1]
    H, W = sub.shape

    # outer line thickness: median leading dark run from each border
    def lead_runs(a):  # a: 2D, runs along axis 0 from index 0
        out = []
        for j in range(a.shape[1]):
            col = a[:, j]
            if col[0]:
                nz = (~col).argmax() if (~col).any() else len(col)
                out.append(nz)
        return out

    runs = lead_runs(sub) + lead_runs(sub.T)
    t = float(np.median(runs)) if runs else 3.0
    xc0, yc0 = (t - 1) / 2, (t - 1) / 2
    Wc, Hc = (W - 1) - (t - 1), (H - 1) - (t - 1)  # span between outer line centres

    py, px = np.nonzero(sub)
    band = t / 2 + 1.5

    def coverage(cw):
        fx = (px - xc0) / cw
        fy = (py - yc0) / cw
        near_x = np.abs(fx - np.round(fx)) * cw <= band
        near_y = np.abs(fy - np.round(fy)) * cw <= band
        return float((near_x | near_y).mean())

    if cells is None:
        for n in range(3, 61):
            cw = Wc / n
            if coverage(cw) >= 0.99:
                cells = n
                break
        if cells is None:
            raise SystemExit("Could not auto-detect grid size; pass --cells N.")
    C = cells
    cw = Wc / C
    R = int(round(Hc / cw))
    m = Maze(R, C)
    bh = int(math.ceil(t / 2 + 1))

    def covered(vals):
        return vals.mean() if vals.size else 0.0

    for r in range(R):
        for c in range(C):
            # north edge of (r,c) and west edge of (r,c); east/south via neighbours/last row/col
            for d in range(4):
                if d == 0:
                    ylc, xa = yc0 + r * cw, xc0 + c * cw
                    horiz = True
                elif d == 2:
                    ylc, xa = yc0 + (r + 1) * cw, xc0 + c * cw
                    horiz = True
                elif d == 3:
                    xlc, ya = xc0 + c * cw, yc0 + r * cw
                    horiz = False
                else:
                    xlc, ya = xc0 + (c + 1) * cw, yc0 + r * cw
                    horiz = False
                if horiz:
                    xs_ = np.arange(int(xa + 0.25 * cw), int(xa + 0.75 * cw) + 1)
                    ylo, yhi = int(round(ylc - bh)), int(round(ylc + bh)) + 1
                    ylo, yhi = max(ylo, 0), min(yhi, H)
                    xs_ = xs_[(xs_ >= 0) & (xs_ < W)]
                    cov = covered(sub[ylo:yhi, xs_].any(axis=0))
                else:
                    ys_ = np.arange(int(ya + 0.25 * cw), int(ya + 0.75 * cw) + 1)
                    xlo, xhi = int(round(xlc - bh)), int(round(xlc + bh)) + 1
                    xlo, xhi = max(xlo, 0), min(xhi, W)
                    ys_ = ys_[(ys_ >= 0) & (ys_ < H)]
                    cov = covered(sub[ys_, xlo:xhi].any(axis=1))
                if cov >= 0.6:
                    m.set_wall(r, c, d, True)
    m.add_perimeter()
    geom = dict(X0=x0, Y0=y0, xc0=xc0, yc0=yc0, cw=cw, t=t)
    return m, geom, im.convert("RGB")


# --------------------------------------------------------------------------
# Robot knowledge, sensing, solver
# --------------------------------------------------------------------------
def set_known(known, r, c, d, v):
    known[r, c, d] = v
    nr, nc = r + DR[d], c + DC[d]
    if 0 <= nr < known.shape[0] and 0 <= nc < known.shape[1]:
        known[nr, nc, OPP[d]] = v


def sense(known, truth, r, c, max_cells):
    """4 ToF rays from the cell centre. Reveals open edges passed + first wall hit."""
    for d in range(4):
        cr, cc = r, c
        for _ in range(max_cells):
            if truth.w[cr, cc, d]:
                set_known(known, cr, cc, d, 1)
                break
            set_known(known, cr, cc, d, 0)
            cr, cc = cr + DR[d], cc + DC[d]


def flood(known, goal):
    R, C = known.shape[:2]
    dist = np.full((R, C), UNREACH, int)
    dist[goal] = 0
    q = deque([goal])
    while q:
        r, c = q.popleft()
        for d in range(4):
            if known[r, c, d] == 1:
                continue
            nr, nc = r + DR[d], c + DC[d]
            if 0 <= nr < R and 0 <= nc < C and dist[nr, nc] > dist[r, c] + 1:
                dist[nr, nc] = dist[r, c] + 1
                q.append((nr, nc))
    return dist


def pick(dist, known, pos, heading, goal, visits):
    r, c = pos
    best = None
    for d in range(4):
        if known[r, c, d] == 1:
            continue
        nr, nc = r + DR[d], c + DC[d]
        if dist[nr, nc] != dist[r, c] - 1:
            continue
        # tie-breaks: keep going straight, then toward goal, then less visited
        key = (0 if d == heading else 1,
               abs(nr - goal[0]) + abs(nc - goal[1]),
               visits.get((nr, nc), 0), d)
        if best is None or key < best[0]:
            best = (key, d)
    return best[1]


def solve(truth, start, goal, max_cells):
    R, C = truth.R, truth.C
    known = np.full((R, C, 4), -1, np.int8)
    known[0, :, 0] = 1
    known[R - 1, :, 2] = 1
    known[:, 0, 3] = 1
    known[:, C - 1, 1] = 1
    pos, heading = start, -1
    path, visits = [pos], {pos: 1}
    for _ in range(60 * R * C):
        sense(known, truth, pos[0], pos[1], max_cells)
        if pos == goal:
            return path, known, True
        dist = flood(known, goal)
        if dist[pos] >= UNREACH:
            return path, known, False
        d = pick(dist, known, pos, heading, goal, visits)
        pos = (pos[0] + DR[d], pos[1] + DC[d])
        heading = d
        path.append(pos)
        visits[pos] = visits.get(pos, 0) + 1
    return path, known, False


def shortest_path(truth, start, goal):
    prev = {start: None}
    q = deque([start])
    while q:
        p = q.popleft()
        if p == goal:
            break
        for d in range(4):
            if truth.w[p[0], p[1], d]:
                continue
            n = (p[0] + DR[d], p[1] + DC[d])
            if n not in prev:
                prev[n] = p
                q.append(n)
    if goal not in prev:
        return None
    out, p = [], goal
    while p is not None:
        out.append(p)
        p = prev[p]
    return out[::-1]


def loop_erase(path):
    stack, idx = [], {}
    for p in path:
        if p in idx:
            while stack[-1] != p:
                idx.pop(stack.pop())
        else:
            idx[p] = len(stack)
            stack.append(p)
    return stack


def est_time(path, cs, vmax, a):
    runs = []
    for p, q in zip(path, path[1:]):
        d = DIRIDX[(q[0] - p[0], q[1] - p[1])]
        if runs and runs[-1][0] == d:
            runs[-1][1] += 1
        else:
            runs.append([d, 1])
    total = 0.0
    for _, n in runs:
        L = n * cs
        total += (L / vmax + vmax / a) if L >= vmax * vmax / a else 2 * math.sqrt(L / a)
    return total, len(runs)


DIRIDX = {(DR[d], DC[d]): d for d in range(4)}


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------
def cell_px(g, r, c):
    return (g["X0"] + g["xc0"] + (c + 0.5) * g["cw"],
            g["Y0"] + g["yc0"] + (r + 0.5) * g["cw"])


def draw_route(base, g, cells, start, goal, color=(220, 0, 0)):
    img = base.copy()
    d = ImageDraw.Draw(img)
    w = max(2, int(g["cw"] * 0.16))
    pts = [cell_px(g, r, c) for r, c in cells]
    if len(pts) > 1:
        d.line(pts, fill=color, width=w, joint="curve")
    rad = max(3, int(g["cw"] * 0.2))
    for (r, c), col in ((start, (0, 160, 0)), (goal, (0, 80, 220))):
        x, y = cell_px(g, r, c)
        d.ellipse([x - rad, y - rad, x + rad, y + rad], outline=col, width=max(2, w // 2))
    return img


def make_gif(base, g, path, start, goal, out, max_frames=150):
    step = max(1, len(path) // max_frames)
    frames, visited = [], set()
    W, H = base.size
    scale = min(1.0, 600 / max(W, H))
    cw = g["cw"]
    for i in range(0, len(path), step):
        visited.update(path[:i + 1])
        tint = Image.new("RGB", base.size, (255, 255, 255))
        td = ImageDraw.Draw(tint)
        for r, c in visited:
            x, y = cell_px(g, r, c)
            td.rectangle([x - cw / 2, y - cw / 2, x + cw / 2, y + cw / 2], fill=(255, 244, 170))
        f = ImageChops.multiply(base, tint)
        fd = ImageDraw.Draw(f)
        pts = [cell_px(g, r, c) for r, c in path[:i + 1]]
        if len(pts) > 1:
            fd.line(pts, fill=(220, 0, 0), width=max(2, int(cw * 0.16)), joint="curve")
        x, y = pts[-1]
        rad = max(3, int(cw * 0.22))
        fd.ellipse([x - rad, y - rad, x + rad, y + rad], fill=(220, 0, 0))
        if scale < 1:
            f = f.resize((int(W * scale), int(H * scale)))
        frames.append(f.convert("P", palette=Image.ADAPTIVE))
    frames[0].save(out, save_all=True, append_images=frames[1:] + [frames[-1]] * 10,
                   duration=70, loop=0)


# --------------------------------------------------------------------------
def corner(R, C, name):
    return {"tl": (0, 0), "tr": (0, C - 1), "bl": (R - 1, 0), "br": (R - 1, C - 1)}[name]


OPPOSITE = {"tl": "br", "br": "tl", "tr": "bl", "bl": "tr"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("png", nargs="?", help="maze image")
    ap.add_argument("--demo", action="store_true", help="generate a random maze PNG and use it")
    ap.add_argument("--demo-size", type=int, default=13)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--cells", type=int, default=None, help="cells per side (auto-detected if omitted)")
    ap.add_argument("--start", choices=["tl", "tr", "bl", "br"], default="bl", help="start corner (goal = opposite)")
    ap.add_argument("--cell-size", type=float, default=0.22, help="cell pitch in metres")
    ap.add_argument("--range", type=float, default=1.0, help="reliable ToF range in metres")
    ap.add_argument("--vmax", type=float, default=0.8, help="cruise speed m/s (time estimate)")
    ap.add_argument("--accel", type=float, default=2.0, help="accel m/s^2 (time estimate)")
    ap.add_argument("--out", default="sim_out")
    ap.add_argument("--gif", action="store_true")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    if a.demo:
        gt = random_maze(a.demo_size, seed=a.seed)
        a.png = os.path.join(a.out, "demo_maze.png")
        render_maze_png(gt, a.png)
        print(f"generated {a.png}")
    if not a.png:
        ap.error("give a PNG path or --demo")

    truth, g, base = load_png(a.png, a.cells)
    print(f"grid: {truth.R} x {truth.C} cells (cell = {g['cw']:.1f}px, wall = {g['t']:.1f}px)")
    start = corner(truth.R, truth.C, a.start)
    goal = corner(truth.R, truth.C, OPPOSITE[a.start])
    max_cells = int(a.range / a.cell_size - 0.5) + 1

    opt = shortest_path(truth, start, goal)
    if opt is None:
        raise SystemExit("Start and goal are not connected in this maze (check --start / --cells).")
    path, known, ok = solve(truth, start, goal, max_cells)
    stem = os.path.splitext(os.path.basename(a.png))[0]
    full_png = os.path.join(a.out, f"{stem}_route.png")
    net_png = os.path.join(a.out, f"{stem}_net.png")

    net = loop_erase(path)
    draw_route(base, g, path, start, goal).save(full_png)
    draw_route(base, g, net, start, goal).save(net_png)
    if a.gif:
        make_gif(base, g, path, start, goal, os.path.join(a.out, f"{stem}_run.gif"))

    t_run, runs_run = est_time(path, a.cell_size, a.vmax, a.accel)
    t_net, runs_net = est_time(net, a.cell_size, a.vmax, a.accel)
    t_opt, runs_opt = est_time(opt, a.cell_size, a.vmax, a.accel)
    seen = int((known != -1).any(axis=2).sum())
    print(f"solved: {ok}   ToF range: {max_cells} cells ({a.range} m)")
    print(f"route driven : {len(path) - 1} moves, {len(set(path))} unique cells, ~{t_run:.1f} s")
    print(f"net route    : {len(net) - 1} moves, {runs_net} straight runs")
    print(f"true optimum : {len(opt) - 1} moves, {runs_opt} straight runs, ~{t_opt:.1f} s (needs the map)")
    print(f"cells with any wall info: {seen}/{truth.R * truth.C}")
    print("route:", "".join(NAME[DIRIDX[(q[0] - p[0], q[1] - p[1])]] for p, q in zip(path, path[1:])))
    print(f"saved {full_png}\nsaved {net_png}")


if __name__ == "__main__":
    main()
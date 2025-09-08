#%%
import numpy as np
import random
import math
from scipy.spatial import KDTree
import vedo as vd

vd.settings.default_backend = 'vtk'
random.seed(0)

# =========================== Config ===========================
# If your PNG has DARK (black) obstacles on white background -> True
# If your PNG has BRIGHT (white) obstacles on dark background -> False
OBSTACLE_IS_DARK = True
BIN_THRESHOLD = 0.5

# Flip Y when mapping world<->image indices (NumPy row 0 is top)
PICK_Y_FLIP = True

# --- Demo toggles (default: normal mode) ---
COLLISION_ENABLED = True     # when False => no collision checks -> uniform exploration
GOAL_ENABLED = True          # when False => tree won't stop on reaching goal
obstacles_alpha = 1.0        # 1.0 = opaque map; lower values fade the map

# ======================= Data structures ======================
class Sample:
    def __init__(self, x, parent=None):
        # x is [ix, iy] in IMAGE-INDEX coordinates (float ok)
        self.x = np.array(x, dtype=float)
        self.parent = parent  # index of parent in samples list

# ======================= Geometry / RRT =======================
def NearestSample(samples, x):
    tree = KDTree([s.x for s in samples])
    _, sid = tree.query(x)
    return sid

def Collision(img, x1, x2):
    """
    Check if a straight line path between x1 and x2 intersects any obstacle.
    'img' is a 2D boolean array [row(y), col(x)] where True=obstacle.
    x1, x2 are in IMAGE-INDEX coords (ix,iy).
    """
    x1 = np.array(x1, dtype=float)
    x2 = np.array(x2, dtype=float)

    dist = np.linalg.norm(x2 - x1)
    n = int(np.ceil(dist)) + 1
    n = max(n, 2)
    t = np.linspace(0, 1, n)
    pts = x1[None, :] + (x2 - x1)[None, :] * t[:, None]  # [N,2]

    h, w = img.shape
    for px, py in pts:
        ix = int(round(px))
        iy = int(round(py))
        if ix < 0 or ix >= w or iy < 0 or iy >= h:
            return True
        if img[iy, ix]:
            return True
    return False

def getNextSample(dim, samples, stepsize):
    """
    Randomly sample a point, then step from the nearest node towards it by 'stepsize'.
    dim = [width(px), height(px)] in IMAGE-INDEX coords.
    """
    xrand = np.array([random.uniform(0, dim[0]-1), random.uniform(0, dim[1]-1)], dtype=float)

    sid = NearestSample(samples, xrand)
    xnear = samples[sid].x

    d = xrand - xnear
    dist = np.linalg.norm(d)
    if dist > 1e-9:
        d = d / dist
        xnew = xnear + d * min(dist, stepsize)
    else:
        ang = random.uniform(0, 2*np.pi)
        d = np.array([math.cos(ang), math.sin(ang)], dtype=float)
        xnew = xnear + d * stepsize

    xnew[0] = np.clip(xnew[0], 0, dim[0]-1)
    xnew[1] = np.clip(xnew[1], 0, dim[1]-1)
    return Sample(xnew, sid)

def extractPath(samples, goal_idx):
    """Backtrack parents to form a path [ix,iy] from start to goal (inclusive)."""
    path = []
    cur = goal_idx
    while cur is not None:
        path.append(samples[cur].x.copy())
        cur = samples[cur].parent
    path.reverse()
    return path

# ======================= Image I/O & Mapping =======================
def load_obstacle_png(path, obstacle_is_dark=True, threshold=0.5):
    """
    Returns boolean mask 'img' where True = OBSTACLE, preserving your original polarity.
    obstacle_is_dark=True  -> dark (low luminance) pixels are obstacles
    obstacle_is_dark=False -> bright (high luminance) pixels are obstacles
    """
    raw = vd.Image(path).bw()                 # grayscale actor
    gray = raw.tonumpy().astype(float)       # to numpy array
    # normalize defensively
    gmin, gmax = gray.min(), gray.max()
    if gmax > gmin:
        gray = (gray - gmin) / (gmax - gmin)
    if obstacle_is_dark:
        img_bool = gray < threshold
    else:
        img_bool = gray > threshold
    return img_bool

def rebuild_image_actor():
    """Render boolean img as RGB: obstacles=black, free space=white; apply alpha."""
    global vd_img
    rgb = np.where(img[..., None], 0, 255).astype(np.uint8)  # True->black, False->white
    plt.remove("ObstacleImage")
    vd_img = vd.Image(rgb)
    vd_img.name = "ObstacleImage"
    vd_img.pickable(True)
    vd_img.alpha(obstacles_alpha)  # fade/opaque map
    plt.add(vd_img)
    plt.render()

def world_to_img_idx(xw, yw, img_shape, actor, flip_y=True):
    """
    Map world coords (xw, yw) -> numpy indices (ix, iy) of 'img'.
    Uses actor bounds for robust mapping and optional Y flip.
    """
    h, w = img_shape
    x0, x1, y0, y1, _, _ = actor.bounds()
    if (x1 - x0) == 0 or (y1 - y0) == 0:
        return 0, 0
    px = (xw - x0) * (w - 1) / (x1 - x0)
    py = (yw - y0) * (h - 1) / (y1 - y0)
    ix = int(np.clip(np.round(px), 0, w - 1))
    iy = int(np.clip(np.round(py), 0, h - 1))
    if flip_y:
        iy = (h - 1) - iy
    return ix, iy

def img_idx_to_world(ix, iy, img_shape, actor, flip_y=True):
    """Inverse mapping: image indices (ix,iy) -> world coords (xw,yw)."""
    h, w = img_shape
    x0, x1, y0, y1, _, _ = actor.bounds()
    if flip_y:
        py = (h - 1) - iy
    else:
        py = iy
    px = ix
    sx = (x1 - x0) / max(1, (w - 1))
    sy = (y1 - y0) / max(1, (h - 1))
    xw = x0 + px * sx
    yw = y0 + py * sy
    return np.array([float(xw), float(yw)], dtype=float)

# ============================ UI helpers =============================
def _idx_from_evt(evt):
    """Return (ix, iy) pixel indices from an event; None if not pickable/valid."""
    p = getattr(evt, "picked3d", None)
    if p is None:
        return None
    xw, yw = float(p[0]), float(p[1])
    return world_to_img_idx(xw, yw, img.shape, vd_img, flip_y=PICK_Y_FLIP)

# ============================ Callbacks ==============================
def sliderCallback(widget, event):
    """Step-size slider callback."""
    global stepSize
    stepSize = float(widget.value)

def resetTree():
    """Reset only the RRT tree (keep current map & source/dest)."""
    global samples, points, edges, path_lines, goal_reached

    # remove all lines/paths from scene
    for e in edges:
        plt.remove(e)
    for pl in path_lines:
        plt.remove(pl)
    edges.clear()
    path_lines.clear()

    samples = [Sample(source, parent=None)]

    # reset points cloud to only source (draw in WORLD coords)
    src_w = img_idx_to_world(int(round(source[0])), int(round(source[1])),
                             img.shape, vd_img, flip_y=PICK_Y_FLIP)
    pts0 = np.array([np.hstack([src_w, 0.0])])
    points.vertices = pts0

    goal_reached = False

    plt.add(points)
    plt.render()

def resetAll():
    """Reset map back to ORIGINAL + reset the RRT tree (triggered by 'C')."""
    global img
    img = img_orig.copy()   # restore original pixels
    rebuild_image_actor()   # recreate the image actor (alpha respected)
    resetTree()             # then reset the tree

def setSource_idx(ix, iy):
    """Set source in IMAGE-INDEX coords + update marker."""
    global source
    source = np.array([ix, iy], dtype=float)
    resetTree()
    plt.remove("SourceMarker")
    pos_w = img_idx_to_world(ix, iy, img.shape, vd_img, flip_y=PICK_Y_FLIP)
    m = vd.Circle(pos=[pos_w[0], pos_w[1], 0.1], r=3, c="green"); m.name = "SourceMarker"
    plt.add(m); plt.render()

def setDest_idx(ix, iy):
    """Set destination in IMAGE-INDEX coords + update marker."""
    global dest
    dest = np.array([ix, iy], dtype=float)
    plt.remove("DestMarker")
    pos_w = img_idx_to_world(ix, iy, img.shape, vd_img, flip_y=PICK_Y_FLIP)
    m = vd.Circle(pos=[pos_w[0], pos_w[1], 0.1], r=3, c="red"); m.name = "DestMarker"
    plt.add(m); plt.render()

def toggleObstacle_at_idx(ix, iy, r=10):
    """Toggle a disk (radius r, in pixels) around (ix,iy) in IMAGE-INDEX coords."""
    if 0 <= iy < img.shape[0] and 0 <= ix < img.shape[1]:
        y0 = max(0, iy - r); y1 = min(img.shape[0], iy + r + 1)
        x0 = max(0, ix - r); x1 = min(img.shape[1], ix + r + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (xx - ix)**2 + (yy - iy)**2 <= r*r
        sub = img[y0:y1, x0:x1]
        sub[mask] = ~sub[mask]
        rebuild_image_actor()

def toggleObstacle(evt):
    """Right-click: toggle obstacle (brush) at clicked pixel."""
    idx = _idx_from_evt(evt)
    if idx is None:
        return
    ix, iy = idx
    toggleObstacle_at_idx(ix, iy, r=10)

# Track last valid mouse WORLD position for keyboard toggles
last_world = None
def OnMouseMove(evt):
    """Update last_world from mouse moves so the keyboard can use it."""
    global last_world
    p = getattr(evt, "picked3d", None)
    if p is not None and np.isfinite(p[:2]).all():
        last_world = (float(p[0]), float(p[1]))

def KeyPress(evt):
    """Keyboard:
      R = start/stop RRT
      C = reset map to ORIGINAL + reset tree
      K = toggle collisions on/off   (use K to avoid Vedo's default 'x' cutter)
      J = toggle goal-connect on/off
      U = toggle 'Uniform Exploration' demo (collisions OFF, goal OFF, map faded)
      Right Arrow = toggle obstacle at last mouse position
    """
    global timer_id, COLLISION_ENABLED, GOAL_ENABLED, obstacles_alpha
    key = getattr(evt, "keyPressed", getattr(evt, "keypress", None))

    if key in ("R", "r"):
        if timer_id != -1:
            plt.timer_callback("destroy", timer_id); timer_id = -1
        else:
            timer_id = plt.timer_callback("create", dt=10)

    elif key in ("C", "c"):
        resetAll()

    elif key in ("K", "k"):
        COLLISION_ENABLED = not COLLISION_ENABLED
        print(f"Collisions enabled: {COLLISION_ENABLED}")

    elif key in ("J", "j"):
        GOAL_ENABLED = not GOAL_ENABLED
        print(f"Goal connect enabled: {GOAL_ENABLED}")

    elif key in ("U", "u"):
        # Toggle uniform-exploration demo
        if COLLISION_ENABLED or GOAL_ENABLED or obstacles_alpha >= 1.0:
            # turn ON demo
            COLLISION_ENABLED = False
            GOAL_ENABLED = False
            obstacles_alpha = 0.2
            print("Uniform demo: ON (collisions OFF, goal OFF)")
        else:
            # turn OFF demo -> back to normal
            COLLISION_ENABLED = True
            GOAL_ENABLED = True
            obstacles_alpha = 1.0
            print("Uniform demo: OFF (collisions ON, goal ON)")
        rebuild_image_actor()
        resetTree()

    elif key in ("Right", "RightArrow", "KP_Right"):
        if last_world is not None:
            xw, yw = last_world
            ix, iy = world_to_img_idx(xw, yw, img.shape, vd_img, flip_y=PICK_Y_FLIP)
            toggleObstacle_at_idx(ix, iy, r=10)

toggle_dst = False
def LeftButtonPress(evt):
    """Left-click: set Source first, then Destination (toggle), from IMAGE-INDEX coords."""
    global toggle_dst
    idx = _idx_from_evt(evt)
    if idx is None:
        return
    ix, iy = idx
    if not toggle_dst:
        setSource_idx(ix, iy)
        toggle_dst = True
    else:
        setDest_idx(ix, iy)
        toggle_dst = False

def doRRTIteration(evt):
    """Main RRT loop tick (called by Plotter's timer)."""
    global goal_reached, samples, edges, path_lines, timer_id

    if goal_reached:
        return

    h, w = img.shape  # rows, cols
    ns = getNextSample([w, h], samples, stepSize)

    parent = samples[ns.parent]

    # Only block when collisions are enabled
    if COLLISION_ENABLED and Collision(img, parent.x, ns.x):
        return

    # append node
    samples.append(ns)

    # draw edge + point in WORLD coords
    p_new_w = img_idx_to_world(int(round(ns.x[0])), int(round(ns.x[1])),
                               img.shape, vd_img, flip_y=PICK_Y_FLIP)
    p_par_w = img_idx_to_world(int(round(parent.x[0])), int(round(parent.x[1])),
                               img.shape, vd_img, flip_y=PICK_Y_FLIP)

    points.vertices = np.vstack([points.vertices, np.hstack([p_new_w, 0.0])])
    e_line = vd.Line(np.hstack([p_new_w, 0.0]),
                     np.hstack([p_par_w, 0.0]),
                     lw=2, c="blue")
    edges.append(e_line)
    plt.add(e_line)
    plt.add(points)

    # goal check (only if enabled)
    if GOAL_ENABLED:
        d2goal = np.linalg.norm(ns.x - dest)
        ok_direct = True
        if COLLISION_ENABLED:
            ok_direct = not Collision(img, ns.x, dest)
        if d2goal < stepSize * 1.5 and ok_direct:
            goal_reached = True
            goal_sample = Sample(dest, parent=len(samples) - 1)
            samples.append(goal_sample)

            path = extractPath(samples, len(samples) - 1)

            # draw the path thicker/green in WORLD coords
            for i in range(len(path) - 1):
                a = img_idx_to_world(int(round(path[i][0])),   int(round(path[i][1])),
                                     img.shape, vd_img, flip_y=PICK_Y_FLIP)
                b = img_idx_to_world(int(round(path[i+1][0])), int(round(path[i+1][1])),
                                     img.shape, vd_img, flip_y=PICK_Y_FLIP)
                pl = vd.Line(np.hstack([a, 0.2]), np.hstack([b, 0.2]), lw=4, c="green")
                pl.name = "PathSeg"
                path_lines.append(pl)
                plt.add(pl)

            plt.timer_callback("destroy", timer_id)
            timer_id = -1
            print(f"Goal reached. Path waypoints: {len(path)} | Samples: {len(samples)}")

    plt.render()

# ============================ App Setup ==============================
imagePath = 'obstacle_map.png'  # optional; if not found, a default map is created
stepSize = 10.0
goal_reached = False
toggle_dst = False  # left-click toggles Source -> Dest

# Load ORIGINAL map or create default (True means OBSTACLE)
try:
    img_orig = load_obstacle_png(imagePath, obstacle_is_dark=OBSTACLE_IS_DARK, threshold=BIN_THRESHOLD)
except Exception:
    print("Creating default obstacle map...")
    img_orig = np.zeros((400, 500), dtype=bool)
    img_orig[100:150, 200:250] = True
    img_orig[250:300, 100:150] = True
    img_orig[150:200, 350:400] = True

# Working copy we edit during drawing/toggling
img = img_orig.copy()

h, w = img.shape

# Initial source/dest in IMAGE-INDEX coords
source = np.array([50.0, 50.0])           # (ix,iy)
dest   = np.array([min(400.0, w-1), min(300.0, h-1)])

# Plotter (white background for contrast)
plt = vd.Plotter(title="RRT Path Planning", bg="white")
plt.user_mode('2d')

# Image actor for obstacles (RGB) + initial drawables
vd_img = None  # set by rebuild_image_actor()
rebuild_image_actor()

samples = [Sample(source)]
# points actor must draw in WORLD coords:
src_w = img_idx_to_world(int(round(source[0])), int(round(source[1])), img.shape, vd_img, flip_y=PICK_Y_FLIP)
points = vd.Points([np.hstack([src_w, 0.0])], c='yellow', r=5)
edges: list[vd.Line] = []       # list of vd.Line
path_lines: list[vd.Line] = []  # list of vd.Line
plt.add(points)

# Step size slider (bottom-left to bottom-right)
slider = plt.add_slider(
    sliderCallback,
    xmin=5, xmax=50, value=stepSize,
    title="Step Size",
    pos=[(0.1, 0.05), (0.4, 0.05)]
)

# Markers (drawn in WORLD coords)
src_marker_w = img_idx_to_world(int(round(source[0])), int(round(source[1])), img.shape, vd_img, flip_y=PICK_Y_FLIP)
dst_marker_w = img_idx_to_world(int(round(dest[0])),   int(round(dest[1])),   img.shape, vd_img, flip_y=PICK_Y_FLIP)
src_marker = vd.Circle(pos=[src_marker_w[0], src_marker_w[1], 0.1], r=3, c='green'); src_marker.name = "SourceMarker"
dst_marker = vd.Circle(pos=[dst_marker_w[0]], pos2=[dst_marker_w[1], 0.1], r=3, c='red') if False else vd.Circle(pos=[dst_marker_w[0], dst_marker_w[1], 0.1], r=3, c='red')
dst_marker.name = "DestMarker"
plt.add(src_marker); plt.add(dst_marker)

# Event callbacks
plt.add_callback('MouseMove',        OnMouseMove)
plt.add_callback('KeyPress',         KeyPress)
plt.add_callback('LeftButtonPress',  LeftButtonPress)
plt.add_callback('RightButtonPress', toggleObstacle)

# RRT timer (starts stopped; press 'R' to run/stop)
timer_id = -1
plt.add_callback("timer", doRRTIteration, enable_picking=False)

# Instructions
instructions = f"""
Instructions:
- Press 'R' to start/stop the RRT
- Press 'C' to RESTORE the map to the ORIGINAL image and reset the tree
- Press 'U' to toggle UNIFORM EXPLORATION demo (collisions OFF, goal OFF, map faded)
- Press 'K' to toggle collisions; 'J' to toggle goal-connect   (avoid Vedo's 'X' cutter)
- Left-click: set Source (green) then Destination (red), alternating
- Right-click: toggle a BRUSH (radius=10px) of obstacles at clicked pixel
- Right Arrow: toggle a brush dab at the last mouse position
- Use the slider to adjust the RRT step size
- Obstacles are black, free space is white (polarity preserved: OBSTACLE_IS_DARK={OBSTACLE_IS_DARK})
"""
plt.add(vd.Text2D(instructions, pos="top-left", s=0.6, c='b'))

# Go
plt.show(zoom="tightest").close()
# %%

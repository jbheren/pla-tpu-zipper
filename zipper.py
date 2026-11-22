#!/usr/bin/env python3
"""Print-in-place PLA/TPU zipper: generates every part of the frozen design.

Run `python zipper.py` (or `--teeth N`, default 18 per row). Writes binary STLs
to ./stl/:

  print-chain-row1-pla.stl   print-chain-row2-pla.stl   print-slider-n.stl
  print-chain-row1-tpu.stl   print-chain-row2-tpu.stl
  usage-chain-row1-pla.stl   usage-chain-row2-pla.stl   usage-slider-n.stl
  usage-chain-row1-tpu.stl   usage-chain-row2-tpu.stl

`print-*` are in bed poses (rows flat, second row pulled out of the socket,
slider standing on its mouth with print tabs). `usage-*` are in assembled
poses. Dependencies: numpy, manifold3d, shapely.

Coordinate conventions (mm):
  Chain: u along the rows, v across (row 1 on v < 0, row 2 mirrored on v > 0),
  z up. Row 2 is row 1 rotated 180 deg about z.
  Slider: own frame, x towards the mouth (x = 0 at the narrow end), teeth
  engage at x = 0 along a path given by frame(); z between the two plates.
  Print pose: slider stands on its mouth (matrix R_SL), narrow end at z = 0.
"""
from __future__ import annotations

import argparse
import math

import numpy as np
from manifold3d import CrossSection, Manifold, OpType
from shapely import affinity
from shapely.geometry import LineString, Polygon, box as sbox
from shapely.ops import unary_union

# ---------------------------------------------------------------- tooth / tape
PITCH = 5.0                    # tooth spacing
T = 2.4                        # tooth height (12 layers)
HEAD, NECK = 3.2, 1.5          # dovetail head / neck widths
ROOT_H = 1.7                   # root half width
EDGE = 5.4                     # tape edge distance from row axis
ROOT_V = 2.2                   # root extends from EDGE to 2.2 from the axis
HEAD_V = (0.2, 1.8)            # head v-range
R_TIP, R_BACK = 0.7, 0.2       # tip / back rounding radii
LOCK_D = 0.6                   # neck rib height (z lock)
RIB_Z = (0.8, 1.6)             # rib z-range (full layers)
GROOVE_Z = (0.6, 1.8)          # head groove z-range
LOCK_CH = (0.1, 0.2)           # rib crest chamfer (u x z)
RIDGE_V = (-ROOT_V - 0.05, -0.3)   # rib runs along the neck, root to 0.3 under the axis
TAPE_W = 3.0                   # tape width
BULB_T = 1.2                   # tape anchor bulb height
U1 = 5.0                       # first row-1 tooth; row 2 offset by half a pitch
TAPE_B = 1.6                   # tape thickness (TPU, eight layers)
TAPE_N = 0.8                   # tape thickness the slider lips were designed for
GROOVE_V = (3.0, 4.6)          # root groove (slider rib runs here), distance to axis
GROOVE_D = 0.8                 # groove depth

# ---------------------------------------------------------------- slider
J = 0.3                        # working clearance per side
PLATE_T = 1.5
THETA, BEND_R = math.radians(25.0), 10.0   # path: straight, arc, then 25 deg
S_NARROW, S_MOUTH = -4.0, 11.4             # slider extent along its path
POST_S0 = 6.4                    # post starts here (between the rows)
POST_CLEAR = 0.5                 # air between post and tooth tips
X_END = 13.5                     # plates and post end here
POST_S1 = 12.0                   # post template drawn up to here, clipped at X_END
POST_STOP_CLEAR = 0.3            # air between post and upper-stop nose
RIB_C = EDGE - (GROOVE_V[0] + GROOVE_V[1]) / 2   # slider rib axis, from tape edge
RIB_TAPER = 3.0                  # rib tapers to 0.4 wide over its last 3 mm
NOTCH_S, NOTCH_D = (7.0, 10.5), 0.4   # rib notch (unused on this chain, kept for shape parity)
SL_OUT = 1.6                     # plates overhang the tapes by this
LIP_W = 1.0                      # lip return width
LIP_AIR_Z = 0.2                  # air above the tape when the slider is bottomed
LIP_RELIEF = 0.6                 # lip rises 0.6 near the narrow end ...
LIP_RELIEF_L = 4.0               # ... over 4 mm from the end, then ramps back at 45 deg
NERVE_RELIEF = 0.2               # material removed from each slider-rib flank, s 0 to 5
MARK_D = 0.6                     # play notches: depth in the top plate face

# ---------------------------------------------------------------- socket / pin (row-2 entry)
U_BOX = (-8.0, -1.0)             # socket block: PLA, on row 1; open slider bottoms on it
U_FACE = U_BOX[1]                # socket face towards the chain
BOX_T = 3.9                      # socket taller than the chain: top plate stops on it
PIN_U = (-6.5, 4.8)              # pin slot range (used by skin/veil references)
WEB_V, WEB_Z = (3.6, 4.8), 0.6   # T-bar pin stem: 1.2 wide, plate to z 0.6
BAR_V, BAR_Z = (3.0, 5.4), (1.2, 1.6)   # T-bar pin bar; underside at 45 deg
BOX_V1 = BAR_V[1] + 1.6          # socket outer wall
PIN_T = 1.6                      # pin/bar top height (used by the end-stop ramp)
FILL_U0 = -0.7                   # TPU under the pin bar starts here
FREE = 1.0                       # bare TPU between each PLA anchor and the first root
PIN_IN = 3.5                     # pin engagement into the socket
PIN_U0 = U_FACE - PIN_IN         # pin back stop
PIN_FIT = (0.00, 0.25, 0.25)     # (plan, 45 deg vertical, roof) clearances
LEAD, LEAD_N = 0.30, 3           # socket entry: profile widened in 3 steps
BUTT_U1 = U1 - ROOT_H - FREE     # row-1 buttress reaches here
BULBS1 = (0.0, 0.9)              # row-1 tape anchor bulbs (between socket and first root)
GAP_OPEN = 6.0                   # print pose: row 2 pulled 6 mm in v ...
PIN_OUT = 9.0                    # ... and pin pulled 9 mm out of the socket
P_USAGE = 24.3                   # usage x of the slider for checks

# ---------------------------------------------------------------- stops (top of the chain)
STOP_LEN, STOP_REACH = 3.0, 9.0  # rigid PLA stop: length / nose reach
ARRET_REACH, ARRET_T = 3.2, T    # soft TPU end-stop: reach / height

# ---------------------------------------------------------------- slider letter "N"
LET = dict(cx=4.0, cy=0.0, ax=2.3, ay=1.7, stroke=0.9, depth=0.6)

# ---------------------------------------------------------------- base letter "B"
LET_B = dict(h=6.0, w=5.0, stroke=1.5, depth=0.6)

# ---------------------------------------------------------------- print tabs (ears)
EAR_D = 10.0            # disc diameter
EAR_T = 0.4             # two layers
EAR_NECK = 8.0          # web joining each disc to the part edge
EAR_GAP = 0.3           # disc stands 0.3 off the edge
EAR_Y_IN = 5.0          # disc centre, 5.0 from the plate end

BEC_T, BEC_L = 2.0, 3.0  # mouth lip: 2.0 proud of each plate over 3.0, then a 45 deg slope

R_SL = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0]], dtype=float)  # stand slider on its mouth


# ================================================================ low-level helpers
def ccw(pts):
    a = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return pts if a > 0 else pts[::-1]


def box(u0, v0, z0, u1, v1, z1) -> Manifold:
    return Manifold.cube([u1 - u0, v1 - v0, z1 - z0]).translate([u0, v0, z0])


def union(parts) -> Manifold:
    parts = list(parts)
    return Manifold.batch_boolean(parts, OpType.Add) if len(parts) > 1 else parts[0]


def from_shapely(geom, z0, z1) -> Manifold:
    """Extrude a shapely polygon (or MultiPolygon) from z0 to z1."""
    if geom.is_empty:
        return Manifold()
    polys = list(geom.geoms) if hasattr(geom, "geoms") else [geom]
    rings = []
    for p in polys:
        if p.geom_type != "Polygon" or p.area < 1e-9:
            continue
        rings.append(ccw([tuple(c) for c in p.exterior.coords[:-1]]))
        for hole in p.interiors:
            rings.append(ccw([tuple(c) for c in hole.coords[:-1]])[::-1])
    return CrossSection(rings).extrude(z1 - z0).translate([0, 0, z0])


def prism_uz(pts, v0, v1) -> Manifold:
    """(u, z) section extruded along v."""
    m = CrossSection([ccw(list(pts))]).extrude(v1 - v0)
    return m.transform(np.array([[1, 0, 0, 0], [0, 0, -1, v1], [0, 1, 0, 0]], dtype=float))


def prism_vz(pts, u0, u1) -> Manifold:
    """(v, z) section extruded along u."""
    m = CrossSection([ccw(list(pts))]).extrude(u1 - u0)
    return m.transform(np.array([[0, 0, 1, u0], [1, 0, 0, 0], [0, 1, 0, 0]], dtype=float))


def clean(m: Manifold) -> Manifold:
    """Drop zero-volume slivers left by booleans between coplanar faces (z = 0)."""
    return union([q for q in m.decompose() if q.volume() > 1e-3])


# ================================================================ tooth
def head_2d():
    """Dovetail head, row 1 convention (head towards v+)."""
    h = HEAD / 2
    back = sbox(-h + R_BACK, HEAD_V[0] + R_BACK, h - R_BACK, 1.1).buffer(R_BACK, resolution=8)
    tip = sbox(-h + R_TIP, HEAD_V[0] + R_TIP, h - R_TIP, HEAD_V[1] - R_TIP).buffer(R_TIP, resolution=8)
    return unary_union([back, tip])


def tooth_2d():
    """Flat outline of one row-1 tooth: root, neck, head."""
    root = sbox(-ROOT_H, -EDGE, ROOT_H, -ROOT_V)
    neck = sbox(-NECK / 2, -ROOT_V - 0.1, NECK / 2, HEAD_V[0] + 0.1)
    return unary_union([root, neck, head_2d()])


def rib(u0: float, sign: float, v0: float, v1: float) -> Manifold:
    """Neck rib along v: flat foot on u = u0, chamfered crest at u0 + sign * LOCK_D."""
    (z0, z1), (cu, cz) = RIB_Z, LOCK_CH
    f, d = sign * (LOCK_D - cu), sign * LOCK_D
    pts = [(u0 - sign * 0.1, z0), (u0 + f, z0), (u0 + d, z0 + cz), (u0 + d, z1 - cz), (u0 + f, z1), (u0 - sign * 0.1, z1)]
    return prism_uz(pts, v0, v1)


def groove(u0: float, sign: float, v0: float, v1: float) -> Manifold:
    """Head groove along v: open on u = u0, flat bottom at u0 + sign * LOCK_D."""
    z0, z1 = GROOVE_Z
    return prism_uz([(u0 - sign * 1.0, z0), (u0 + sign * LOCK_D, z0), (u0 + sign * LOCK_D, z1), (u0 - sign * 1.0, z1)], v0, v1)


def tooth_pla() -> Manifold:
    """PLA tooth (row 1 convention), centred at u = 0, sitting at z = 0."""
    m = from_shapely(tooth_2d(), 0, T)
    h, n = HEAD / 2, NECK / 2
    for sg in (-1, 1):
        m = m + rib(sg * n, sg, RIDGE_V[0], RIDGE_V[1])                       # neck rib
        m = m - groove(sg * h, -sg, HEAD_V[0] - 0.5, HEAD_V[1] + 0.5)         # head groove
    m = m - box(-5, -GROOVE_V[1], T - GROOVE_D, 5, -GROOVE_V[0], T + 1)       # root groove
    return m - bulb(0.0)


def bulb(u: float) -> Manifold:
    """TPU tape anchor moulded into the root."""
    return box(u - 0.5, -EDGE - 0.01, 0, u + 0.5, -EDGE + 0.61, BULB_T) + box(u - 1.0, -EDGE + 0.6, 0, u + 1.0, -EDGE + 1.8, BULB_T)


def flip(m: Manifold) -> Manifold:
    """Row-2 tooth: row 1 rotated half a turn about z (no mirroring)."""
    return m.rotate([0, 0, 180])


def row_u(row: int, n_teeth: int):
    return [U1 + (PITCH / 2 if row == 2 else 0.0) + PITCH * k for k in range(n_teeth)]


# ================================================================ slider path
def frame(s: float):
    """Tape edge of row 1 inside the slider: straight (s <= 0), arc of radius
    BEND_R, then straight at -THETA."""
    arc = BEND_R * THETA
    if s <= 0:
        return s, -EDGE, 0.0
    if s < arc:
        a = s / BEND_R
        return BEND_R * math.sin(a), -EDGE - BEND_R * (1 - math.cos(a)), -a
    x0, y0 = BEND_R * math.sin(THETA), -EDGE - BEND_R * (1 - math.cos(THETA))
    d = s - arc
    return x0 + d * math.cos(THETA), y0 - d * math.sin(THETA), -THETA


def band(off0: float, off1: float, s0: float, s1: float, n: int = 160):
    """Band between two offsets of the row-1 tape edge (positive: towards the axis)."""
    lo, hi = [], []
    for s in np.linspace(s0, s1, n):
        x, y, a = frame(s)
        nx, ny = -math.sin(a), math.cos(a)
        lo.append((x + off0 * nx, y + off0 * ny))
        hi.append((x + off1 * nx, y + off1 * ny))
    return Polygon(lo + hi[::-1]).buffer(0)


# ================================================================ slider
def relief(sv: float, delta: float = NERVE_RELIEF, s0: float = 0.0, s1: float = 5.0, ramp: float = 3.0) -> float:
    """Material removed from a slider-rib flank: `delta` between s0 and s1,
    linear ramps of `ramp` outside."""
    if delta <= 0:
        return 0.0
    if s0 <= sv <= s1:
        return delta
    if s0 - ramp <= sv < s0:
        return delta * (sv - (s0 - ramp)) / ramp
    if s1 < sv <= s1 + ramp:
        return delta * ((s1 + ramp) - sv) / ramp
    return 0.0


def slider_rib_2d(j: float):
    """Slider rib guiding in the root groove: groove width minus two clearances,
    tapered at the mouth, thinned by relief() on both flanks between s 0 and 5."""
    w = (GROOVE_V[1] - GROOVE_V[0]) - 2 * j
    lo, hi = [], []
    for sv in np.linspace(S_NARROW, S_MOUTH, 200):
        k = min(1.0, max(0.0, (S_MOUTH - sv) / RIB_TAPER))
        h = (0.4 + (w - 0.4) * k) / 2
        h = max(0.1, h - relief(sv))
        x, y, a = frame(sv)
        nx, ny = -math.sin(a), math.cos(a)
        lo.append((x + (RIB_C - h) * nx, y + (RIB_C - h) * ny))
        hi.append((x + (RIB_C + h) * nx, y + (RIB_C + h) * ny))
    return Polygon(lo + hi[::-1]).buffer(0)


def post_2d():
    """Post: the V between the tooth-tip rows, seen from the slider. Past the
    upper stop face, the stop's nose occupies the row-2 side: the post goes
    around it (POST_STOP_CLEAR air) and stays on the row-1 side."""
    off = EDGE + HEAD_V[1] + POST_CLEAR
    pts = []
    for sv in np.linspace(POST_S0, POST_S1, 60):
        x, y, a = frame(sv)
        pts.append((x - off * math.sin(a), y + off * math.cos(a)))
    pts = [q for q in pts if q[1] < -0.05]
    wedge = Polygon(pts + [(x, -y) for x, y in pts[::-1]]).buffer(0)
    # forbidden zone: the row-2 stop nose, extended along its row
    k = math.tan(THETA) * STOP_REACH
    nose = Polygon([(0, -EDGE), (-k, -EDGE + STOP_REACH), (-k + 40.0, -EDGE + STOP_REACH), (40.0, -EDGE)])
    x, y, a = frame(S_MOUTH)
    nose = affinity.translate(affinity.rotate(affinity.translate(nose, 0, EDGE), math.degrees(a), origin=(0, 0)), x, y)
    nose = affinity.translate(affinity.scale(nose, 1, -1, origin=(0, 0)), 0, -POST_STOP_CLEAR)
    face = frame(S_MOUTH)[0]
    appui = wedge.intersection(sbox(0, -40, face, 40))          # before the stop face
    suite = wedge.difference(nose).intersection(sbox(face, -40, 40, 40)).buffer(-0.2).buffer(0.2)
    return appui.union(suite)


def post_solid(j: float) -> Manifold:
    """Post alone: between the plates (z -j to T + j), up to X_END."""
    return from_shapely(post_2d().intersection(sbox(frame(S_NARROW)[0], -40, X_END, 40)), -j - 0.01, T + j + 0.01)


def slider_outline(j: float):
    """Plan outline of both plates: convex hull of the two tape lanes."""
    mir = lambda g: affinity.scale(g, 1, -1, origin=(0, 0))  # noqa: E731
    lane = band(-SL_OUT, EDGE + 0.1, S_NARROW - 1, S_MOUTH + 3)
    return unary_union([lane, mir(lane)]).convex_hull.intersection(sbox(frame(S_NARROW)[0], -40, X_END, 40))


def lip_z(j: float) -> float:
    """Underside of the lips: LIP_AIR_Z above the 0.8 tape when bottomed."""
    return TAPE_N + j + LIP_AIR_Z


def lips_2d(j: float):
    """Lips in plan: a strip LIP_W wide, at j from the root edge, along both rows."""
    lane = band(-(j + LIP_W), -j, S_NARROW - 1, S_MOUTH + 6)
    return lane.union(affinity.scale(lane, 1, -1, origin=(0, 0))).intersection(slider_outline(j))


def lips(j: float) -> Manifold:
    """Lips, from lip_z up to the top plate. With the top plate and the rib they
    form a C-slide around each row. The narrow end opens with a 45 deg ramp
    and the underside rises LIP_RELIEF over LIP_RELIEF_L."""
    m = from_shapely(lips_2d(j), lip_z(j), T + j + 0.01)
    x0, zt = frame(S_NARROW)[0], T + j
    m = m - prism_uz([(x0 - 5, -5), (x0 + zt + 5, -5), (x0 - 5, zt + 5)], -40, 40)          # entry ramp
    lz, r = lip_z(j), LIP_RELIEF
    e = x0 + LIP_RELIEF_L
    return m - prism_uz([(x0 - 5, lz - 1), (e + r, lz - 1), (e + r, lz), (e, lz + r), (x0 - 5, lz + r)], -40, 40)


def bec(j: float) -> Manifold:
    """Mouth lip: proud of each plate by BEC_T over BEC_L then a 45 deg slope.
    Gives the standing print a wide first layer and a thumb stop."""
    out = slider_outline(j)
    zt = T + j + PLATE_T
    sec = [(X_END - BEC_L - BEC_T, 0.0), (X_END - BEC_L, BEC_T), (X_END, BEC_T), (X_END, 0.0)]
    m = Manifold()
    for z0, sg in ((zt - 0.01, 1), (-j - PLATE_T + 0.01, -1)):
        prof = prism_uz([(u, z0 + sg * d) for u, d in sec], -40, 40)
        m = m + (prof ^ from_shapely(out, -20, 20))
    return m


def slider_parts(j: float, marks: int = 2) -> dict:
    """Slider solids: two plates, rib (in the root groove), post, lips, mouth lip,
    and the play notches. Slider frame: x along the path, teeth engage at x = 0."""
    mir = lambda g: affinity.scale(g, 1, -1, origin=(0, 0))  # noqa: E731
    x0, x1 = frame(S_NARROW)[0], frame(S_MOUTH)[0]
    lane = band(-SL_OUT, EDGE + 0.1, S_NARROW - 1, S_MOUTH + 3)
    outline = unary_union([lane, mir(lane)]).convex_hull.intersection(sbox(x0, -40, X_END, 40))
    clip = sbox(x0, -40, x1, 40)
    zt = T + j
    r1 = slider_rib_2d(j).intersection(clip)
    ribs = from_shapely(r1, zt - GROOVE_D, zt + 0.01) + from_shapely(mir(r1), zt - GROOVE_D, zt + 0.01)
    xa, xb = frame(NOTCH_S[0])[0], frame(NOTCH_S[1])[0]
    zr = zt - GROOVE_D
    ribs = ribs - prism_uz([(xa, zr - 0.01), (xa + NOTCH_D, zr + NOTCH_D),
                            (xb - NOTCH_D, zr + NOTCH_D), (xb, zr - 0.01)], 0.0, 20.0)
    crans = Manifold()
    for i in range(marks):       # play marks: 0.6 notches in the top plate face
        crans = crans + box(x0 + 1.0 + 2.0 * i, -1.5, zt + PLATE_T - MARK_D, x0 + 2.0 + 2.0 * i, 1.5, zt + PLATE_T + 1)
    return {"top_plate": from_shapely(outline, zt, zt + PLATE_T),
            "bottom_plate": from_shapely(outline, -j - PLATE_T, -j),
            "rib": ribs, "post": post_solid(j), "lips": lips(j), "mouth_lip": bec(j), "marks": crans}


def letter_n_2d():
    """Letter N: 2*ax tall (along u) by 2*ay wide, stroke 0.9. Local up lands on
    -u (bottom of the standing print), local right on +v."""
    ax, ay, w = LET["ax"], LET["ay"], LET["stroke"]
    hh, hw = ax - w / 2, ay - w / 2
    g = unary_union([LineString([(-hw, -hh), (-hw, hh)]).buffer(w / 2),
                     LineString([(hw, -hh), (hw, hh)]).buffer(w / 2),
                     LineString([(-hw, hh), (hw, -hh)]).buffer(w / 2)])
    return affinity.translate(affinity.rotate(g, 90, origin=(0, 0)), LET["cx"], LET["cy"])


def letter_n_cut(j: float) -> Manifold:
    """The letter, cut LET['depth'] into the top plate outer face."""
    return from_shapely(letter_n_2d(), T + j + PLATE_T - LET["depth"], T + j + PLATE_T + 1.0)


def slider_n(j: float = J) -> Manifold:
    """Slider N in its own frame (teeth engage at x = 0), letter engraved."""
    p = slider_parts(j)
    body = union([p[k] for k in ("top_plate", "bottom_plate", "rib", "post", "lips", "mouth_lip")]) - p["marks"]
    return body - letter_n_cut(j)


def ears(j: float = J) -> Manifold:
    """Print tabs: two discs per plate corner, joined by a web, in the slider frame."""
    out = slider_outline(j)
    x_hi = X_END
    zt = T + j
    z_out_hi = zt + PLATE_T + BEC_T         # outer edge incl. mouth lip (solid over BEC_L)
    z_out_lo = -j - PLATE_T - BEC_T
    ymax = out.intersection(sbox(x_hi - 0.05, -40, x_hi, 40)).bounds[3]
    ear = Manifold()
    for z_out, sz in ((z_out_hi, 1.0), (z_out_lo, -1.0)):
        for sy in (1.0, -1.0):
            cz = z_out + sz * (EAR_GAP + EAR_D / 2)
            cy = sy * (ymax - EAR_Y_IN)
            disk = Manifold.cylinder(EAR_T, EAR_D / 2, EAR_D / 2, 72).rotate([0, 90, 0]).translate([x_hi - EAR_T, cy, cz])
            za, zb = sorted((z_out - sz * 0.3, z_out + sz * (EAR_GAP + 0.3)))   # web bites 0.3 into edge and disc
            neck = box(x_hi - EAR_T, cy - EAR_NECK / 2, za, x_hi, cy + EAR_NECK / 2, zb)
            ear = ear + disk + neck
    return ear


# ================================================================ socket (base) and pin
def t_profile():
    """(v, z) section of the T-bar pin: stem from bed to WEB_Z, bar above."""
    (w0, w1), (b0, b1), (zc, zt) = WEB_V, BAR_V, BAR_Z
    return Polygon([(w0, 0.0), (w1, 0.0), (w1, WEB_Z), (b1, zc), (b1, zt), (b0, zt), (b0, zc), (w0, WEB_Z)])


def pin_t() -> Manifold:
    """Row-2 T-bar pin: same section, from its back stop to PIN_U[1]."""
    return prism_vz(list(t_profile().exterior.coords[:-1]), PIN_U0, PIN_U[1])


def pin_fill() -> Manifold:
    """TPU under the pin bar (outside the socket) tying the pin to the tape."""
    (w0, w1), (b0, b1), (zc, zt) = WEB_V, BAR_V, BAR_Z
    return prism_vz([(w1, 0.0), (b1, 0.0), (b1, zc), (w1, WEB_Z)], FILL_U0, PIN_U[1])


def butt() -> Manifold:
    """Row-1 buttress: mirror image of the pin bar section, socket face to BUTT_U1."""
    return box(U_FACE - 0.05, -BAR_V[1], 0, BUTT_U1, -BAR_V[0], BAR_Z[1])


def t_channel_roof(fit, lead: float = 0.0, apex_cap: float | None = None):
    """Socket counter-profile: walls +`jp` in plan, 45 deg lips lowered by `g45`
    vertically, roof as a double 45 deg slope `gt` above the bar edges (no flat
    bridge). `lead` widens everything (entry steps); `apex_cap` bounds the peak."""
    jp, g45, gt = (f + lead for f in fit)
    (w0, w1), (b0, b1), (zc, zt) = WEB_V, BAR_V, BAR_Z
    zl = WEB_Z + jp - g45
    zh = zc + jp - g45
    z_eave = zt + gt
    half = (b1 - b0) / 2 + jp
    vm = (b0 + b1) / 2
    poly = Polygon([(w0 - jp, -1.0), (w1 + jp, -1.0), (w1 + jp, zl), (b1 + jp, zh), (b1 + jp, z_eave), (vm, z_eave + half),
                    (b0 - jp, z_eave), (b0 - jp, zh), (w0 - jp, zl)])
    if apex_cap is not None:
        poly = poly.intersection(sbox(-50, -5, 50, apex_cap))
    return list(poly.exterior.coords[:-1])


def apex(fit) -> float:
    (b0, b1), zt = BAR_V, BAR_Z[1]
    return zt + fit[2] + (b1 - b0) / 2 + fit[0]


def letter_b_2d(name: str, cu: float, cv: float):
    """Base letter (LET_B): read along u, top towards +u. Pure rotation, no mirror."""
    hh, hw, w = LET_B["h"] / 2 - LET_B["stroke"] / 2, LET_B["w"] / 2 - LET_B["stroke"] / 2, LET_B["stroke"]
    r = hh / 2

    def bowl(yc):
        cx = hw - r
        arc = [(cx + r * math.sin(t), yc + r * math.cos(t)) for t in np.linspace(0, math.pi, 25)]
        return LineString([(-hw, yc + r)] + arc + [(-hw, yc - r)])

    g = unary_union([bowl(r).buffer(w / 2), bowl(-r).buffer(w / 2), LineString([(-hw, -hh), (-hw, hh)]).buffer(w / 2)])
    g = affinity.affine_transform(g, [0, 1, -1, 0, 0, 0])
    return affinity.translate(g, cu, cv)


def box_new(fit=PIN_FIT, u0: float = U_BOX[0], vlo: float = -EDGE, letter: str | None = "B") -> Manifold:
    """Socket: PLA block with the T channel and a 45 deg roof, from the back
    stop to its face, entry widened in steps; letter engraved on the top face."""
    ub = U_BOX[1]
    cap = apex(fit)
    m = box(u0, vlo, 0, ub, BOX_V1, BOX_T) - prism_vz(t_channel_roof(fit), PIN_U0, ub + 1)
    for k in range(LEAD_N):
        d = LEAD * (LEAD_N - k) / LEAD_N
        m = m - prism_vz(t_channel_roof(fit, d, cap), ub - LEAD * (k + 1) / LEAD_N, ub + 1)
    if letter:
        cu, cv = (u0 + ub) / 2, vlo + 1.1 + LET_B["w"] / 2
        m = m - from_shapely(letter_b_2d(letter, cu, cv), BOX_T - LET_B["depth"], BOX_T + 1)
    return m


def veil(u0: float, u1: float, us: list[float]) -> Manifold:
    """Thin TPU veil between roots (no tooth printed on TPU): from the tape edge
    to the root tips, z 0 to 0.4, subtracting the root footprints."""
    v = box(u0, -EDGE - 0.01, 0, u1, -ROOT_V, 0.4)
    return v - union([box(u - ROOT_H, -EDGE - 0.02, -0.1, u + ROOT_H, -ROOT_V + 0.01, 0.5) for u in us])


# ================================================================ stops
def stop_block(low: bool) -> Manifold:
    """Top-of-chain stops, row-1 convention, stop face through u = 0 at the tape
    edge, biased by THETA so they sit square in the slider mouth.
    low = False: rigid PLA stop (row 2), full corridor, covers the post face.
    low = True: soft TPU end-stop (row 1), in the root corridor only, with a
    ramp from above so the rib crushes it once, on threading."""
    reach = ARRET_REACH if low else STOP_REACH
    k = math.tan(THETA) * reach
    g = Polygon([(0, -EDGE), (-k, -EDGE + reach), (-k + STOP_LEN, -EDGE + reach), (STOP_LEN, -EDGE)])
    if not low:
        return from_shapely(g, 0, T)
    ramp = prism_uz([(-5, -1), (-5, ARRET_T), (0.8, ARRET_T), (STOP_LEN, PIN_T), (STOP_LEN + 5, PIN_T), (STOP_LEN + 5, -1)], -20, 20)
    return from_shapely(g, 0, ARRET_T) ^ ramp


# ================================================================ chain
def chain(u_last2: float, u_stop2: float, u_stop1: float, u_end: float, n_teeth: int) -> dict:
    """Both rows, usage pose (closed), each split into its PLA and TPU solids.
    Row 1: teeth, socket, buttress / tape, anchors, veil, soft end-stop.
    Row 2: teeth, pin, rigid stop / tape, anchors, veil, pin fill."""
    t = tooth_pla()
    u1s, u2s = row_u(1, n_teeth), row_u(2, n_teeth)
    cf = butt()
    tpu1 = union([box(U_FACE, -EDGE - TAPE_W, 0, u_end, -EDGE, TAPE_B)] + [bulb(u) for u in u1s]
                 + [bulb(u) for u in BULBS1] + [veil(BUTT_U1, u_end, u1s)])
    pla1 = clean(union([t.translate([u, 0, 0]) for u in u1s] + [box_new(), cf]) - tpu1)
    tpu2 = union([box(U_FACE, -EDGE - TAPE_W, 0, u_end, -EDGE, TAPE_B)] + [bulb(u) for u in u2s]
                 + [bulb(u) for u in (1.0, 3.4)] + [veil(PIN_U[1], u_end, u2s)])
    tpu2 = tpu2.mirror([0, 1, 0]) + pin_fill()
    pla2 = clean(union([flip(t).translate([u, 0, 0]) for u in u2s] + [pin_t()]) - tpu2)
    # upper stops: rigid PLA on row 2, soft TPU on row 1
    s2 = stop_block(low=False).translate([u_stop2, 0, 0]).mirror([0, 1, 0])
    tpu1 = tpu1 + stop_block(low=True).translate([u_stop1, 0, 0])
    tpu2 = (tpu2 - s2) + bulb(u_stop2 + 1.5).mirror([0, 1, 0])
    pla1 = clean(pla1 - tpu1)
    pla2 = clean((pla2 + s2) - tpu2)
    return {"chain-row1-pla": pla1, "chain-row1-tpu": tpu1, "chain-row2-pla": pla2, "chain-row2-tpu": tpu2}


# ================================================================ poses
def pose_t(m: Manifold) -> list[float]:
    """Translation applied after R_SL so the standing slider is centred in x/y
    and sits on the bed."""
    st = m.transform(R_SL)
    b = st.bounding_box()
    return [-(b[0] + b[3]) / 2, -(b[1] + b[4]) / 2, -b[2]]


def to_print(m: Manifold, ref: Manifold) -> Manifold:
    """Slider frame -> bed pose (standing on its mouth, centred, at z = 0)."""
    return m.transform(R_SL).translate(pose_t(ref))


CHAIN_PARTS = ["chain-row1-pla", "chain-row1-tpu", "chain-row2-pla", "chain-row2-tpu"]


def bed_pose(chain_parts: dict, slider: Manifold) -> tuple[dict, list]:
    """Everything in bed poses: rows flat, row 2 out of the socket and apart,
    slider standing next to the chain with its tabs."""
    posed, shifts = {}, {}
    ch = {k: (m.translate([PIN_OUT, GAP_OPEN, 0]) if "row2" in k else m) for k, m in chain_parts.items()}
    bbs = [m.bounding_box() for m in ch.values()]
    lo = [min(b[i] for b in bbs) for i in range(3)]
    hi = [max(b[i + 3] for b in bbs) for i in range(3)]
    for k, m in ch.items():
        posed[k] = m.translate([-lo[0], -lo[1], -lo[2]])
    ref = slider + ears()
    pm = to_print(ref, ref)
    b = pm.bounding_box()
    sh = [0.0 - b[0], hi[1] - lo[1] + 12.0 - b[1], -b[2]]
    posed["slider-n"] = pm.translate(sh)
    return posed, sh


# ================================================================ STL output
def write_stl(m: Manifold, path):
    """Binary STL, float32 records."""
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3].astype("f8")
    f = np.asarray(mesh.tri_verts).astype("i8")
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as fh:
        fh.write(b"pla-tpu-zipper".ljust(80, b" "))
        fh.write(np.uint32(len(f)).tobytes())
        fh.write(rec.tobytes())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--teeth", type=int, default=18, help="teeth per row (default 18)")
    ap.add_argument("--out", default="stl", help="output directory (default ./stl)")
    args = ap.parse_args()
    n = args.teeth

    u_last2 = U1 + PITCH / 2 + PITCH * (n - 1)   # last row-2 tooth
    u_stop2 = u_last2 + 6.0                      # rigid PLA stop: 6.0 past it
    u_stop1 = u_stop2 + 0.5                      # soft TPU end-stop: 0.5 past that
    u_end = u_stop1 + 3.0                        # tape ends: 3.0 past the end-stop

    ch = chain(u_last2, u_stop2, u_stop1, u_end, n)
    sl = slider_n()
    bed, _ = bed_pose(ch, sl)

    from pathlib import Path
    out = Path(args.out)
    for k in CHAIN_PARTS:
        write_stl(ch[k], out / f"usage-{k}.stl")
        write_stl(bed[k], out / f"print-{k}.stl")
    write_stl(sl.translate([P_USAGE, 0, 0]), out / "usage-slider-n.stl")
    write_stl(bed["slider-n"], out / "print-slider-n.stl")
    print(f"{2 * (len(CHAIN_PARTS) + 1)} STL -> {out}/")


if __name__ == "__main__":
    main()

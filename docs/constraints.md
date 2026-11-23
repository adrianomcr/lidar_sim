# Plane constraints

Primitives are infinite (planes, cylinders) or closed (spheres, ellipsoids). Plane constraints cut
them into finite or open pieces: a tube, a square panel, a dome, an engine intake.

## Definition

A constraint is a plane given by a normal $n$ (any non-zero length) and a point $p_0$ on it. It
keeps the half-space the normal points to:

$$
\text{keep } p \iff \hat n \cdot (p - p_0) > 0, \qquad \hat n = \frac{n}{\lVert n \rVert}.
$$

$h(p) = \hat n \cdot (p - p_0)$ is the signed distance from $p$ to the cutting plane, so a
point is kept when it lies strictly on the positive side. The constructor of `PlaneConstraint`
stores $\hat n$ and the offset $\delta = \hat n \cdot p_0$, and the test becomes a single dot
product:

$$
\hat n \cdot p - \delta > 0.
$$

- The inequality is strict: points exactly on the cutting plane are removed.
- A zero normal is rejected with an exception when the world is loaded.

## Frame

Constraints are written in the **local frame of their primitive**: $n$ and $p_0$ are relative to
the primitive's `center` and rotate with its `angles`. Moving or rotating an object in the world
file moves its cuts with it.

To cut along a plane known in world coordinates (normal $n_w$ through $p_w$), convert it with the
primitive's pose:

$$
n = R^\top n_w, \qquad p_0 = R^\top (p_w - c).
$$

## Several constraints

All constraints of a primitive must hold. The kept region is the intersection of half-spaces, a
convex (possibly unbounded) polyhedron:

$$
K = \bigcap_i \{\, p : \hat n_i \cdot (p - p_{0,i}) > 0 \,\}.
$$

The visible surface is the part of the primitive inside $K$. A non-convex region, like a wall with
a window, cannot be expressed by one primitive. Use several copies of the same primitive, each
with a convex set of constraints; the world draws their union (see the recipes below).

## Effect on visibility

The constraints are tested on every candidate hit, in order of distance (see
[Ray casting](ray_casting.md#selecting-the-visible-hit)). When the near intersection of a ray is
cut away, the far one is tested next. Because primitives are surfaces, cutting a closed shape
opens it: the sensor sees the inside through the opening.

The cost is one dot product per constraint per candidate hit, so constraints are cheap.

## Examples from the example scene

All of these are in `config/world.yaml` (built by `scripts/create_world.py`).

| Object | Constraints (local frame) | Kept region | Result |
| --- | --- | --- | --- |
| Airplane engines (ellipsoids) | $n = (-1, 0, 0)$, $p_0 = 0$ | $x < 0$ | Rear half of the ellipsoid; the opening faces the nose (+x) like an intake |
| Main wings (flat ellipsoids, yawed by ±0.3 rad) | $n = (0, \pm 1, 0)$, $p_0 = 0$ | $\pm y > 0$ | One half per side, cut at the fuselage centerline, forming swept wings |
| Vertical tail (ellipsoid, $s_z = 8$) | $n = (0, 0, -1)$, $p_0 = (0, 0, 6.5)$ | $z < 6.5$ | Flat top on the tail fin |
| Square panel (plane) | $n = (0, \mp 1, 0)$, $p_0 = (0, \pm 1, 0)$; $n = (\mp 1, 0, 0)$, $p_0 = (\pm 1, 0, 0)$ | $\lvert x \rvert < 1$, $\lvert y \rvert < 1$ | 2 m × 2 m square |
| Tube (cylinder, $\rho = 0.5$) | $z > -1$, $z < 5$, and $n = (0, 1, -0.2)$, $p_0 = (0, -0.25, 0)$ | $-1 < z < 5$ and $y > 0.2 z - 0.25$ | Finite tube sliced obliquely, so part of the wall is missing and the inside is visible |
| Dome (ellipsoid) | $n = (-0.4, 0, 1)$, $p_0 = 0$ | $z > 0.4 x$ | Half shell cut by a tilted plane through the center |
| Cap (sphere) | $n = (0, 0.4, 1)$, $p_0 = 0$ | $0.4 y + z > 0$ | Hemisphere bounded by a tilted plane |

## Recipes

Snippets for the `plane_constraints` list of an object in the world file. Replace $L$, $W$, $H$,
$\rho$ and $h$ with numbers: the world file does not evaluate expressions.

**Finite tube of length $L$** (cylinder, centered on `center`):

```yaml
plane_constraints:
- {normal: [0, 0, 1], point: [0, 0, -L/2]}   # z > -L/2
- {normal: [0, 0, -1], point: [0, 0, L/2]}   # z < L/2
```

**Rectangular panel $W \times H$** (plane, centered on `center`):

```yaml
plane_constraints:
- {normal: [1, 0, 0], point: [-W/2, 0, 0]}   # x > -W/2
- {normal: [-1, 0, 0], point: [W/2, 0, 0]}   # x < W/2
- {normal: [0, 1, 0], point: [0, -H/2, 0]}   # y > -H/2
- {normal: [0, -1, 0], point: [0, H/2, 0]}   # y < H/2
```

**Dome or hemisphere** (sphere or ellipsoid): `{normal: [0, 0, 1], point: [0, 0, 0]}` keeps
$z > 0$.

**Spherical cap of height $h$** (sphere of radius $\rho$): `{normal: [0, 0, 1], point: [0, 0, ρ - h]}`.

**Half pipe along the local z axis** (cylinder): `{normal: [0, -1, 0], point: [0, 0, 0]}` keeps
the $y < 0$ half.

**Closing the end of a tube.** Constraints only remove surface. To close a tube of length $L$, add
a plane with the same `angles`, centered at the end of the tube
($c + R\,(0, 0, L/2)$ in world coordinates), and bound it to a regular $N$-gon of inradius $\rho$.
For $k = 0, \dots, N-1$ and $\theta_k = 2\pi k / N$:

```yaml
- {normal: [-cos θk, -sin θk, 0], point: [ρ cos θk, ρ sin θk, 0]}   # cos θk x + sin θk y < ρ
```

A disk can only be approximated this way, because constraints are linear. A flattened ellipsoid
is an alternative end cap.

**Wall with a rectangular window.** The wall minus the window is not convex, so split it into
four strips, each a copy of the wall plane with its own constraints. For a window
$|x| < a$, $|y| < b$ in the wall's local frame:

| Copy | Constraints |
| --- | --- |
| Left | $x < -a$ |
| Right | $x > a$ |
| Bottom | $-a < x < a$, $y < -b$ |
| Top | $-a < x < a$, $y > b$ |

Generate repetitive cases like these in `scripts/create_world.py` with a loop rather than by hand.

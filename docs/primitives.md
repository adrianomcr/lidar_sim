# Geometric primitives

Each primitive is a surface defined in its own local frame (see [Conventions](conventions.md)).
`Primitive::intersect()` hands every subclass the ray already expressed in that frame,

```math
r'(t) = o' + t\, d', \qquad \lVert d' \rVert = 1,
```

and the subclass returns the values of $t$ where the ray meets its surface, in ascending order.
Discarding hits behind the sensor and applying constraints is done by the base class
([Ray casting](ray_casting.md)). Primes, which mark local-frame quantities, are dropped below to
keep the formulas short.

| Primitive | Local surface | World file fields | Source |
| --- | --- | --- | --- |
| Plane | $z = 0$ | `center`, `angles` | `geometry/plane.cpp` |
| Sphere | $x^2 + y^2 + z^2 = \rho^2$ | `center`, `radius`, optional `angles` | `geometry/sphere.cpp` |
| Cylinder | $x^2 + y^2 = \rho^2$ | `center`, `angles`, `radius` | `geometry/cylinder.cpp` |
| Cone | $x^2 + y^2 = k^2 z^2$ | `center`, `angles`, `slope` | `geometry/cone.cpp` |
| Ellipsoid | $\frac{x^2}{s_x^2} + \frac{y^2}{s_y^2} + \frac{z^2}{s_z^2} = 1$ | `center`, `size`, `angles` | `geometry/ellipsoid.cpp` |

Planes, cylinders and cones are infinite. Use [plane constraints](constraints.md) to bound them.

## Plane

**Surface.** The plane $z = 0$ of the local frame. Its normal is the local z axis, which in the
world is $n = R\thinspace\hat z$, the third column of $R$. In world coordinates the plane is
$\lbrace p : n \cdot (p - c) = 0 \rbrace$.

**Intersection.** The z component of the ray must vanish:

```math
o_z + t\, d_z = 0 \quad\Rightarrow\quad t = -\frac{o_z}{d_z}.
```

If $|d_z| < 10^{-12}$ the ray is parallel to the plane and there is no hit. The same distance
written in world coordinates is $t = \dfrac{n \cdot (c - o)}{n \cdot d}$.

**Notes.**

- The plane is visible from both sides; the sign of the normal does not matter for the
  intersection, only for the constraints.
- The local x and y axes (first two columns of $R$) span the plane. Constraints on $x$ and $y$
  turn it into a finite polygon, for example the 2 m × 2 m square of the example scene
  (see [constraints.md](constraints.md#recipes)).

## Sphere

**Surface.** All points at distance $\rho$ (`radius`) from the local origin: $\lVert p \rVert^2 = \rho^2$.

**Intersection.** Substituting the ray:

```math
\lVert o + t\,d \rVert^2 = \rho^2
\quad\Rightarrow\quad
\underbrace{(d \cdot d)}_{a}\, t^2 + \underbrace{2\,(o \cdot d)}_{b}\, t + \underbrace{(o \cdot o - \rho^2)}_{c} = 0.
```

With a unit direction $a = 1$. The discriminant

```math
\Delta = b^2 - 4ac = 4\left[\rho^2 - \left(\lVert o \rVert^2 - (o \cdot d)^2\right)\right]
```

compares $\rho^2$ with the squared distance from the center to the ray's line, so $\Delta \ge 0$
exactly when the line passes within $\rho$ of the center. When the sensor is inside the sphere,
$c < 0$, the roots have opposite signs, and the visible hit is the inner wall at $t_2$.

**Notes.** Rotating a sphere does not change its shape. `angles` only matter when the sphere has
constraints, because they rotate the cut with it.

## Cylinder

**Surface.** An infinite cylinder of radius $\rho$ whose axis is the local z axis:
$x^2 + y^2 = \rho^2$, for any $z$. In the world the axis passes through `center` along $R\thinspace\hat z$.

**Intersection.** Only the components perpendicular to the axis matter. With
$o_\perp = (o_x, o_y)$ and $d_\perp = (d_x, d_y)$:

```math
\lVert o_\perp + t\, d_\perp \rVert^2 = \rho^2
\quad\Rightarrow\quad
a = \lVert d_\perp \rVert^2,\quad b = 2\, o_\perp \cdot d_\perp,\quad c = \lVert o_\perp \rVert^2 - \rho^2.
```

$a = 1 - d_z^2 = \sin^2\beta$, where $\beta$ is the angle between the ray and the axis. A ray
parallel to the axis ($a < 10^{-12}$) never meets the surface and returns no hit.

The equivalent world-frame form, with axis direction $u = R\thinspace\hat z$ and
$w = (o - c) - ((o - c)\cdot u)\thinspace u$, is $a = 1 - (d \cdot u)^2$,
$b = 2\thinspace(d \cdot w - (d \cdot u)(u \cdot w))$, $c = \lVert w \rVert^2 - \rho^2$. Working in the
local frame avoids these projections.

**Notes.**

- Two constraints, $z > z_{\min}$ and $z < z_{\max}$, make a finite tube. The tube stays open:
  constraints remove surface, they do not add end caps. A slanted constraint gives a beveled end.
- The structural beams of the example hangar are unconstrained cylinders of radius 0.4 m. They
  are infinite, but they run through the walls, floor or roof, which hide everything outside the
  hangar, so they look finite from inside.

## Cone

**Surface.** An infinite double cone with its apex at the local origin and its axis along the
local z axis. With $k$ the `slope`:

```math
x^2 + y^2 - k^2 z^2 = 0.
```

The radius at height $z$ is $k\lvert z \rvert$, so $\tan\alpha = k$ where $\alpha$ is the angle
between the axis and a straight line on the surface. Both nappes are present: $z > 0$ and
$z < 0$. In the world the axis passes through `center` along $R\thinspace\hat z$. `slope` must be
positive; its sign would not change the surface.

**Intersection.** Substituting the ray, with $o_\perp = (o_x, o_y)$ and $d_\perp = (d_x, d_y)$:

```math
a = \lVert d_\perp \rVert^2 - k^2 d_z^2,\quad
b = 2\left(o_\perp \cdot d_\perp - k^2 o_z d_z\right),\quad
c = \lVert o_\perp \rVert^2 - k^2 o_z^2.
```

$a$ is the cylinder coefficient with the axial part subtracted. For a unit direction it is
$\sin^2\beta - k^2\cos^2\beta$, where $\beta$ is the angle between the ray and the axis, so
$a > 0$ when that angle is wider than the cone. The roots come from `solveQuadratic()`. A ray
parallel to a generatrix has $a = 0$ (the same $10^{-12}$ test). It then meets the cone at most
once, at $t = -c / b$ when $b$ is not zero. If $a$ and $b$ both vanish, the ray lies on the cone
or misses it, and there is no single hit.

**Notes.**

- A constraint $z > 0$ keeps the upper nappe. Adding $z < H$ makes a finite cone of height $H$
  whose base radius is $k H$. Two cuts on the same side of the apex make a frustum. Constraints
  remove surface, so the base stays open.
- The example scene has one of these next to the other test shapes: apex at $(-8, 22, 2)$,
  `slope` $0.5$, kept where $z > 0$ and $z < 4$.
- A ray aimed at the apex is an ordinary root of the quadratic. Along the axis it is a double
  root at the apex.

## Ellipsoid

**Surface.** An ellipsoid centered at the local origin with semi-axes $s = (s_x, s_y, s_z)$
(`size`, half the extent along each local axis). With $D = \mathrm{diag}(s_x^{-2}, s_y^{-2}, s_z^{-2})$:

```math
p^\top D\, p = 1.
```

**Intersection.** Substituting the ray:

```math
(o + t\,d)^\top D\,(o + t\,d) = 1
\quad\Rightarrow\quad
a = d^\top D\, d,\quad b = 2\, o^\top D\, d,\quad c = o^\top D\, o - 1.
```

$D$ is diagonal, so these are three weighted dot products. The constructor stores $D$ and rejects
non-positive semi-axes. Since $D$ is positive definite, $a > 0$ for any direction.

**Interpretation.** Scaling space by $S^{-1} = \mathrm{diag}(1/s_x, 1/s_y, 1/s_z)$ maps the
ellipsoid onto the unit sphere and the ray onto another line with the same parameter $t$ but a
direction $S^{-1} d$ that is no longer unit length, which is why $a \ne 1$ here.

**Notes.**

- A sphere is the ellipsoid with $s_x = s_y = s_z = \rho$.
- Strongly flattened ellipsoids make good wings and plates: the tail wing of the example airplane
  has semi-axes $(4, 8, 0.6)$.
- `size` holds semi-axes, not full dimensions: the fuselage `size: [28, 4, 4]` is 56 m long and
  8 m in diameter.

## Summary of the quadratic coefficients

| Primitive | $a$ | $b$ | $c$ |
| --- | --- | --- | --- |
| Sphere | $d \cdot d$ | $2\thinspace o \cdot d$ | $o \cdot o - \rho^2$ |
| Cylinder | $d_\perp \cdot d_\perp$ | $2\thinspace o_\perp \cdot d_\perp$ | $o_\perp \cdot o_\perp - \rho^2$ |
| Cone | $\lVert d_\perp \rVert^2 - k^2 d_z^2$ | $2(o_\perp \cdot d_\perp - k^2 o_z d_z)$ | $\lVert o_\perp \rVert^2 - k^2 o_z^2$ |
| Ellipsoid | $d^\top D\thinspace d$ | $2\thinspace o^\top D\thinspace d$ | $o^\top D\thinspace o - 1$ |

The roots are $t_{1,2} = \dfrac{-b \mp \sqrt{b^2 - 4ac}}{2a}$, returned with $t_1 \le t_2$.

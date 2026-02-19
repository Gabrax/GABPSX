<div align="center">

# Physics Demonstration in 2D Games

</div>

In MENU move around with arrow, press ENTER to select, press BACKSPACE to return

## Getting started
> [!WARNING]
> To run this project, CMake needs to be installed <br>
> On Windows use Git interpreter to run script<br>

```bash
git clone https://github.com/Gabrax/2DPhysicsExamples.git
```

```bash
sh build.sh
```

---
## Coloring Polygon
https://github.com/user-attachments/assets/1f3f6bb5-1ec1-4e1b-9615-04a49c23a109

### Circle-Polygon Edge
The signed distance from the circle center to the polygon edge is calculated using the normal:

$$
d = \mathbf{n} \cdot (\mathbf{p_c} - \mathbf{p_s})
$$

where:
- $$\( \mathbf{n} \)$$ is the normalized edge normal,
- $$\( \mathbf{p_c} \)$$ is the circle center,
- $$\( \mathbf{p_s} \)$$ is the edge start point.

### Collision Response (Elastic Reflection)
The velocity after collision is computed using:

$$
\mathbf{v'} = \mathbf{v} - 2 (\mathbf{v} \cdot \mathbf{n}) \mathbf{n}
$$

where:
- $$\( \mathbf{v} \)$$ is the original velocity,
- $$\( \mathbf{n} \)$$ is the collision normal.

### Polygon Rotation (2D Rotation Formula)
Each vertex is rotated around the polygon's center using:

$$
x' = x \cos(\theta) - y \sin(\theta)
$$
$$
y' = x \sin(\theta) + y \cos(\theta)
$$

where:
- $$\( (x, y) \)$$ is the original vertex position relative to the center,
- $$\( (x', y') \)$$ is the rotated position,
- $$\( \theta \)$$ is the rotation angle.

### Polygon Area (Shoelace Formula)
The area of a polygon is computed as:

$$
A = \frac{1}{2} \left| \sum_{i=0}^{n-1} (x_i y_{i+1} - y_i x_{i+1}) \right|
$$

where:
- $$\( (x_i, y_i) \)$$ are the polygon vertices, and
- the last vertex wraps around with $$\( x_n = x_0, y_n = y_0 \)$$.

### Point-In-Polygon (Ray-Casting Algorithm)
A point is inside a polygon if the number of times a ray from the point crosses an edge is odd:

$$
\text{inside} = (\text{crossings} \mod 2) = 1
$$

where "crossings" is the number of times a horizontal ray from the point intersects the polygon edges.

### Motion Update (Euler Integration)
The position update per frame is:

$$
\mathbf{p} = \mathbf{p} + \mathbf{v} \cdot dt
$$

where:
- $$\( \mathbf{p} \)$$ is the position,
- $$\( \mathbf{v} \)$$ is the velocity,
- $$\( dt \)$$ is the time step (assumed $$\( dt = 1 \)$$ per frame).

### Pixel-Based Area Calculation (Image Sampling)
The proportion of colored pixels in an image is:

$$
\text{colored percentage} = \left( \frac{\text{colored pixels}}{\text{polygon pixels}} \right) \times 100
$$

where "colored pixels" are those matching a target color (e.g., yellow).

---

## Collision Detection
https://github.com/user-attachments/assets/5e854fc3-9daa-48ff-b31f-2f47f369edc2

### Quadratic Equation
To determine if two moving circles will collide within the next frame, we solve the quadratic equation:

$$
a t^2 + b t + c = 0
$$

where:

$$
a = \mathbf{v_r} \cdot \mathbf{v_r}
$$

$$
b = 2 (\mathbf{p_r} \cdot \mathbf{v_r})
$$

$$
c = (\mathbf{p_r} \cdot \mathbf{p_r}) - (4 r^2)
$$

where:
- $$\( \mathbf{v_r} = \mathbf{v_2} - \mathbf{v_1} \)$$ is the relative velocity,
- $$\( \mathbf{p_r} = \mathbf{p_2} - \mathbf{p_1} \)$$ is the relative position,
- $$\( r \)$$ is the radius of each circle.

The discriminant:

$$
D = b^2 - 4ac
$$

If $$\( D \geq 0 \)$$, then a collision occurs at:

$$
t = \frac{-b \pm \sqrt{D}}{2a}
$$

### Elastic Collision Response
Once a collision is detected, the new velocities are calculated using the **1D elastic collision equations** along the collision normal:

$$
v_1' = \frac{v_1 (m_1 - m_2) + 2 m_2 v_2}{m_1 + m_2}
$$

$$
v_2' = \frac{v_2 (m_2 - m_1) + 2 m_1 v_1}{m_1 + m_2}
$$

where:
- $$\( v_1 \)$$ and $$\( v_2 \)$$ are initial velocities along the collision normal,
- $$\( v_1' \)$$ and $$\( v_2' \)$$ are the new velocities,
- $$\( m_1 \)$$ and $$\( m_2 \)$$ are the masses.

### Position Update (Kinematics)
Before and after the collision, positions are updated using:

$$
\mathbf{p} = \mathbf{p} + \mathbf{v} \cdot dt
$$

where:
- $$\( \mathbf{p} \)$$ is the position,
- $$\( \mathbf{v} \)$$ is the velocity,
- $$\( dt \)$$ is the frame time.

  ### Position Correction (Overlap Resolution)
To prevent objects from sticking together due to numerical errors, we apply position correction:

$$
\text{overlap} = (2r) - \text{distance}(\mathbf{p_1}, \mathbf{p_2})
$$

Each object is moved **half the overlap** along the collision normal:

$$
\mathbf{p_1} = \mathbf{p_1} - \mathbf{n} \cdot \frac{\text{overlap}}{2}
$$

$$
\mathbf{p_2} = \mathbf{p_2} + \mathbf{n} \cdot \frac{\text{overlap}}{2}
$$

where $$\( \mathbf{n} \)$$ is the normalized collision normal.

---
## Central Force Fields

https://github.com/user-attachments/assets/40cb0256-9d07-4a64-b05a-5697e693f866

### Gravitational Force (Newton's Law of Universal Gravitation)
The force between two point masses is given by:

$$
\mathbf{F} = G \frac{m_1 m_2}{r^2} \hat{\mathbf{r}}
$$

where:
- $$\( G \)$$ is the gravitational constant,
- $$\( m_1, m_2 \)$$ are the masses,
- $$\( r \)$$ is the distance between the two masses,
- $$\( \hat{\mathbf{r}} \)$$ is the unit vector pointing from one mass to the other.


### Euler Integration (Velocity and Position Update)
The position and velocity updates using Euler's method:

$$
\mathbf{a} = \frac{\mathbf{F}}{m}
$$

$$
\mathbf{v} = \mathbf{v} + \mathbf{a} \cdot dt
$$

$$
\mathbf{p} = \mathbf{p} + \mathbf{v} \cdot dt
$$

where:
- $$\( \mathbf{a} \)$$ is acceleration,
- $$\( \mathbf{v} \)$$ is velocity,
- $$\( \mathbf{p} \)$$ is position,
- $$\( dt \)$$ is the time step.


### Runge-Kutta 4th Order Method (RK4)
RK4 uses four intermediate steps:

$$
k_1^v = \mathbf{a}(\mathbf{p}, \mathbf{v})
$$
$$
k_1^p = \mathbf{v}
$$

$$
k_2^v = \mathbf{a}(\mathbf{p} + \frac{dt}{2} k_1^p, \mathbf{v} + \frac{dt}{2} k_1^v)
$$
$$
k_2^p = \mathbf{v} + \frac{dt}{2} k_1^v
$$

$$
k_3^v = \mathbf{a}(\mathbf{p} + \frac{dt}{2} k_2^p, \mathbf{v} + \frac{dt}{2} k_2^v)
$$
$$
k_3^p = \mathbf{v} + \frac{dt}{2} k_2^v
$$

$$
k_4^v = \mathbf{a}(\mathbf{p} + dt \cdot k_3^p, \mathbf{v} + dt \cdot k_3^v)
$$
$$
k_4^p = \mathbf{v} + dt \cdot k_3^v
$$

The final updates:

$$
\mathbf{v} = \mathbf{v} + \frac{dt}{6} (k_1^v + 2k_2^v + 2k_3^v + k_4^v)
$$

$$
\mathbf{p} = \mathbf{p} + \frac{dt}{6} (k_1^p + 2k_2^p + 2k_3^p + k_4^p)
$$

### Kinetic Energy
The kinetic energy of each mass:

$$
KE = \frac{1}{2} m v^2
$$

where \( v \) is the speed.

### Gravitational Potential Energy
The gravitational potential energy between two masses:

$$
U = -G \frac{m_1 m_2}{r}
$$

where $$\( r \)$$ is the distance between the masses.

### Orbital Velocity (Circular Orbit Approximation)
To initialize orbital motion:

$$
v = \sqrt{\frac{G (m_1 + m_2)}{r}}
$$

where $$\( r \)$$ is the initial separation between the two bodies.

---

## 👨‍💻 Tech stack
- [Raylib](https://www.raylib.com)


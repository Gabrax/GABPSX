<div align="center">
  
## Coloring Polygon

</div>

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

<div align="center">
  
## Collision Detection

</div>
  
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

<div align="center">
  
## Central Force Fields

</div>

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

<div align="center">
  
## Dependencies

</div>

<div align="center">

<p>
<a href="https://www.raylib.com">raylib</a>
</p>

</div>


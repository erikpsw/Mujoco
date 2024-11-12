For rigid body

<img src="Dynamic model/image-20241112195525405.png" alt="image-20241112195525405" style="zoom: 25%;" />

Using similar triangle, we have
$$
v_{o x}=\frac{v_{L}+v_{R}}{2} \\ \dot{\psi}_{o}=\frac{v_{L}-v_{R}}{W}
$$
 for velocity in y direction, using ICR (instantaneous centers of rotation)
$$
I C R_{x}=-\frac{v_{o y}}{\dot{\psi}_{o}} \\ I C R_{y}=\frac{v_{o x}}{\dot{\psi}_{o}}
$$
Combine these we get
$$
\left(\begin{array}{c}v_{o x} \\ v_{o y} \\ \dot{\psi}_{o}\end{array}\right)=\left(\begin{array}{cc}\frac{1}{2} & \frac{1}{2} \\ \frac{I C R_{x}}{W} & -\frac{I C R_{x}}{W} \\ \frac{1}{W} & -\frac{1}{W}\end{array}\right)\binom{v_{L}}{v_{R}}
$$

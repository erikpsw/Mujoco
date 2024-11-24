For rigid body, the local coordinate

<img src="Dynamic model/image-20241112195525405.png" alt="image-20241112195525405" style="zoom: 25%;" />

Using a similar triangle, we have
$$
v_{o x}=\frac{v_{L}+v_{R}}{2} \\ \dot{\psi}_{o}=\frac{v_{L}-v_{R}}{W}
$$
 for velocity in $y$ direction, using ICR (instantaneous centers of rotation)
$$
I C R_{x}=-\frac{v_{o y}}{\dot{\psi}_{o}} \\ I C R_{y}=\frac{v_{o x}}{\dot{\psi}_{o}}
$$
Combine these we get
$$
\left(\begin{array}{c}\dot{x}_{o} \\ \dot{y}_{o} \\ \dot{\psi}_{o}\end{array}\right)=\left(\begin{array}{cc}\frac{1}{2} & \frac{1}{2} \\ \frac{I C R_{x}}{W} & -\frac{I C R_{x}}{W} \\ \frac{1}{W} & -\frac{1}{W}\end{array}\right)\binom{v_{L}}{v_{R}}
$$

Since $ICR_x=0$, we can simplify our model to
$$
\left(\begin{array}{c}v \\ \omega \end{array}\right)=\left(\begin{array}{cc}\frac{1}{2} & \frac{1}{2}\\ \frac{1}{W} & -\frac{1}{W}\end{array}\right)\binom{v_{L}}{v_{R}}
$$
Using inverse
$$
\binom{v_{L}}{v_{R}}=\left(\begin{array}{cc}\frac{1}{2} & \frac{1}{2}\\ \frac{1}{W} & -\frac{1}{W}\end{array}\right)^{-1}\left(\begin{array}{c}v \\ \omega \end{array}\right)=\left(\begin{array}{cc}1 & \frac{W}{2}\\ 1 & -\frac{W}{2}\end{array}\right)\left(\begin{array}{c}v \\ \omega \end{array}\right)
$$
For state vector
$$
\xi=\left(\begin{array}{c}x\\y\\\theta\end{array}\right),u=\left(\begin{array}{c}v\\ \omega\end{array}\right)\\
\dot{\xi}=f(\xi, u) \Leftrightarrow\left[\begin{array}{c}\dot{x} \\ \dot{y} \\ \dot{\theta}\end{array}\right]=\left[\begin{array}{cc}\cos \theta & 0 \\ \sin \theta & 0 \\ 0 & 1\end{array}\right]\left[\begin{array}{c}v \\ \omega\end{array}\right]=\left[\begin{array}{c}v \cdot \cos \theta \\ v \cdot \sin \theta \\ \omega\end{array}\right] 
$$
Taylor linearization
$$
\dot{\xi}=f\left(\xi_r, u_r\right)+\frac{\partial f}{\partial \xi}_{\mid \xi=\xi_r, u=u_r}\left(\xi-\xi_r\right)+\frac{\partial f}{\partial u}_{\mid \xi=\xi_r, u=u_r}\left(u-u_r\right) 
$$
We define
$$
A=\left[\begin{array}{ccc}0 & 0 & -v_r \cdot \sin \theta_r \\ 0 & 0 & v_r \cdot \cos \theta_r \\ 0 & 0 & 0\end{array}\right] \quad B=\left[\begin{array}{cc}\cos \theta_r & 0 \\ \sin \theta_r & 0 \\ 0 & 1\end{array}\right] \quad O=-A\left[\begin{array}{l}x_r \\ y_r \\ \theta_r\end{array}\right]\\
\dot{\xi}=A \xi+B u+O
$$
Discretization
$$
\frac{\xi_{k+1}-\xi_{k}}{T}=A_{k}\xi_{k}+B_{k}u_{k}+{ O}_{k}\\
\begin{array}{l c r}{{\xi_{k+1}=(I+T A_{k})\xi_{k}+T B_{k}u_{k}+T O_{k}}} {{=\hat{A}_{k}\xi_{k}+\hat{B}_{k}u_{k}+\hat{O}_{k}}}\end{array}
$$

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

Plug in we get
$$
\hat{A}_k=\left[\begin{array}{ccc}1 & 0 & -Tv_r \sin \theta_r \\ 0 & 1 & Tv_r \cos \theta_r \\ 0 & 0 & 1\end{array}\right] \quad \hat{B}_k=\left[\begin{array}{cc}T\cos \theta_r & 0 \\ T\sin \theta_r & 0 \\ 0 & T\end{array}\right]\\\hat{O}_k=\left[\begin{array}{c} T\theta_rv_r \sin \theta_r \\  -T\theta_rv_r \cos \theta_r \\  0\end{array}\right]
$$
For future state
$$
{X}= \begin{bmatrix} \xi_{k+1}\\ \xi_{k+2}\\ \dots\\ \xi_{k+N}\end{bmatrix}  \;\;\;\;\;\;\;{U}= \begin{bmatrix} U_{k}\\ U_{k+1}\\ \dots\\ U_{k+N-1}\end{bmatrix}\\
X=M\xi_{k}+CU+NO \\
M=\left[\hat{ A}_{k}~~~\hat{ A}_{k}\hat{ A}_{k+1}~~\dots~~\prod_{i=0}^{N-1}\hat{ A}_{k+i}\right]^{T}_{3N\times 3} \\
C=\left[\begin{array}{c c c c} {{\hat{B}_{k}}}&{{0}}&{{\cdots}}&{{0}}\\  {{\hat{A}_{k+1}\hat{B}_{k}}}&{{\hat{B}_{k+1}}}&{{\cdots}}&{{0}}\\  \vdots & \vdots & \ddots & \vdots\\  {{\prod_{i=1}^{N-1}\hat{A}_{k+i}\hat{B}_{k}}}& {{\prod_{i=2}^{N-1}\hat{A}_{k+i}\hat{B}_{k+1}}}& \dots&{{\hat{B}_{k+N-1}}}\end{array}\right]_{3N \times 2N}\\D= \begin{bmatrix} 1 & 0 & \cdots & 0\\  {{\hat{A}_{k+1}}}&{{1}}&{{\cdots}}&{{0}}\\  {{\vdots}}&{{\vdots}}&{{\ddots}}&{{\vdots}}\\  \prod_{i=1}^{N-1}\hat{A}_{k+i} & \prod_{i=2}^{N-1}\hat{A}_{k+i} &{{\cdots}}&{{1}} \end{bmatrix} _{3N \times 3N} \\
{O}= \begin{bmatrix} \hat{O}_{k}\\ \hat{O}_{k+1}\\ \vdots\\ \hat{O}_{k+N-1}\\ \end{bmatrix} _{3N\times 1}
$$
Error
$$
\tilde{X}=X-X_{r}=MX_{k}+CU+DO-X_{r}
$$
So the objective function is 
$$
\begin{align} \tilde{X}^{T}Q\tilde{X} &=(MX_{k}+DO-X_{r}+CU)^{T}Q(MX_{k}+DO-X_{r}+CU) \\ &=(E+CU)^TQ(E+CU)\\ &=E^{T}Q E+E^{T}QCU+U^{T}C^{T}QCU+U^{T}C^{T}Q E\\ &=U^{T}\left(C^{T}QC\right)U+\left(2E^TQ C\right)U+E^{T}Q E \end{align}
$$

Optimization
$$
\underset{U}{\min}\;J=\frac{1}{2}\,U^{T}\left(2C^{T}{Q}C+2W\right)U+\left(2E^TQ C\right)U  \\  \text{s.t.} \;\;U_{\min}\le U \le U_{\max}\
$$

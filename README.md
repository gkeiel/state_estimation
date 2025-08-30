# State estimation

A repository of algorithms for state estimation in modern control systems. Each algorithm has its own folder, containing the function code and a variety of systems as examples.

The following algorithms are available:
- [Kalman filter](./kalman_filter)

## Kalman filter

A code to compute Kalman optimal gain and minimum mean square error (MMSE) estimates of a system states. A variety of models avaiable as example. Easily adaptable to other systems and inputs, which makes it good for study applications.

Considering an RC system with process noise where $y(t)$ is the capacitor voltage $v_c(t)$, the following results are obtained:
<img width="1920" height="926" alt="kalman_filter_01" src="https://github.com/user-attachments/assets/b91bf736-91cd-4a8f-95bb-141f03522659" />

Considering an DC motor system with process noise where $y(t)$ is the angular position $\theta (t)$, the following results are obtained:
<img width="1920" height="926" alt="kalman_filter_02" src="https://github.com/user-attachments/assets/6dcdc34e-15ab-411f-a499-967c9a5c85fd" />

Considering an RLC system with process noise where $y(t)$ is the capacitor voltage $v_c(t)$, the following results are obtained:
<img width="1920" height="926" alt="kalman_filter_03" src="https://github.com/user-attachments/assets/ce78c1aa-097c-4599-b0c8-b8e4448af824" />

# intermediate_axis_theorem_sim

Simulation of the intermediate axis theorem (as known as the tennis racket theorem or the Dzhanibekov effect).

## Euler’s equations of rigid body motion (Euler's rotation equations)

- General vector form

![general_vector_form](assets/general_vector_form.png)
    
- Under torque-free condition

![torque_free_condition](assets/torque_free_condition.png)


## Simulation (visualised in ROS 2 RViz2)
- Rotate mainly along $\omega_1$ ($I_1$), with small perturbations along the other two axes.
    > Stable rotation
    <img src="assets/i1.gif" alt="Intermediate Axis Theorem Demo" width="400"/>

- Rotate mainly along $\omega_3$ ($I_3$), with small perturbations along the other two axes.
    > Stable rotation
    <img src="assets/i3.gif" alt="Intermediate Axis Theorem Demo" width="400"/>

- Rotate mainly along $\omega_2$ ($I_2$) (the intermediate axis), with small perturbations along the other two axes.
    > Unstable rotation (axis flipped)
    <img src="assets/i2.gif" alt="Intermediate Axis Theorem Demo" width="400"/>


## Demo

Run the simulation (rigid body rotation in the three main axes, with other two being perturbed):
```bash
# Simulation in matplotlib
cd ~/intermediate_axis_theorem_sim && python3 sim.py

# Simulation in ros2 and rivz2
cd ~/intermediate_axis_theorem_sim && python3 sim_ros2.py
```
# UR-FP02
iLQG

This was developed as part of a project in the course "Underactuated Robots" at La Sapienza University by the students Barbara Bazzana, Tommaso Belvedere and Christian D’Amico.

The main file is where the simulation is run. Inside the simulation loop, the trajectoryOptimization function is called to perform the forwards and backwards passes and the regularization steps as described in the paper's algorithm. The control in the found optimal solution is applied to the system to control it in MPC-like fashion. The next iteration the algorithm can achieve faster convergence by using the previously found solution as initial guess.

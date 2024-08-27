<div align="center">
<h1> FRSP</h1>
<h2>Flow Inspired Multi-Robot Real-Time Scheduling Planner</h2>
</div>

## 📜 Introduction

Collision avoidance and trajectory planning are crucial in multi-robot systems, particularly in environments with numerous obstacles. Although extensive research has been conducted in this field, the challenge of rapid traversal through such environments has not been fully addressed. 
This paper addresses this gap by proposing a novel real-time scheduling scheme designed to optimize the passage of multi-robot systems through complex, obstacle-rich maps. Drawing inspiration from network flow optimization, our scheme decomposes the environment into a network structure, enabling the efficient allocation of robots to paths based on real-time congestion data. The proposed scheduling planner operates on top of existing collision avoidance algorithms, focusing on minimizing traversal time by balancing robot detours and waiting times. 
Our experimental results show high computational efficiency, with an average calculation time of around 0.9 seconds per instance for 500 robots.
We further validated the effectiveness of our scheme through real world flight tests involving ten quadrotors.
This work contributes a lightweight, effective scheduling planner capable of meeting the real-time demands of multi-robot systems in obstacle-rich environments.

<div align=center><img src="imgs/system.jpg" height=40% width=40% ></div>


## 🛠️ Installation
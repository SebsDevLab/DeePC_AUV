# DeePC AUV Project
Data-Enabled Predictive Control with Predictive Adaptive Line-of-Sight Guidance for the REMUS 100 AUV

## Publication Reference
S. Zieglmeier, M. Hudoba de Badyn, N. D. Warakagoda, T. R. Krogstad, P. Engelstad, *Data-Enabled Predictive Control with Predictive Adaptive Line-of-Sight Guidance for 3-D Path Following of Autonomous Underwater Vehicles*, arXiv preprint, 2026. arXiv: [2510.25309](https://arxiv.org/abs/2510.25309)

---

## Overview
This repository implements a **fully data-driven control and guidance framework** for the REMUS 100 AUV based on Data-Enabled Predictive Control (DeePC). The framework eliminates explicit hydrodynamic modelling by using measured input-output trajectories, and extends classical Adaptive Line-of-Sight (ALOS) guidance to a predictive multistep formulation (PALOS) for 3-D waypoint path following.

Key features:
- DeePC implementation for heading control
- Cascaded DeePC architecture with loop-frequency separation for depth control
- Predictive Adaptive Line-of-Sight (PALOS) guidance for 3-D waypoint path following
- Integration with the Marine Systems Simulator (MSS) toolbox by Thor I. Fossen
- Validation across nominal operation, ocean-current disturbances, operation beyond the data regime, and 3-D waypoint missions

---

## Requirements and Dependencies

| Tool | Version | Source | License |
|------|---------|--------|---------|
| MATLAB | R2024b (tested) | [MathWorks](https://www.mathworks.com) | Proprietary |
| YALMIP | latest | [YALMIP website](https://yalmip.github.io/download/) | GNU GPL |
| MOSEK | 10.x or latest | [MOSEK downloads](https://www.mosek.com/downloads/) | Free for academic use |
| MSS Toolbox (Fossen) | 2024 version | [MSS GitHub](https://github.com/cybergalactic/MSS) | MIT License |
| This project | — | GitHub repo | MIT License |

---

## Repository Structure
DeePC_AUV/
├── SIMremus100_with_DeePC.m    Main simulation entry point
├── PALOS/
│   └── PALOS.m                 Predictive ALOS implementation
├── DeePC/                      DeePC formulation and solver setup
└── README.md


---

## How to Run

1. Install the dependencies listed above and add them to your MATLAB path.
2. Clone the MSS toolbox and add it to the MATLAB path.
3. Open `SIMremus100_with_DeePC.m` in MATLAB.
4. Configure the scenario at the top of the script (nominal, ocean currents, beyond-data-regime, or 3-D path following).
5. Run the script. Plots of the AUV response are generated at the end of the simulation.

The DeePC and PALOS hyperparameters can be adjusted in the corresponding script sections.

---

## Citation

If you use this repository or the code in your research, please cite:

```bibtex
@article{zieglmeier2025data,
  author    = {Sebastian Zieglmeier and Mathias Hudoba de Badyn and Narada D. Warakagoda and Thomas R. Krogstad and Paal Engelstad},
  title     = {Data-Enabled Predictive Control with Predictive Adaptive Line-of-Sight Guidance for 3-D Path Following of Autonomous Underwater Vehicles},
  journal   = {arXiv preprint arXiv:2510.25309},
  year      = {2026}
}
```

---

## Contact

**Sebastian Zieglmeier**
Email: s.g.zieglmeier@its.uio.no
GitHub: [SebsDevLab](https://github.com/SebsDevLab)

**Mathias Hudoba de Badyn**
Email: mathias.hudoba.de.badyn@its.uio.no

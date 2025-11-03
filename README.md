# DeePC AUV Project  
Data-Enabled Predictive Control and Guidance for the REMUS 100 AUV

## Publication Reference  
S. Zieglmeier, M. Hudoba de Badyn, N. D. Warakagoda, T. R. Krogstad, P. Engelstad, *Data-Enabled Predictive Control and Guidance for Autonomous Underwater Vehicles*, arXiv preprint, 2025. DOI / arXiv: [2510.25309](https://arxiv.org/abs/2510.25309)

---

## Overview  
This repository implements a **fully data-driven control & guidance framework** for the REMUS 100 AUV, based on the DeePC (Data-Enabled Predictive Control) method. The goal is to eliminate the need for explicit hydrodynamic modelling by using measured input-output data, combined with a predictive guidance law (PALOS) for 3-D waypoint tracking.

Key features:
- DeePC implementation for depth & heading control  
- Cascaded control architecture (inner/outer loops)  
- 3D path following via Adaptive Line-of-Sight (ALOS) predictive guidance  
- Integration with the MSS (Marine Systems Simulator) toolbox by Thor I. Fossen  
- Ready-to-use simulation environment with the REMUS 100 AUV model  

---

## Requirements & Dependencies  

| Tool | Version | Source | License / Note |
|------|---------|--------|---------------|
| MATLAB | R2024b (tested) | [MathWorks](https://www.mathworks.com) | Proprietary |
| YALMIP | latest | [YALMIP website](https://yalmip.github.io/download/) | GNU GPL |
| MOSEK | 10.x or latest | [MOSEK downloads](https://www.mosek.com/downloads/) | Free for academic use |
| MSS Toolbox (Fossen) | 2024 version | [MSS GitHub](https://github.com/cybergalactic/MSS) | MIT License |
| This project | — | GitHub repo | MIT License |

---

## Citation

If you use this repository or the code in your research, please cite:

@article{Zieglmeier2025_DeePC_AUV,
  author    = {Sebastian Zieglmeier and Mathias Hudoba de Badyn and Narada D. Warakagoda and Thomas R. Krogstad and Paal Engelstad},
  title     = {Data-Enabled Predictive Control and Guidance for Autonomous Underwater Vehicles},
  journal   = {arXiv preprint arXiv:2510.25309},
  year      = {2025}
}


## Contact

Sebastian Zieglmeier

Email: seb.zieglmeier@gmail.com, s.g.zieglmeier@its.uio.no

GitHub: SebsDevLab

Mathias Hudoba de Badyn

Email: mathias.hudoba.de.badyn@its.uio.no

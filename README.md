# Adaptive CPG-ZMP Exoskeleton Controller

A multi-disciplinary control framework for a Lower-Limb Robotic Exoskeleton (LLRE), integrating Central Pattern Generators (CPG) and Zero Moment Point (ZMP) stability for uneven terrain locomotion in post-stroke rehabilitation.

## Overview

This project implements a three-module control algorithm for adaptive gait rehabilitation:

- **Module 1 — Matsuoka CPG Rhythm Generator**: Produces bio-mimetic gait rhythms using a coupled neural oscillator model, phase-locked to each patient's natural cadence.
- **Module 2 — ZMP Stability Supervisor**: Monitors the Zero Moment Point in real time and triggers hierarchical balance strategies (ankle → hip → stepping) when postural instability is detected.
- **Module 3 — Adaptive PID Assistive Controller**: Uses contralateral limb mirroring with an Assist-as-Needed gain schedule to minimise the kinematic gap between paretic and non-paretic limbs.

The system was validated on a clinical dataset of using 4 of the 50 stroke survivors across five terrain profiles (Flat, Slope, Rough, Sine, Step).

## Requirements

- **MATLAB R2024a** or later
- **Simulink** (included with MATLAB)
- **Simscape Multibody** toolbox

No additional toolboxes are required.

## Dataset

This project uses the publicly available post-stroke gait dataset by Van Criekinge et al. (2023):

> Van Criekinge, T., Saeys, W., Truijen, S., Vereeck, L., Sloot, L.H. and Hallemans, A. (2023). *A full-body motion capture gait dataset of 138 able-bodied adults across the life span and 50 stroke survivors.* Scientific Data, 10, 852.

**Download the dataset from Figshare:**

🔗 [https://doi.org/10.6084/m9.figshare.c.6503791.v1](https://doi.org/10.6084/m9.figshare.c.6503791.v1)

You need the normalised post-stroke `.mat` file:

```
MAT_normalizedData_PostStrokeAdults_v27-02-23.mat
```

## Setup

1. **Clone the repository**
   ```bash
   git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git
   cd YOUR_REPO_NAME
   ```

2. **Download the dataset**
   - Go to the [Figshare collection](https://doi.org/10.6084/m9.figshare.c.6503791.v1)
   - Download `MAT_normalizedData_PostStrokeAdults_v27-02-23.mat`
   - Place the `.mat` file in the root project directory

3. **Import the dataset into the MATLAB workspace**
   ```matlab
   load('MAT_normalizedData_PostStrokeAdults_v27-02-23.mat')
   ```

4. **Run the simulation**
   - Open the main script in MATLAB
   - Select a Subject ID (1–50) when prompted
   - The simulation will run the baseline (uncontrolled) and controlled configurations sequentially

## Project Structure

```
├── README.md
├── main.m                      % Entry point — runs full simulation pipeline
├── pre_controller/             % Module 1: Kinematic analysis & CPG initialisation
├── controller/                 % Module 2 & 3: PID-CPG gait + ZMP balance controller
├── terrain/                    % Terrain generation (Flat, Slope, Step, Rough, Sine)
├── visualisation/              % 3D gait animation and plotting scripts
└── results/                    % Output figures and readiness scores
```

## Citation

If you use this code in your research, please cite:

```
Zafar, M.H. (2026). A Multi-Disciplinary Approach to Exoskeleton Control: Integrating
Central Pattern Generators and Zero Moment Point Stability for Uneven Terrain Locomotion.
BEng Thesis, University of Leeds.
```

## Licence

This project is for academic purposes. The clinical dataset is available under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) as provided by Van Criekinge et al.

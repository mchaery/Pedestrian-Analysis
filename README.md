# Pedestrian Data Analysis

A MATLAB-based program for analyzing human gait patterns using real-world joint angle data collected from multiple views. 

## 📌 Overview

This project addresses the complexity of real-world sensor data through computational modeling and signal processing.  
We analyzed pedestrian gait using joint angle data (shoulder, pelvis, knee, ankle) from both **back** and **right** perspectives.  
The program allows users to compare two pedestrian cases through:

- 📊 Plots of joint angle trajectories
- 🧩 Overlayed comparative graphs
- 📈 Statistical summaries of motion cycles

## 📌 Key Features

- Kalman and low-pass filters for denoising and signal enhancement  
- Fast Fourier Transform (FFT) to reveal periodic motion patterns in the frequency domain  
- Automatic generation of comparative graphs and CSV reports  
- Statistical analysis: mean trajectories and standard deviation bands  

## 🛠 Technologies Used

- **Language**: MATLAB  
- **Techniques**: Kalman filter, low-pass filter, FFT, statistical modeling  
- **Data Format**: `.csv` with joint angle data  

## 📌 How It Works

The program:
1. Reads raw gait data from `reference_CES48.csv`
2. Processes shoulder, pelvis, knee, and ankle angles from back and right views
3. Applies filters and transformations
4. Outputs comparative results in a new `.csv` and plots for visualization

Team Project @ Hongik University Digital Human Lab

---

> 📎 *This project helped me strengthen my skills in filtering noisy real-world data, designing intuitive analysis pipelines, and building user-friendly research tools in MATLAB.*

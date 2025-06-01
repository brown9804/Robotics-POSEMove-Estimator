# Robotics Pose Estimation, Rotation \& Translation

University of Costa Rica

[![GitHub](https://img.shields.io/badge/--181717?logo=github&logoColor=ffffff)](https://github.com/)
[brown9804](https://github.com/brown9804)

Last updated: 2019-11-14

----------

> This repository contains code for estimating the position and orientation of a robot part, as well as the translation and rotation of the robot over time.

The main file of this project is responsible for the core functionality of pose estimation, rotation, and translation.

This program reads from a text file called `current_control_parameters.txt`, the position `[GX,GY,GZ]^T` and orientation `R=[RX, RY, RX]^T` of the local coordinate system of an object `(R,S,T)` with respect to the reference coordinate system `(X,Y,Z)`, as well as the position of a point `[HR,HS,HT]^T` of the object with respect to the coordinate system of the object `(R,S,T)`. It then calculates the position `[HX,HY,HZ]^T` of that same point with respect to the reference coordinate system `(X,Y,Z)`. For this, it uses the pose transformation:

$$ [HX,HY,HZ]^T=R*[HR,HS,HT]^T+[GX,GY,GZ]^T $$

where `R` is the corresponding rotation matrix. The rotation matrix `R` and the position `[HX,HY,HZ]^T` are displayed in the terminal and also stored in a text file called `resultados.txt`.

## Dependencies

> The program uses the following headers from the standard C library:

- `<string.h>` for string operations
- `<stdio.h>` for input and output operations
- `<stdlib.h>` for dynamic memory management and process control
- `<math.h>` for mathematical functions

## Usage

> To use this program, compile and run the main file. The program will read the control parameters from the `current_control_parameters.txt` file, perform the calculations, and save the results in the `resultados.txt` file.

<div align="center">
  <h3 style="color: #4CAF50;">Total Visitors</h3>
  <img src="https://profile-counter.glitch.me/brown9804/count.svg" alt="Visitor Count" style="border: 2px solid #4CAF50; border-radius: 5px; padding: 5px;"/>
</div>

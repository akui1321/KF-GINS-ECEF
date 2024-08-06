# KF-GINS-ECEF

[[中]](./readme_zh.md) &ensp; [[EN]](./readme.md)

## Overview
This project implements a loosely coupled GNSS and INS positioning in the Earth-Centered, Earth-Fixed coordinate system (ECEF). It includes initial alignment algorithms, IMU mechanization algorithms, and a loosely coupled algorithm based on the Extended Kalman Filter (EKF), which can achieve robust and high-precision positioning.

The project contains two folders:
- `dataset`: The dataset folder, which contains three dynamic experimental data segments collected in Wuhan University.
- `GINS`: The program folder, which includes the solution file and the source code folder. The source code folder further includes header files, cpp files, and configuration files.

## Opening the Program
Use Microsoft Visual Studio to open the solution file in the `GINS` folder to open this program.

## Eigen Library Configuration
Since the program uses the Eigen library to manage matrices, you need to download the Eigen library first. You can download the Eigen library from the following link:
[Eigen Library Download](https://eigen.tuxfamily.org/index.php?title=Main_Page)

## Project Configuration Steps
1. **Eigen Library Path Configuration**: After opening the project, first configure the path of the Eigen library in the properties to ensure that the project can use the Eigen library normally.

2. **Configuration File Modification**:
   - Modify the path of the configuration file in `main.cpp`.
   - The configuration files include:
     - `friend.cfg`: The configuration file for Friendship Square.
     - `play_in.cfg`: The configuration file for the inner side of the playground.
     - `play_out.cfg`: The configuration file for the outer side of the playground.

3. **Data File Path Configuration**: Modify the paths in the configuration files that point to the data files, including the following five paths:
   - IMU file path
   - GNSS file path
   - Reference result file path
   - Processed result file path
   - Comparison result file path

   These files can all be found in the `dataset` folder. Please modify the configuration file paths according to the paths on your device.

## Program Execution
Once all path issues are resolved, you can run the program in release mode. After running, you will get the processed result file and the comparison result file. Using these result files, you can draw result diagrams and perform various analyses.

## Notes
- Make sure all paths are configured correctly to avoid errors during program execution.
- Ensure that the content format of the dataset files is correct, and the format must strictly follow the content format of the `dataset`.
- If you encounter other issues, please contact chengzhuogui@gmail.com.

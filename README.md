
# ADAPTIVE FEEDBACK CONTROLLER FOR FUNCTIONAL ELECTRICAL STIMULATION CONTROL OF THE ELBOW JOINT 

**Creator:** _Markus Haukipää_,&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;(Code template: 横谷隆佑, (Ryusuke 
Yokoya))  
**Email:** _markus.haukipaa@tuni.fi_  
**School / Affiliation:**  
_**T**ohoku University Biomedical Engineering(Japan),  
**T**ampere University Biomedical Sciences and Engineering (Finland)_  
**Master's Thesis**


## Description

_The program uses adaptive algorithm to control the simulated Elbow 
joint angle in excitation. Aiming finding the performance of
adaptive Proportional Derivate Integral (PID) controller in elbow angle control. 
The adaptive is done using Stochastic Extrenum Seeking (SES) algorithm.

This is done to simulate the performance of a real-life automatic Functional 
Electric Stimulation (FES).  It utilizes a simplified arm model from OpenSim,
making it well-suited for researching fundamental control challenges,
potential solutions and parameter tuning effects. The program allows the 
testing of variety of different muscle model types. Additionally, program 
does the visualizing of the process and collecting and showing the important 
data points._

---

---

### 📁 `Muscle_models/`

_Description:_  
Different model information from OpenSim. In the simulation default is used.

---

### 📁 `Program/`

_Description:_  
Holds all nessesary for running the program

- **Main/** – Main control and functionality
- **sub/** – Supporting functions or modules used by `Main`.

---

### 📁 `Result_Folder/`

_Description:_  
Stores output data generated from simulation runs.

---

### 📄 `.gitignore`

_Standard Git ignore file to exclude temporary or generated files from version control._

---

### 📄 `README.md`

_You're looking at it!

---

## Getting Started

More descriptive demands on needs can be found from the OpenSim's website.  
Python 3.8 is needed for the Opensim 4.5 that we utilize. Main folder holds the main files. The Control_Panel functions as the main 
control of the Simulator. There you may change the models, trajectories and the controllers used. Run the Control_Panel to run 
the program.

Simulator holds basic functionality. 
ES,SES_algorithm and PID_controller work together as the adaptive controller.
Changes to them can be made in their respected files. Osim file initilizes 
and works as the interference to Opensim.

---

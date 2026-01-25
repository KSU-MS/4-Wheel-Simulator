# 4-Wheel Tire Model Simulator

A Python-based 4-wheel vehicle dynamics simulator featuring longitudinal and lateral tire forces with **Pacejka tire model integration**.
The simulator is designed to achieve **within 10% accuracy** compared to a full high-fidelity run, including **straight-line into corner braking accuracy**.

---

## Features

* 4-wheel vehicle dynamics model
* Straight-line acceleration and braking into cornering
* Pacejka tire model (in progress / being introduced)
* Modular setup for **path**, **motor**, and **vehicle parameters**
* Outputs simulation data to a configurable data folder

---

## Requirements

* Windows OS
* Python 3 (Windows Store version recommended)

---

## Installation & Setup

1. **Download the repository as a ZIP**
2. **Unzip** the folder to your desired location
3. **Install Python 3**

   * Open the Microsoft Store
   * Search for **Python**
   * Install the latest Python 3 version
4. **Run** the PipInstaller.bat file
5. **Open a terminal in the project folder**

   * Right-click on an empty space inside the folder
   * Click **Open in Terminal**

---

## Running the Simulator

From the terminal, run:

```bash
python3 Simulator.py
```

> ⚠️ **Important:**
> Continue using `Simulator.py` **until this file is explicitly removed or deprecated**.

---

## Configuration

To modify the simulation setup:

1. Open `Simulator.py`
2. Scroll to the **very bottom** of the file
3. Edit the following parameters as needed:

   * **Path**
   * **Motor**
   * **Vehicle / Car parameters**
4. Change the value of the output directory:

   * Update the path where simulation data is saved (default is the `Data` folder)

---

## Output Data

* Simulation results are automatically saved to the specified **Data** folder
* Output format and content depend on the selected vehicle and path configuration

---

## Notes

* The Pacejka tire model is actively being introduced and refined
* Accuracy target: **within 10%** of full simulation results

---

## Troubleshooting

* Ensure Python is added to your system PATH
* If `python3` does not work, try:

  ```bash
  python Simulator.py
  ```
* Verify that all required files remain in the same directory structure after unzipping







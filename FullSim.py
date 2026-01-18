import matplotlib.pyplot as plt
import numpy as np  
import pandas as pd
import types
from matplotlib.collections import LineCollection
from time import time

EPS = 1e-9

def CSV_Reader(file_path, key_col=0, value_col=1):
    """Reads a CSV into a SimpleNamespace for dot-notation access."""
    data = pd.read_csv(file_path, header=0)
    data = data.set_index(data.keys()[key_col]).to_dict()[data.keys()[value_col]]
    for key in data:
        try:
            data[key] = float(data[key])
        except:
            try:
                data[key] = data[key].lower() == 'true'
            except:
                pass 
    return types.SimpleNamespace(**data)

def TrackLoader(file_path):
    """Loads track data as a numpy array."""
    data = pd.read_csv(file_path, header=None)
    return data.to_numpy()

def AeroForce(veh, v):
    """Calculates downforce and drag."""
    v2 = v**2
    F_df = 0.5 * veh.rho_air_kgpm3 * v2 * veh.ClA
    F_drag = 0.5 * veh.rho_air_kgpm3 * v2 * veh.CdA
    return F_df, F_drag


def NormalForce(veh, Fdf, ax, ay):
    """Calculates individual wheel loads including weight transfer."""
    m = veh.m_total_kg 
    g = 9.81
    
    Fz_f_static = m*g*(1-veh.swd_frac) + Fdf*(1-veh.cp_frac)
    Fz_r_static = m*g*veh.swd_frac + Fdf*veh.cp_frac
    
    WT_long = (m * ax * veh.cg_z_m) / veh.wb_m
    Fz_f = Fz_f_static - WT_long 
    Fz_r = Fz_r_static + WT_long
    
    Fz_total = np.maximum(Fz_f + Fz_r, EPS)
    WT_lat_f = (m * ay * veh.cg_z_m / veh.tw_f_m) * (Fz_f / Fz_total)
    WT_lat_r = (m * ay * veh.cg_z_m / veh.tw_r_m) * (Fz_r / Fz_total)
    
    return types.SimpleNamespace(**{"Fz": np.array([
    Fz_f/2 + WT_lat_f/2, # FL
    Fz_f/2 - WT_lat_f/2, # FR
    Fz_r/2 + WT_lat_r/2, # RL
    Fz_r/2 - WT_lat_r/2  # RR
    ])})

def MotorForceLimitsSingle(motor, v, r_tire, Fx_cap_rear, P_pack_limit):
    """Determines powertrain output based on motor, tire, and battery limits."""
    omega_wheel = v / r_tire
    omega = omega_wheel * motor.n_gear
    
    T_wheel_max = motor.tmax_nm * motor.n_gear * motor.tran_eff
    T_pack_cap = motor.kt_nm_per_a * motor.i_max_a * motor.n_gear * motor.tran_eff if v < EPS else P_pack_limit / omega_wheel

    T_rear_total = np.maximum(0,np.min([T_wheel_max, T_pack_cap, np.sum(Fx_cap_rear) * r_tire]))

    T_used = (T_rear_total) / (motor.n_gear * motor.tran_eff)
    I_phase = T_used / motor.kt_nm_per_a
    P_mech = T_rear_total * omega_wheel
    V_back = motor.ke_v_per_rad * omega
    P_elec = P_mech / motor.tran_eff + (I_phase**2 * motor.r_phase_ohm) + (V_back * motor.i0_a)

    return T_rear_total/r_tire, P_elec, P_mech, I_phase, omega

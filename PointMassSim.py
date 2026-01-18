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
    m = veh.m_total_kg
    g = 9.81
    Fz = m * g + Fdf
    return types.SimpleNamespace(**{"Fz": Fz})
def MotorForceLimitsSingle(motor, v, veh, Fx_load, ):
    """Calculates motor force limits based on torque and power limits."""
    omega_wheel = v / veh.r_tire_m
    omega = omega_wheel * motor.n_gear / motor.tran_eff
    T_motor_limit = motor.tmax_nm*motor.n_gear* motor.tran_eff
    T_power_limit = veh.power_limit_W/omega_wheel if v >EPS else motor.kt_nm_per_a * motor.i_max_a * motor.n_gear * motor.tran_eff
    T_mech_limit = min(T_motor_limit, T_power_limit,Fx_load*veh.r_tire_m)

    T_used = T_mech_limit / (motor.n_gear * motor.tran_eff)
    I_phase = T_used / motor.kt_nm_per_a
    P_mech = T_mech_limit * omega_wheel
    V_back = motor.ke_v_per_rad * omega
    P_elec = P_mech / motor.tran_eff + (I_phase**2 * motor.r_phase_ohm) + (V_back * motor.i0_a)

    return T_mech_limit/veh.r_tire_m, P_elec, P_mech, I_phase, omega
def SimulateSegment(veh, motor, seg, state):
    v_vec = []
    ax_vec = []
    ay_vec = []
    P_elec_vec = []
    P_mech_vec = []
    I_phase_vec = []
    omega_vec = []
    P_batt_vec = []
    P_drag_vec = []
    s = 0.0
    v = state.v
    t = 0.0
    E_Wh = 0.0
    ay = state.ay
    ax = state.ax
    while s < seg.L - EPS:
        Fdf, Fdrag = AeroForce(veh, v)
        loads = NormalForce(veh, Fdf, ax, ay)
        if seg.R > EPS:
            v_corner_cap = np.sqrt(veh.mu_lat_0 * veh.m_total_kg * 9.81 * seg.R/(veh.m_total_kg - veh.mu_lat_0* seg.R*veh.ClA*veh.rho_air_kgpm3*.5))
            if v > v_corner_cap:
                v = v_corner_cap
            ay = v**2 / seg.R
        else:
            ay = 0
        Fx_max = veh.mu_long_0 * (loads.Fz ** veh.load_sens_exp)
        Fy_max = veh.mu_lat_0 * (loads.Fz ** veh.load_sens_exp)
        Fy_req = loads.Fz*ay/9.81 if ay > EPS else 0
        Fx_load = Fx_max*np.sqrt(1-(Fy_req/Fy_max)**2)
        Fx_limit, P_elec_motor, P_mech_motor, I_phase, omega = MotorForceLimitsSingle(motor, v, veh, Fx_load/2)
        ax = (Fx_limit - Fdrag) / veh.m_total_kg

        s = s + v * veh.dt_s + 0.5 * ax * veh.dt_s**2
        v = v + ax * veh.dt_s
        t = t + veh.dt_s
        v_vec.append(v)
        ax_vec.append(ax)
        ay_vec.append(ay)
        P_elec_vec.append(P_elec_motor)
        P_mech_vec.append(P_mech_motor)
        I_phase_vec.append(I_phase)
        omega_vec.append(omega)
        Vpack = np.maximum(veh.pack_voltage_v, 12.0)
        I_pack = P_elec_motor / Vpack
        P_loss = (I_pack**2) * veh.r_internal_ohm
        P_pack = P_elec_motor + P_loss
        P_batt_vec.append(P_pack)
        P_drag_vec.append(Fdrag*v)
        E_Wh += (P_pack * veh.dt_s) / 3600.0

    return (t,v_vec, ax_vec, ay_vec, 
            P_elec_vec, P_mech_vec, P_drag_vec, P_batt_vec, 
            E_Wh)
def SimulateLap(track,veh,motor):
    res = {k: np.array([]) for k in ["time_s", "s_path", "v", "a_long", "a_lat", "P_elec", "P_mech", "P_drag", "P_batt_loss", "Fx_total", "Fy_total"]}
    E_Wh, t0, current_s = 0.0, 0.0, 0.0
    state = types.SimpleNamespace(v=0.0, s=0.0, ax=0.0, ay=0.0)
    for k in range(1, track.shape[0]):
        seg = types.SimpleNamespace(R=np.float64(track[k][1]), L=np.float64(track[k][2]))
        t, v, ax, ay, P_elec, P_mech, P_drag, P_batt, Eseg = SimulateSegment(veh, motor, seg, state)
    
        res["s_path"] = np.append(res["s_path"], current_s + seg.L)
        res["v"], res["a_long"], res["a_lat"] = np.append(res["v"], v), np.append(res["a_long"], ax), np.append(res["a_lat"], ay)
        res["P_elec"], res["P_mech"], res["P_drag"] = np.append(res["P_elec"], P_elec), np.append(res["P_mech"], P_mech), np.append(res["P_drag"], P_drag)
        res["P_batt_loss"] = np.append(res["P_batt_loss"], P_batt)

        E_Wh += Eseg
        state.v = v[-1]
        current_s += seg.L
        t0 += t
    res.update({"lap_time_s": t0, "energy_used_Wh": E_Wh, "v_max": np.max(res["v"]), "v_mean": np.mean(res["v"])})
    return types.SimpleNamespace(**res)

def Plotters(track_path, r):
    """Generates kinematics, power flow, and track velocity map."""
    plt.style.use('bmh')
    fig1, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axs[0].plot(r.time_s, r.v); axs[0].set_ylabel('Speed (m/s)')
    axs[1].plot(r.time_s, r.a_long); axs[1].set_ylabel('ax (m/s^2)')
    axs[2].plot(r.time_s, r.a_lat); axs[2].set_ylabel('ay (m/s^2)'); axs[2].set_xlabel('Time (s)')
    plt.suptitle('Kinematics')
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(r.time_s, r.P_elec/1e3, label='Electrical')
    plt.plot(r.time_s, r.P_mech/1e3, label='Mechanical')
    plt.plot(r.time_s, r.P_drag/1e3, label='Drag')
    plt.xlabel('Time (s)'); plt.ylabel('Power (kW)'); plt.legend(); plt.title('Power Flows')

    df = pd.read_csv(track_path, names=['radius_m', 'length_m'], header=0)
    x, y, theta = 0.0, 0.0, 0.0
    X, Y = [x], [y]
    for R, L in zip(df.radius_m, df.length_m):
        s = np.linspace(0, L, 50)
        if abs(R) < EPS:
            xs, ys = x + s*np.cos(theta), y + s*np.sin(theta)
        else:
            k = 1.0/R
            xs = x + (1/k)*(np.sin(theta + k*s) - np.sin(theta))
            ys = y - (1/k)*(np.cos(theta + k*s) - np.cos(theta))
            theta += k*L
        x, y = xs[-1], ys[-1]
        X.extend(xs); Y.extend(ys)
    
    points = np.column_stack((X, Y))
    segments = np.stack([points[:-1], points[1:]], axis=1)
    vel_interp = np.interp(np.linspace(0, 1, len(segments)), np.linspace(0, 1, len(r.v)), r.v)
    lc = LineCollection(segments, cmap='turbo', linewidth=3)
    lc.set_array(vel_interp)

    fig3, ax3 = plt.subplots(figsize=(8, 8))
    ax3.add_collection(lc); ax3.autoscale(); ax3.set_aspect('equal')
    plt.colorbar(lc, label='Velocity (m/s)'); plt.title('Velocity Trace')
    plt.show()


def Main(veh_path, motor_path, track_path):
    t1 = time()
    veh = CSV_Reader(veh_path)
    motor = CSV_Reader(motor_path)
    track = TrackLoader(track_path)
    R = SimulateLap(track, veh, motor)
    t2 = time()
    print("time:" , t2-t1 )
    print(f"\n--- Simulation Results ({veh_path}) ---")
    print(f"Lap Time:   {R.lap_time_s:.2f} s")
    print(f"Energy:     {R.energy_used_Wh/1000:.2f} kWh")
    print(f"Max Speed:  {R.v_max*3.6:.1f} km/h")
    print(f"Avg Speed:  {R.v_mean*3.6:.1f} km/h")
    Plotters(track_path, R)


Main('Data/KS9E.csv', 'Data/emrax_208_mv.csv', 'Data/Straight.csv')
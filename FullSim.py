import matplotlib.pyplot as plt
import numpy as np  
import pandas as pd
import types
from matplotlib.collections import LineCollection
from time import time
from scipy.optimize import root_scalar
from scipy.integrate import quad

EPS = 1e-9
'''TO DO:
- Add Braking
- Add Pacejka Tire Model
'''
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

def DF(veh, v):
    """Calculates downforce."""
    v2 = v**2
    F_df = 0.5 * veh.rho_air_kgpm3 * v2 * veh.ClA
    return F_df
def Drag(veh, v):
    """Calculates downforce and drag."""
    v2 = v**2
    F_drag = 0.5 * veh.rho_air_kgpm3 * v2 * veh.CdA
    return F_drag
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
    
    return types.SimpleNamespace(**{"Fz_Norm": np.array([
    Fz_f/2 + WT_lat_f/2, # FL
    Fz_f/2 - WT_lat_f/2, # FR
    Fz_r/2 + WT_lat_r/2, # RL
    Fz_r/2 - WT_lat_r/2  # RR
    ])})
def Fz_normalized(veh, v, ax, ay):
    Fz = NormalForce(veh,ax,ay,DF(veh,v)).Fz_Norm
    return veh.fz_ref_N**(veh.load_sens_exp)*Fz**(1-veh.load_sens_exp)
def SlipAngle(veh,R,v):
    pass
def SlipRatio(veh,R,v):
    pass
def PacejkaFy_Max(veh, ax, ay, v):
    S_a = 5 
    D = veh.mu_lat_0 * Fz_normalized(veh,v,ax,ay)
    C = 1.3
    B = veh.Cor_stif/(C*D)
    E = .97
    return D*np.sin(C*np.arctan(B*S_a - E*(B*S_a - np.arctan(B*S_a))))

def PacejkaFx_Max(veh, ax,ay,v):   
    S_r= 14
    D = veh.mu_long_0 * Fz_normalized(veh,v,ax,ay)
    C = 1.3
    B = veh.Cor_stif/(C*D)
    E = .97
    return D*np.sin(C*np.arctan(B*S_r - E*(B*S_r - np.arctan(B*S_r))))

def Vmax (veh,R,ax,ay):
    def Frontfunc(v):
        return R*np.sum(PacejkaFy_Max(veh,ax,ay,v)[:2])/(veh.m_total_kg*(1-veh.swd_frac))-v**2
    def Rearfunc(v):
        return R*np.sum(PacejkaFy_Max(veh,ax,ay,v)[2:4])/(veh.m_total_kg*veh.swd_frac)-v**2
    Front_Vmax = root_scalar(Frontfunc, bracket=[0, 1e3], method='brentq',).root
    Rear_Vmax = root_scalar(Rearfunc, bracket=[0, 1e3], method='brentq').root

    return min(Front_Vmax,Rear_Vmax)
def MotorForceLimitsSingle(motor, v, r_tire, Fx_cap_rear, P_pack_limit):
    """Determines powertrain output based on motor, tire, and battery limits."""
    omega_wheel = v / r_tire
    omega = omega_wheel * motor.n_gear
    
    T_max = {
    'T_wheel_max': motor.tmax_nm * motor.n_gear * motor.tran_eff * motor.pack_eff,
    'T_pack_max': motor.tmax_nm * motor.n_gear * motor.tran_eff * motor.pack_eff if v < EPS else P_pack_limit*motor.tran_eff*motor.pack_eff / omega_wheel,
    'T_grip_max': np.sum(Fx_cap_rear) * r_tire
    }
    Limit_val = np.array(list(T_max.values()))
    T_rear_total = np.min(Limit_val)
    Limiter = np.array(list(T_max.keys()))[np.argmin(Limit_val)]
    T_used = (T_rear_total) / (motor.n_gear * motor.tran_eff* motor.pack_eff)
    I_phase = T_used / motor.kt_nm_per_a
    P_mech = T_rear_total * omega_wheel
    V_back = motor.ke_v_per_rad * omega
    P_elec = P_mech / motor.tran_eff + (I_phase**2 * motor.r_phase_ohm) + (V_back * motor.i0_a)

    return T_rear_total/r_tire, P_elec, Limiter, I_phase, omega*30/np.pi
def Fx_cap(veh,v,ax,ay):
    return PacejkaFx_Max(veh,ax,ay,v)*np.sqrt(1-((veh.m_total_kg*ay*Fz_normalized(veh, v, ax, ay)/np.sum(Fz_normalized(veh, v, ax, ay)))/PacejkaFy_Max(veh,ax,ay,v))**2)

def brakedistance(veh,v_curr,v_next,ax,ay):
    def BrakeForce(veh,v,ax,ay):
        return np.sum(Fx_cap(veh,v,ax,ay))+Drag(veh,v)
    def integrand(v,veh):
        integrand = veh.m_total_kg*v/BrakeForce(veh,v,ax,ay)
        return integrand
    integral = quad(integrand,v_next,v_curr,args=(veh))[0]
    Force = BrakeForce(veh,v_curr,ax,ay)
    return integral, Force

def SimulateSegment(state, seg, veh, motor):
    """Simulates one segment of the track (straight or corner)."""
    dt = veh.dt_s
    
    max_steps = int(min(2e6, (seg.L / np.maximum(state.v, 0.5)) / dt * 2 + 1000))
    t_vec, s_vec, v_vec = [np.zeros(max_steps) for _ in range(3)]
    ax_vec, ay_vec, Fx_vec = [np.zeros(max_steps) for _ in range(3)]
    P_elec_vec, P_mech_vec, P_batt_vec, Fy_vec, P_drag_vec,omega_vec = [np.zeros(max_steps) for _ in range(6)]
    Limiter_vec = []

    v, s, t, E_Wh, idx, ax,ay = float(state.v), 0.0, 0.0, 0.0, 0, state.ax, 0.0

    while s < seg.L and idx < max_steps:
        is_cornering = seg.R > EPS

        if is_cornering:
            v_max_curr = Vmax(veh, seg.R, ax, ay)
            if v > v_max_curr:
                v = v_max_curr
                ax = 0   
            ay = v**2 / seg.R
        else:
            ay = 0.0
        braking_required = False
        if seg.Rnext > EPS:
            v_max_next = Vmax(veh, seg.Rnext, ax, ay)
            if v_max_next < v:
                req_dist, req_brake_force = brakedistance(veh, v, v_max_next, ax, ay)
                if (seg.L - s) < req_dist:
                    braking_required = True
                    Limiter = "Braking"
                    P_elec_motor = 0.0
                    ax = -req_brake_force / veh.m_total_kg
                    Fx_total = (ax * veh.m_total_kg) + Drag(veh, v)
        if not braking_required:
            Fx_limits = Fx_cap(veh, v, ax, ay)[2:4]
            Fx_total, P_elec_motor, Limiter, _, omega = MotorForceLimitsSingle(
                motor, 
                v, 
                veh.r_tire_m, 
                Fx_limits, 
                veh.power_limit_W
            )
            ax = (Fx_total - Drag(veh, v)) / veh.m_total_kg
        if omega > motor.rpm_max:
            ax = 0.0
        t_vec[idx], s_vec[idx], v_vec[idx] = t, s, v
        ax_vec[idx], ay_vec[idx], Fx_vec[idx] = ax, ay, Fx_total
        Fy_vec[idx], P_elec_vec[idx], P_mech_vec[idx] = np.sum(veh.m_total_kg*ay*Fz_normalized(veh, v, ax, ay)), P_elec_motor, Fx_total * v
        P_drag_vec[idx] = Drag(veh,v) * v
        omega_vec[idx] = omega
        Limiter_vec.append(Limiter)
        Vpack = np.maximum(veh.pack_voltage_v, 12.0)
        I_pack = P_elec_motor / Vpack
        P_loss = (I_pack**2) * veh.r_internal_ohm
        P_pack = (P_elec_motor + P_loss)
        P_batt_vec[idx] = P_pack
        
        E_Wh += (P_pack * dt) / 3600.0
        s += v * dt + 0.5 * ax * dt**2
        v += ax * dt
        t += dt
        idx += 1

    return (t_vec[:idx], s_vec[:idx], v_vec[:idx], ax_vec[:idx], ay_vec[:idx], 
            P_elec_vec[:idx], P_mech_vec[:idx], P_drag_vec[:idx], P_batt_vec[:idx], 
            Fx_vec[:idx], Fy_vec[:idx], E_Wh, omega_vec[:idx], np.array(Limiter_vec))

def SimulateLap(track, veh, motor):
    """Aggregates multiple segments into a full lap simulation."""
    res = {k: np.array([]) for k in ["time_s", "s_path", "v", "a_long", "a_lat", "P_elec", "P_mech", "P_drag", "P_batt_loss", "Fx_total", "Fy_total", "omega", "Limiter"]}
    E_Wh, t0, current_s = 0.0, 0.0, 0.0
    state = types.SimpleNamespace(v=0.0, s=0.0, ay =0.0, ax=0.0)

    for k in range(1, track.shape[0]):
        seg = types.SimpleNamespace(R=np.abs(np.float64(track[k][1])), L=np.float64(track[k][2]), Left = 2*(np.float64(track[k][1]) > 0)-1, Rnext = np.abs(np.float64(track[min(k+1, len(track)-1)][1])))
        t, s, v, ax, ay, P_elec, P_mech, P_drag, P_batt, Fx, Fy, Eseg, omega, Limiter = SimulateSegment(state, seg, veh, motor)
        res["time_s"] = np.append(res["time_s"], t0 + t)
        res["s_path"] = np.append(res["s_path"], current_s + s)
        res["v"], res["a_long"], res["a_lat"] = np.append(res["v"], v), np.append(res["a_long"], ax), np.append(res["a_lat"], ay)
        res["P_elec"], res["P_mech"], res["P_drag"] = np.append(res["P_elec"], P_elec), np.append(res["P_mech"], P_mech), np.append(res["P_drag"], P_drag)
        res["P_batt_loss"], res["Fx_total"], res["Fy_total"], res["omega"], res["Limiter"] = np.append(res["P_batt_loss"], P_batt), np.append(res["Fx_total"], Fx), np.append(res["Fy_total"], Fy), np.append(res["omega"], omega), np.append(res["Limiter"], Limiter)
        E_Wh += Eseg
        state.v = v[-1]
        state.ax = ax[-1]
        state.ay = ay[-1]
        current_s += seg.L
        t0 += t[-1]

    res.update({"lap_time_s": t0, "energy_used_Wh": E_Wh, "v_max": np.max(res["v"]), "v_mean": np.mean(res["v"])})
    return types.SimpleNamespace(**res)

# --- Visualization ---
def Plotters(track_path, r):
    """Generates kinematics, power flow, and track velocity map."""
    plt.style.use('bmh')
    fig1, axs = plt.subplots(5, 1, figsize=(10, 8), sharex=True)
    axs[0].plot(r.time_s, r.v); axs[0].set_ylabel('Speed (m/s)')
    axs[1].plot(r.time_s, r.a_long); axs[1].set_ylabel('ax (m/s^2)')
    axs[2].plot(r.time_s, r.a_lat); axs[2].set_ylabel('ay (m/s^2)'); axs[2].set_xlabel('Time (s)')
    axs[3].plot(r.time_s, r.omega); axs[3].set_ylabel('omega (rpm)'); axs[3].set_xlabel('Time (s)')
    limiter_colors = {'T_wheel_max': 'red', 'T_pack_max': 'blue', 'T_grip_max': 'green', 'Braking': 'black'}
    points = np.array([r.time_s, r.Fx_total]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    colors = [limiter_colors[l] for l in r.Limiter[:-1]]
    lc = LineCollection(segments, colors=colors, linewidth=2)
    axs[4].add_collection(lc)
    axs[4].autoscale()
    axs[4].set_ylabel("Fx (N)")
    axs[4].set_xlabel("Time (s)")
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
        s = np.linspace(0, L, 100)
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
    print(f"\n--- Simulation Results ({track_path}) ---")
    print(f"Lap Time:   {R.lap_time_s:.2f} s")
    print(f"Lap Time range: {R.lap_time_s*0.90:.2f} s - {R.lap_time_s*1.1:.2f} s")
    print(f"Energy:     {R.energy_used_Wh/1000:.3f} kWh")
    print(f"Max Speed:  {R.v_max*3.6:.2f} km/h")
    print(f"Avg Speed:  {R.v_mean*3.6:.2f} km/h")
    Plotters(track_path, R)


Main('Data/KS9E.csv', 'Data/emrax_208_mv.csv', 'Data/enduro_24.csv')
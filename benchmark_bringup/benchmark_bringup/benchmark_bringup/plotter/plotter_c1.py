#!/usr/bin/env python3
import os
import math
import time
import csv
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node

# --- ROS msgs ---
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from pid_msgs.msg import PidParams

# --- Plotting ---
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec


def yaw_from_quat(qx, qy, qz, qw):
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

def trapz_integral(t, y_abs=None, y=None, weight_time=False):
    """Integración trapezoidal robusta (usa longitudes coincidentes)."""
    if len(t) < 2:
        return 0.0
    t_arr = np.asarray(t)
    if y_abs is not None:
        v = np.asarray(y_abs)
    elif y is not None:
        v = np.asarray(y)
    else:
        return 0.0
    if weight_time:
        v = np.asarray(y_abs) * t_arr[:len(y_abs)]
    # --- asegurar longitudes iguales ---
    n = min(len(t_arr), len(v))
    t_arr = t_arr[-n:]
    v = v[-n:]
    return float(np.trapz(v, t_arr))


def su_integral(t, u, abs_u=False):
    """ISU / IAU: maneja diferencias de tamaño."""
    if len(t) < 2 or len(u) < 2:
        return 0.0
    t_arr = np.asarray(t)
    u_arr = np.asarray(u)
    n = min(len(t_arr), len(u_arr))
    t_arr = t_arr[-n:]
    u_arr = u_arr[-n:]
    v = np.abs(u_arr) if abs_u else (u_arr ** 2)
    return float(np.trapz(v, t_arr))


def sdu_integral(t, u):
    """ISDU = ∫ (du/dt)^2 dt con ajuste de longitud."""
    if len(t) < 3 or len(u) < 3:
        return 0.0
    t_arr = np.asarray(t)
    u_arr = np.asarray(u)
    n = min(len(t_arr), len(u_arr))
    t_arr = t_arr[-n:]
    u_arr = u_arr[-n:]
    dt = np.diff(t_arr)
    du = np.diff(u_arr)
    dt = np.where(dt <= 1e-9, 1e-9, dt)
    dudt_sq = (du / dt) ** 2
    return float(np.sum(dudt_sq * dt))



class PlotterHierarchical(Node):
    def __init__(self):
        super().__init__('plotter_hierarchical')

        # ----------------- Buffers PID (para subplots) -----------------
        maxlen = 5000
        self.t_pid = deque(maxlen=maxlen)
        self.sp_vel_l = deque(maxlen=maxlen)
        self.sp_vel_r = deque(maxlen=maxlen)
        self.act_vel_l = deque(maxlen=maxlen)
        self.act_vel_r = deque(maxlen=maxlen)
        self.torque_l = deque(maxlen=maxlen)
        self.torque_r = deque(maxlen=maxlen)
        self.error_l = deque(maxlen=maxlen)
        self.error_r = deque(maxlen=maxlen)
        self.pitch_deg = deque(maxlen=maxlen)
        self.disturbance = deque(maxlen=maxlen)

        # Flags de PID (condicionales de gráficas)
        self.enable_antiwindup = False
        self.enable_feedforward = False
        self.enable_referencefilter = False

        # ----------------- Históricos para índices (3 niveles) -----------------
        # Usamos un reloj común a 10 Hz (timer plot) para muestrear señales
        self.t_hist = deque(maxlen=maxlen)

        # High-level
        self.e_high = deque(maxlen=maxlen)   # error pos (m)
        self.u_high = deque(maxlen=maxlen)   # |cmd_vel_nav| (magnitude)

        # Mid-level
        self.e_mid = deque(maxlen=maxlen)    # |cmd_vel_nav - cmd_vel|
        self.u_mid = deque(maxlen=maxlen)    # |cmd_vel|

        # Low-level
        self.e_low = deque(maxlen=maxlen)    # avg error wheels
        self.u_low = deque(maxlen=maxlen)    # avg torque (signed)

        # ----------------- Estado "último valor" de cada topic -----------------
        self.start_time = None
        self.lock = False  # bloqueo ligero para evitar reentrada en callbacks simultáneos

        # AMCL pose
        self.amcl_ok = False
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0

        # Goal pose (desde /received_global_plan como PoseStamped)
        self.goal_ok = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        # Twists
        self.nav_ok = False
        self.nav_v = 0.0
        self.nav_w = 0.0

        self.cmd_ok = False
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        # PID params (errores/torques/pitch/disturbance)
        self.pid_ok = False
        self.last_err_l = 0.0
        self.last_err_r = 0.0
        self.last_torque_l = 0.0
        self.last_torque_r = 0.0
        self.last_pitch_deg = 0.0
        self.last_disturbance = 0.0

        # ----------------- Suscripciones -----------------
        self.sub_pid = self.create_subscription(PidParams, '/pid_params', self.cb_pid, 30)
        self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)
        self.sub_goal = self.create_subscription(Path, '/received_global_plan', self.cb_goal_path, 10)
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.cb_cmd_vel_nav, 20)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 50)

        self.get_logger().info("✅ Subscribed to: /pid_params, /amcl_pose, /received_global_plan, /cmd_vel_nav, /cmd_vel")

        # ----------------- Plot init (4 subplots + panel de métricas) -----------------
        plt.ion()
        self.fig = plt.figure(figsize=(13.5, 8.5), tight_layout=True)

        # Columna izquierda 4 subgráficas; derecha 1 panel de métricas (3 bloques)
        gs = GridSpec(4, 2, width_ratios=[3.2, 1.0], figure=self.fig)
        self.ax = [
            self.fig.add_subplot(gs[0, 0]),
            self.fig.add_subplot(gs[1, 0]),
            self.fig.add_subplot(gs[2, 0]),
            self.fig.add_subplot(gs[3, 0]),
        ]
        self.metrics_ax = self.fig.add_subplot(gs[:, 1])
        self.metrics_ax.axis("off")
        self.fig.canvas.manager.set_window_title("Hierarchical Performance – PID/MPC/Planner")

        # Textos de métricas (tres bloques)
        self.txt = {}
        y0 = 0.92
        dy = 0.06

        self.txt['title'] = self.metrics_ax.text(0.05, y0, "Performance Indices", fontsize=14, fontweight="bold")

        # High-level
        self.metrics_ax.text(0.05, y0 - dy*1.2, "High Level (Planner)", fontsize=12, fontweight="bold", color="#2e7d32")
        self.txt['H_IAE']  = self.metrics_ax.text(0.05, y0 - dy*2.1, "", fontsize=10, color="#2e7d32")
        self.txt['H_ISE']  = self.metrics_ax.text(0.05, y0 - dy*2.8, "", fontsize=10, color="#2e7d32")
        self.txt['H_ITAE'] = self.metrics_ax.text(0.05, y0 - dy*3.5, "", fontsize=10, color="#2e7d32")
        self.txt['H_ISU']  = self.metrics_ax.text(0.05, y0 - dy*4.2, "", fontsize=10, color="#2e7d32")
        self.txt['H_IAU']  = self.metrics_ax.text(0.05, y0 - dy*4.9, "", fontsize=10, color="#2e7d32")
        self.txt['H_ISDU'] = self.metrics_ax.text(0.05, y0 - dy*5.6, "", fontsize=10, color="#2e7d32")

        # Mid-level
        base2 = y0 - dy*6.8
        self.metrics_ax.text(0.05, base2, "Mid Level (MPC)", fontsize=12, fontweight="bold", color="#f9a825")
        self.txt['M_IAE']  = self.metrics_ax.text(0.05, base2 - dy*0.9, "", fontsize=10, color="#f9a825")
        self.txt['M_ISE']  = self.metrics_ax.text(0.05, base2 - dy*1.6, "", fontsize=10, color="#f9a825")
        self.txt['M_ITAE'] = self.metrics_ax.text(0.05, base2 - dy*2.3, "", fontsize=10, color="#f9a825")
        self.txt['M_ISU']  = self.metrics_ax.text(0.05, base2 - dy*3.0, "", fontsize=10, color="#f9a825")
        self.txt['M_IAU']  = self.metrics_ax.text(0.05, base2 - dy*3.7, "", fontsize=10, color="#f9a825")
        self.txt['M_ISDU'] = self.metrics_ax.text(0.05, base2 - dy*4.4, "", fontsize=10, color="#f9a825")

        # Low-level
        base3 = y0 - dy*11.2
        self.metrics_ax.text(0.05, base3, "Low Level (PID)", fontsize=12, fontweight="bold", color="#1565c0")
        self.txt['L_IAE']  = self.metrics_ax.text(0.05, base3 - dy*0.9, "", fontsize=10, color="#1565c0")
        self.txt['L_ISE']  = self.metrics_ax.text(0.05, base3 - dy*1.6, "", fontsize=10, color="#1565c0")
        self.txt['L_ITAE'] = self.metrics_ax.text(0.05, base3 - dy*2.3, "", fontsize=10, color="#1565c0")
        self.txt['L_ISU']  = self.metrics_ax.text(0.05, base3 - dy*3.0, "", fontsize=10, color="#1565c0")
        self.txt['L_IAU']  = self.metrics_ax.text(0.05, base3 - dy*3.7, "", fontsize=10, color="#1565c0")
        self.txt['L_ISDU'] = self.metrics_ax.text(0.05, base3 - dy*4.4, "", fontsize=10, color="#1565c0")

        # Timers
        self.timer_plot = self.create_timer(0.10, self.update_plot)      # 10 Hz: refresco subplots y muestreo señales
        self.timer_metrics = self.create_timer(2.5, self.update_metrics)  # 2.5 s: actualización texto métricas

    # --------------------------- Callbacks topics ---------------------------
    def _now_rel(self):
        t_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = t_sec
        return t_sec - self.start_time

    def cb_pid(self, msg: PidParams):
        if self.lock: return
        self.lock = True
        try:
            t = self._now_rel()

            # buffers de subplot (dependen de PID)
            self.t_pid.append(t)
            self.sp_vel_l.append(msg.sp_vel_l)
            self.sp_vel_r.append(msg.sp_vel_r)
            self.act_vel_l.append(msg.act_vel_l)
            self.act_vel_r.append(msg.act_vel_r)
            self.torque_l.append(msg.torque_l)
            self.torque_r.append(msg.torque_r)
            self.error_l.append(msg.error_l)
            self.error_r.append(msg.error_r)
            self.pitch_deg.append(math.degrees(msg.pitch))
            self.disturbance.append(msg.torque_slope)

            # flags (para condicional de gráficas)
            self.enable_antiwindup = msg.enable_antiwindup
            self.enable_feedforward = msg.enable_feedforward
            self.enable_referencefilter = msg.enable_referencefilter

            # ultimo estado (para muestreo a 10Hz de índices)
            self.pid_ok = True
            self.last_err_l = msg.error_l
            self.last_err_r = msg.error_r
            self.last_torque_l = msg.torque_l
            self.last_torque_r = msg.torque_r
            self.last_pitch_deg = math.degrees(msg.pitch)
            self.last_disturbance = msg.torque_slope
        finally:
            self.lock = False

    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.amcl_x = p.x
        self.amcl_y = p.y
        self.amcl_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.amcl_ok = True

    def cb_goal_path(self, msg: Path):
        """Extrae el último punto del plan global como objetivo."""
        if len(msg.poses) == 0:
            return
        p_last = msg.poses[-1].pose.position
        q_last = msg.poses[-1].pose.orientation
        self.goal_x = p_last.x
        self.goal_y = p_last.y
        self.goal_yaw = yaw_from_quat(q_last.x, q_last.y, q_last.z, q_last.w)
        self.goal_ok = True

    def cb_cmd_vel_nav(self, msg: Twist):
        self.nav_v = msg.linear.x
        self.nav_w = msg.angular.z
        self.nav_ok = True

    def cb_cmd_vel(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.cmd_ok = True

    # --------------------------- Muestreo + Plot ---------------------------
    def sample_signals_for_indices(self, t_now):
        """
        Muestrea y almacena señales para integrar índices (tres niveles).
        Llamado a 10 Hz desde update_plot().
        """
        # Sólo avanzamos tiempo si al menos tenemos PID (para tener referencia de reloj)
        # Pero igualmente añadimos t_now siempre; y para señales que no existan aún, no agregamos muestra.
        self.t_hist.append(t_now)

        # High-level: error = distancia amcl -> goal; u = |cmd_vel_nav|
        if self.amcl_ok and self.goal_ok:
            eH = math.hypot(self.goal_x - self.amcl_x, self.goal_y - self.amcl_y)
            self.e_high.append(eH)
        # Entrada high (si hay cmd_vel_nav)
        if self.nav_ok:
            uH = math.hypot(self.nav_v, self.nav_w)
            self.u_high.append(uH)

        # Mid-level: error = |cmd_vel_nav - cmd_vel|; u = |cmd_vel|
        if self.nav_ok and self.cmd_ok:
            eM = math.hypot(self.nav_v - self.cmd_v, self.nav_w - self.cmd_w)
            self.e_mid.append(eM)
        if self.cmd_ok:
            uM = math.hypot(self.cmd_v, self.cmd_w)
            self.u_mid.append(uM)

        # Low-level: error = avg(error_l, error_r); u = avg torque (signed)
        if self.pid_ok:
            eL = 0.5 * (self.last_err_l + self.last_err_r)
            uL = 0.5 * (self.last_torque_l + self.last_torque_r)
            self.e_low.append(eL)
            self.u_low.append(uL)

        # Limitar tamaños si alguna lista crece más que t_hist (por señales ausentes)
        for q in [self.e_high, self.u_high, self.e_mid, self.u_mid, self.e_low, self.u_low]:
            while len(q) > len(self.t_hist):
                q.popleft()

    def update_plot(self):
        if self.lock:
            return
        self.lock = True
        try:
            # Muestreo de señales para índices a 10Hz
            t = self._now_rel()
            self.sample_signals_for_indices(t)

            # Si no hay suficientes puntos para graficar PID, salimos
            if len(self.t_pid) < 2:
                return

            # Limpia todos los ejes
            for ax in self.ax:
                ax.clear()

            # 1) Angular velocity
            self.ax[0].plot(self.t_pid, self.sp_vel_l, 'r-', label='SP L')
            self.ax[0].plot(self.t_pid, self.act_vel_l, 'r--', label='ACT L')
            self.ax[0].plot(self.t_pid, self.sp_vel_r, 'b-', label='SP R')
            self.ax[0].plot(self.t_pid, self.act_vel_r, 'b--', label='ACT R')
            self.ax[0].set_ylabel("Angular velocity (ω) [rad/s]")
            self.ax[0].legend(fontsize=8)
            self.ax[0].grid(True, alpha=0.3)

            # 2) Input torque
            self.ax[1].plot(self.t_pid, self.torque_l, 'r-', label='Torque L')
            self.ax[1].plot(self.t_pid, self.torque_r, 'b-', label='Torque R')
            self.ax[1].set_ylabel("Input torque [Nm]")
            self.ax[1].legend(fontsize=8)
            self.ax[1].grid(True, alpha=0.3)

            # 3) Error
            self.ax[2].plot(self.t_pid, self.error_l, 'r-', label='Error L')
            self.ax[2].plot(self.t_pid, self.error_r, 'b-', label='Error R')
            self.ax[2].set_ylabel("Error ω [rad/s]")
            self.ax[2].legend(fontsize=8)
            self.ax[2].grid(True, alpha=0.3)
            if not self.enable_feedforward:
                self.ax[2].set_xlabel("Time [s]")

            # 4) Condicional (feedforward): Pitch & Disturbance
            if self.enable_feedforward:
                self.ax[3].plot(self.t_pid, self.pitch_deg, 'g-', label='Inclination [°]')
                self.ax[3].plot(self.t_pid, self.disturbance, 'm-', label='Disturbance torque [Nm]')
                self.ax[3].set_ylabel("Inclination [°] / Disturbance [Nm]")
                self.ax[3].set_xlabel("Time [s]")
                self.ax[3].legend(fontsize=8)
                self.ax[3].grid(True, alpha=0.3)
            else:
                # Si no hay feedforward, mostramos sólo pitch en el 4º panel por continuidad visual
                self.ax[3].plot(self.t_pid, self.pitch_deg, 'g-', label='Inclination [°]')
                self.ax[3].set_ylabel("Inclination [°]")
                self.ax[3].set_xlabel("Time [s]")
                self.ax[3].legend(fontsize=8)
                self.ax[3].grid(True, alpha=0.3)

            self.fig.tight_layout(rect=[0, 0, 0.80, 1.0])  # dejar espacio a la derecha
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        finally:
            self.lock = False

    # --------------------------- Métricas (cada 2.5s) ---------------------------
    def compute_all_indices(self):
        """
        Calcula todos los índices (IAE, ISE, ITAE, ISU, IAU, ISDU, MSAE, MSIC, J)
        para los tres niveles: Planner (High), MPC (Mid) y PID (Low).
        Incluye normalización física de errores y entradas.
        """

        # --- Parámetros físicos ---
        V_MAX = 0.75       # [m/s]
        W_MAX = 3.5        # [rad/s]
        E_SCALE_H = 1.0    # [m]   (0.25% de 40 m)
        T_SCALE_L = 50.0   # [Nm]
        U_SCALE = math.sqrt(V_MAX**2 + W_MAX**2)  # ~3.58

        def align(t, *signals):
            """Alinea señales al mismo tamaño."""
            L = len(t)
            sigs = []
            for s in signals:
                if len(s) < 2:
                    sigs.append(None)
                else:
                    s2 = list(s)[-L:]
                    sigs.append(s2)
            return list(t), sigs

        results = {}

        # ---------------- High Level (Planner) ----------------
        tH, (eH, uH) = align(self.t_hist, self.e_high, self.u_high)
        if eH and uH:
            eH = np.array(eH)
            uH = np.array(uH)

            # Normalizaciones
            eH_n = eH / E_SCALE_H
            uH_n = uH / U_SCALE

            # Integrales clásicas
            IAE = trapz_integral(tH, y_abs=np.abs(eH))
            ISE = trapz_integral(tH, y=(eH**2))
            ITAE = trapz_integral(tH, y_abs=np.abs(eH), weight_time=True)
            ISU = su_integral(tH, u=uH, abs_u=False)
            IAU = su_integral(tH, u=uH, abs_u=True)
            ISDU = sdu_integral(tH, u=uH)

            # Índices normalizados
            MSAE = np.mean(np.abs(eH_n))
            duH = np.diff(uH_n)
            MSIC = np.mean(duH**2) if len(duH) > 0 else 0.0
            J = MSAE + MSIC

            results['H'] = (IAE, ISE, ITAE, ISU, IAU, ISDU, MSAE, MSIC, J)
        else:
            results['H'] = None

        # ---------------- Mid Level (MPC) ----------------
        tM, (eM, uM) = align(self.t_hist, self.e_mid, self.u_mid)
        if eM and uM:
            eM = np.array(eM)
            uM = np.array(uM)

            # Normalizaciones
            eM_n = eM / U_SCALE
            uM_n = uM / U_SCALE

            # Integrales clásicas
            IAE = trapz_integral(tM, y_abs=np.abs(eM))
            ISE = trapz_integral(tM, y=(eM**2))
            ITAE = trapz_integral(tM, y_abs=np.abs(eM), weight_time=True)
            ISU = su_integral(tM, u=uM, abs_u=False)
            IAU = su_integral(tM, u=uM, abs_u=True)
            ISDU = sdu_integral(tM, u=uM)

            # Índices normalizados
            MSAE = np.mean(np.abs(eM_n))
            duM = np.diff(uM_n)
            MSIC = np.mean(duM**2) if len(duM) > 0 else 0.0
            J = MSAE + MSIC

            results['M'] = (IAE, ISE, ITAE, ISU, IAU, ISDU, MSAE, MSIC, J)
        else:
            results['M'] = None

        # ---------------- Low Level (PID) ----------------
        tL, (eL, uL) = align(self.t_hist, self.e_low, self.u_low)
        if eL and uL:
            eL = np.array(eL)
            uL = np.array(uL)

            # Normalizaciones
            eL_n = eL / W_MAX
            uL_n = uL / T_SCALE_L

            # Integrales clásicas
            IAE = trapz_integral(tL, y_abs=np.abs(eL))
            ISE = trapz_integral(tL, y=(eL**2))
            ITAE = trapz_integral(tL, y_abs=np.abs(eL), weight_time=True)
            ISU = su_integral(tL, u=uL, abs_u=False)
            IAU = su_integral(tL, u=uL, abs_u=True)
            ISDU = sdu_integral(tL, u=uL)

            # Índices normalizados
            MSAE = np.mean(np.abs(eL_n))
            duL = np.diff(uL_n)
            MSIC = np.mean(duL**2) if len(duL) > 0 else 0.0
            J = MSAE + MSIC

            results['L'] = (IAE, ISE, ITAE, ISU, IAU, ISDU, MSAE, MSIC, J)
        else:
            results['L'] = None

        return results


    def update_metrics(self):
        res = self.compute_all_indices()

        # --- Formateador genérico para High, Mid y Low ---
        def fmt_indices(block, vals):
            if vals is None:
                self.txt[f'{block}_IAE'].set_text("MSAE = N/A")
                self.txt[f'{block}_ISE'].set_text("MSIC = N/A")
                self.txt[f'{block}_ITAE'].set_text("J = N/A")
            else:
                MSAE = vals[6]
                MSIC = vals[7]
                J = vals[8]
                self.txt[f'{block}_IAE'].set_text(f"MSAE = {MSAE:.4f}")
                self.txt[f'{block}_ISE'].set_text(f"MSIC = {MSIC:.4f}")
                self.txt[f'{block}_ITAE'].set_text(f"J = {J:.4f}")

            # Limpia los otros campos visuales del bloque
            for key in ['ISU', 'IAU', 'ISDU']:
                if f'{block}_{key}' in self.txt:
                    self.txt[f'{block}_{key}'].set_text("")

        # Mostrar MSAE / MSIC / J
        fmt_indices('H', res.get('H'))
        fmt_indices('M', res.get('M'))
        fmt_indices('L', res.get('L'))

        self.fig.canvas.draw_idle()


    # --------------------------- Guardado CSV en shutdown ---------------------------
    def save_csv_on_exit(self):
        res = self.compute_all_indices()

        # asegurar carpeta ./result
        out_dir = os.path.join(os.getcwd(), 'result')
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(out_dir, f'performance_{ts}.csv')

        header = [
            'Level', 'IAE', 'ISE', 'ITAE', 'ISU', 'IAU', 'ISDU',
            'MSAE', 'MSIC', 'J',
            'feedforward_enabled', 'antiwindup_enabled', 'referencefilter_enabled'
        ]
        rows = []

        def row_from(level_key, vals):
            if vals is None:
                return [level_key] + ['N/A'] * 9 + [
                    self.enable_feedforward, self.enable_antiwindup, self.enable_referencefilter
                ]
            IAE, ISE, ITAE, ISU, IAU, ISDU, MSAE, MSIC, J = vals
            return [
                level_key, f'{IAE:.6f}', f'{ISE:.6f}', f'{ITAE:.6f}',
                f'{ISU:.6f}', f'{IAU:.6f}', f'{ISDU:.6f}',
                f'{MSAE:.6f}', f'{MSIC:.6f}', f'{J:.6f}',
                self.enable_feedforward, self.enable_antiwindup, self.enable_referencefilter
            ]

        rows.append(row_from('High', res.get('H')))
        rows.append(row_from('Mid',  res.get('M')))
        rows.append(row_from('Low',  res.get('L')))

        with open(out_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)

        self.get_logger().info(f"💾 Saved performance CSV: {out_path}")



# ============================== MAIN ==============================
def main():
    rclpy.init()
    node = PlotterHierarchical()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down plotter...")
    finally:
        try:
            node.save_csv_on_exit()
        except Exception as e:
            node.get_logger().error(f"Error saving CSV: {e}")
        node.destroy_node()
        rclpy.shutdown()
        try:
            plt.close('all')
        except Exception:
            pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Pure Pursuit con modelo Ackermann usando datos del OptiTrack (QCar físico)
# Versión combinada: ROS2 + lógica Pure Pursuit estilo TargetCourse + análisis y gráficas extendidas
# fusion Ivan y Mariana 27 Junio 2025

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

import math
import csv
from pathlib import Path
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt


# ==========================
# CLASES AUXILIARES (PPState, TargetCourse)
# ==========================

class PPState:
    """Estado mínimo del vehículo para Pure Pursuit."""
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.xr = x
        self.yr = y
        self.theta = yaw
        self.v = v

    def calc_distance(self, px, py):
        dx = self.xr - px
        dy = self.yr - py
        return math.hypot(dx, dy)


class TargetCourse:
    """Trayectoria + búsqueda del índice objetivo como en el script de simulación."""
    def __init__(self, path_points, k_gain, base_lookahead):
        self.cx = [p[0] for p in path_points]
        self.cy = [p[1] for p in path_points]
        self.k = k_gain
        self.Lfc = base_lookahead
        self.old_nearest_point_index = None

    def search_target_index(self, state: PPState):
        """
        Implementa la misma lógica del código de Ivan:
        1) Busca punto más cercano
        2) Desde ahí avanza mientras la distancia disminuya
        3) Calcula Lf dinámico = k * v + Lfc
        4) Busca el punto al menos a Lf de distancia
        """
        if self.old_nearest_point_index is None:
            # Primera vez: buscar en toda la trayectoria
            dists = [
                math.hypot(state.xr - cx_i, state.yr - cy_i)
                for cx_i, cy_i in zip(self.cx, self.cy)
            ]
            ind = int(np.argmin(dists))
            self.old_nearest_point_index = ind
        else:
            # Reutilizamos índice anterior y buscamos hacia adelante
            ind = self.old_nearest_point_index
            distance_this = state.calc_distance(self.cx[ind], self.cy[ind])

            # Avanzar mientras ir adelante acerque más al robot
            while ind + 1 < len(self.cx):
                distance_next = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if distance_this < distance_next:
                    break
                ind += 1
                distance_this = distance_next

            self.old_nearest_point_index = ind

        # Lookahead dinámico
        v = max(state.v, 0.0)
        Lf = self.k * v + self.Lfc

        # Buscar punto objetivo con al menos distancia Lf
        target_ind = ind
        while target_ind < len(self.cx) - 1:
            dist = state.calc_distance(self.cx[target_ind], self.cy[target_ind])
            if dist >= Lf:
                break
            target_ind += 1

        return target_ind, Lf


# ==========================
# NODO PURE PURSUIT ROS2
# ==========================

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ----------------- PARÁMETROS -----------------
        self.declare_parameter('path_csv', '')
        # Parámetros para generar trayectoria circular (si no se usa CSV)
        self.declare_parameter('circle_radius', 0.8)      # radio del círculo (m)
        self.declare_parameter('circle_points', 300)      # número de puntos

        # Parámetros de Pure Pursuit (equivalentes al segundo script)
        self.declare_parameter('lookahead', 0.15)         # Lfc
        self.declare_parameter('k_gain', 0.4)             # k (look forward gain)
        self.declare_parameter('v_ref', 0.05)             # m/s

        # Ackermann
        self.declare_parameter('wheelbase', 0.256)
        self.declare_parameter('max_steer_deg', 25.0)
        self.declare_parameter('max_omega', 2.0)

        path_csv = self.get_parameter('path_csv').get_parameter_value().string_value
        self.lookahead = self.get_parameter('lookahead').get_parameter_value().double_value
        self.v_ref = self.get_parameter('v_ref').get_parameter_value().double_value
        self.k_gain = self.get_parameter('k_gain').get_parameter_value().double_value

        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
        max_steer_deg = self.get_parameter('max_steer_deg').get_parameter_value().double_value
        self.max_steer = math.radians(max_steer_deg)
        self.max_omega = self.get_parameter('max_omega').get_parameter_value().double_value

        # Límite de steering del QCar (user_command.y)
        self.max_steer_cmd = 0.5  # rad

        # Timeout de pose
        #self.pose_timeout = 0.2  # [s] sin recibir /qcar/pose -> STOP

        # ----------------- CARGA DEL PATH -----------------
        if path_csv.strip():
            self.path = self.load_path(path_csv)
        else:
            self.path = self.generate_circle_path()

        if not self.path:
            self.get_logger().error('Path vacío, revisa el CSV o los parámetros del círculo')
        else:
            self.get_logger().info(f'Path cargado/generado con {len(self.path)} puntos')
            self.get_logger().info(f"Primeros 5 puntos path: {self.path[:5]}")

        # TargetCourse con lógica del segundo script
        if self.path:
            self.target_course = TargetCourse(self.path, self.k_gain, self.lookahead)
        else:
            self.target_course = None

        # ESTADO
        self.current_pose = None
        self.state = PPState()
        self.target_ind = 0
        self.generated_trajectory = []
        self.last_objective = None

        # Logs adicionales para gráficas (estilo segundo script)
        self.log_t = []
        self.log_x = []
        self.log_y = []
        self.log_yaw = []
        self.log_v = []

        # Tiempo de inicio
        self.start_time = self.get_clock().now()

        # Para timeout de pose
        self.last_pose_time = self.get_clock().now()

        # Subscripciones / Publicaciones
        self.pose_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/pose',
            self.pose_callback,
            10
        )

        self.pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/user_command',
            10
        )

        # Timer 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("PurePursuitNode físico (versión combinada) iniciado.")

    # ----------------- CARGA DEL PATH (/qcar/pose) -----------------
    def load_path(self, csv_file):
        path_points = []
        if not csv_file:
            self.get_logger().warn('No se especificó path_csv')
            return path_points

        p = Path(csv_file)
        if not p.exists():
            self.get_logger().error(f'CSV no encontrado: {csv_file}')
            return path_points

        with p.open('r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    x = float(row['x'])
                    y = float(row['y'])
                    path_points.append((x, y))
                except (KeyError, ValueError):
                    continue

        self.get_logger().info(f'Path simple cargado con {len(path_points)} puntos')
        return path_points
    
    #  # ----------------- CARGA DEL PATH (OptiTrack) -----------------
    # def load_path(self, csv_file):
    #     path_points = []
    #     if not csv_file:
    #         self.get_logger().warn('No se especificó path_csv')
    #         return path_points

    #     p = Path(csv_file)
    #     if not p.exists():
    #         self.get_logger().error(f'CSV no encontrado: {csv_file}')
    #         return path_points

    #     with p.open('r') as f:
    #         reader = csv.reader(f)

    #         # Saltar encabezado de Motive (igual que antes)
    #         for _ in range(7):
    #             next(reader, None)

    #         for row in reader:
    #             if not row or len(row) < 8:
    #                 continue
    #             try:
    #                 X_opt = float(row[5])
    #                 Z_opt = float(row[7])

    #                 # Mapeo OptiTrack -> ROS (ajusta signo según cómo lo veas en RViz)
    #                 x_ros = Z_opt
    #                 y_ros = X_opt  # si necesitas invertir, usar y_ros = -X_opt

    #                 path_points.append((x_ros, y_ros))
    #             except ValueError:
    #                 continue

    #     self.get_logger().info(f'Path cargado con {len(path_points)} puntos (load_path)')
    #     return path_points

    def generate_circle_path(self):
        R = self.get_parameter('circle_radius').get_parameter_value().double_value
        N = int(self.get_parameter('circle_points').get_parameter_value().integer_value)
        cx = 0.0
        cy = R

        if N < 3:
            self.get_logger().warn("circle_points < 3, usando N=3 por seguridad.")
            N = 3

        path_points = []
        for i in range(N):
            theta = 2.0 * math.pi * i / N  # [0, 2pi)
            x = cx + R * math.cos(theta)
            y = cy + R * math.sin(theta)
            path_points.append((x, y))

        self.get_logger().info(
            f"Trayectoria circular generada: R={R:.3f} m, centro=({cx:.3f}, {cy:.3f}), N={N}"
        )
        return path_points

    # ----------------- CALLBACK POSE -----------------
    def pose_callback(self, msg: Vector3Stamped):
        self.current_pose = msg
        self.last_pose_time = self.get_clock().now()

    # ----------------- CONTROL LOOP (ACKERMANN + PP) -----------------
    def control_loop(self):

        now = self.get_clock().now()

        # 1) Si no hay pose o no hay path -> no hacemos nada
        if self.current_pose is None or not self.path or self.target_course is None:
            return

        # # Timeout de pose
        # dt_pose = (now.nanoseconds - self.last_pose_time.nanoseconds) * 1e-9
        # if dt_pose > self.pose_timeout:
        #     self.get_logger().warn_once("Timeout de /qcar/pose, deteniendo QCar.")
        #     self.stop_qcar()
        #     return

        # Extraer pose actual
        x_r = self.current_pose.vector.x
        y_r = self.current_pose.vector.y
        yaw = self.current_pose.vector.z   # yaw del imu en rad

        # Actualizar estado para Pure Pursuit (usamos v_ref como velocidad "actual")
        self.state.xr = x_r
        self.state.yr = y_r
        self.state.theta = yaw
        self.state.v = abs(self.v_ref)

        # ---- Pure Pursuit con TargetCourse (segundo script) ----
        delta = self.compute_pure_pursuit_delta()

        # Modelo bicicleta -> velocidad angular equivalente (solo info)
        w_cmd = (self.v_ref / self.L) * math.tan(delta)
        if abs(w_cmd) > self.max_omega:
            w_cmd = math.copysign(self.max_omega, w_cmd)

        # Saturar steering al rango del QCar
        steer_cmd = delta
        if steer_cmd > self.max_steer_cmd:
            steer_cmd = self.max_steer_cmd
        elif steer_cmd < -self.max_steer_cmd:
            steer_cmd = -self.max_steer_cmd

        # Publicar comando
        cmd = Vector3Stamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.vector.x = float(self.v_ref)    # velocidad adelante
        cmd.vector.y = float(-steer_cmd)     # steering [-0.3, 0.3] rad
        cmd.vector.z = 0.0

        self.pub.publish(cmd)

        # Registrar trayectoria (para error, gráficos y análisis)
        self.generated_trajectory.append((x_r, y_r))

        # Logs extendidos
        t = (now.nanoseconds - self.start_time.nanoseconds) * 1e-9
        self.log_t.append(t)
        self.log_x.append(x_r)
        self.log_y.append(y_r)
        self.log_yaw.append(yaw)
        self.log_v.append(self.v_ref)

    def compute_pure_pursuit_delta(self):
        """
        Implementa pure_pursuit_steer_control Ivan,
        usando self.state y self.target_course.
        """
        # 1) Índice objetivo y Lf dinámico
        ind, Lf = self.target_course.search_target_index(self.state)

        # No permitir ir hacia atrás en el índice
        if self.target_ind >= ind:
            ind = self.target_ind

        self.target_ind = ind

        # 2) Punto objetivo
        tx = self.target_course.cx[ind]
        ty = self.target_course.cy[ind]
        
        # 3) Distancia al objetivo (solo info)
        dist_to_target = self.state.calc_distance(tx, ty)
        if self.last_objective is None or ind != self.last_objective:
                self.last_objective = ind
                self.get_logger().info(
                    f"Nuevo punto objetivo={ind} | ref=({tx:.3f}, {ty:.3f}) | "
                    f"distancia={dist_to_target:.3f} m | Lf={Lf:.3f} m"
                )

        # 4) Ángulo alpha
        alpha = math.atan2(ty - self.state.yr, tx - self.state.xr) - self.state.theta
        # Normalizar alpha a [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # 5) Steering pure pursuit
        Lf = max(Lf, 1e-3)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), Lf)

        # # 6) Saturar al ángulo máximo geométrico
        # if delta > self.max_steer:
        #     delta = self.max_steer
        # elif delta < -self.max_steer:
        #     delta = -self.max_steer

        return delta

    # ----------------- STOP -----------------
    def stop_qcar(self):
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.vector.x = 0.0
        cmd.vector.y = 0.0
        cmd.vector.z = 0.0
        self.pub.publish(cmd)

    # ----------------- ERROR TRACKING -----------------
    def find_closest_point(self, point, traj):
        min_dist = float('inf')
        closest = None
        for ref in traj:
            d = math.hypot(point[0] - ref[0], point[1] - ref[1])
            if d < min_dist:
                min_dist = d
                closest = ref
        return min_dist, closest

    def calculate_tracking_error(self):
        if not self.generated_trajectory or not self.path:
            return 0.0, 0.0, []

        errors = []
        for gen_point in self.generated_trajectory:
            min_dist, _ = self.find_closest_point(gen_point, self.path)
            errors.append(min_dist)

        return float(np.mean(errors)), float(np.max(errors)), errors

    def calculate_similarity_percentage(self, errors, tolerance=0.05):
        if not errors:
            return 0.0
        return sum(1 for e in errors if e <= tolerance) / len(errors) * 100.0

    def report_performance(self, avg_error, max_error, errors):

        tol1 = 0.05
        tol2 = 0.10

        sim05 = self.calculate_similarity_percentage(errors, tol1)
        sim10 = self.calculate_similarity_percentage(errors, tol2)
        std = float(np.std(errors))

        # Evaluación cualitativa (igual que el primer código)
        if avg_error < 0.05:
            assessment = "EXCELENTE"
            color = "\033[92m"  # Verde
        elif avg_error < 0.10:
            assessment = "BUENO"
            color = "\033[96m"  # Cian
        elif avg_error < 0.20:
            assessment = "ACEPTABLE"
            color = "\033[93m"  # Amarillo
        else:
            assessment = "DEFICIENTE"
            color = "\033[91m"  # Rojo

        reset = "\033[0m"
        bold = "\033[1m"

        print("\n========== EVALUACIÓN DE TRAJECTORIA ==========")
        print(f"Error promedio:            {avg_error:.3f} m")
        print(f"Error máximo:              {max_error:.3f} m")
        print(f"Desviación estándar:       {std:.3f} m")
        print(f"% dentro de 5cm:           {sim05:.1f}%")
        print(f"% dentro de 10cm:          {sim10:.1f}%")
        print(f"Trayectoria referencia:    {len(self.path)} puntos")
        print(f"Trayectoria real:          {len(self.generated_trajectory)} puntos")
        print(f"\n{bold}Evalución general: {reset} {color}{assessment}{reset}")
        print("================================================\n")

    # ----------------- GUARDADO Y GRÁFICAS -----------------
    def save_trajectory_csv(self, errors, out_dir, timestamp, tag):

        csv_path = out_dir / f"trayectoria_{tag}_{timestamp}.csv"

        with csv_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 't', 'x_real', 'y_real', 'yaw', 'v_cmd',
                             'x_ref', 'y_ref', 'error'])

            for i, (xr, yr) in enumerate(self.generated_trajectory):
                t = self.log_t[i] if i < len(self.log_t) else ''
                yaw = self.log_yaw[i] if i < len(self.log_yaw) else ''
                v_cmd = self.log_v[i] if i < len(self.log_v) else ''
                err = errors[i] if i < len(errors) else ''

                _, closest = self.find_closest_point((xr, yr), self.path)
                if closest:
                    xc, yc = closest
                else:
                    xc, yc = '', ''
                writer.writerow([i, t, xr, yr, yaw, v_cmd, xc, yc, err])

        print(f"CSV guardado en: {csv_path}")

    def plot_results(self):

        if not self.path or not self.generated_trajectory:
            print("No hay datos para graficar.")
            return

        exp_x = [p[0] for p in self.path]
        exp_y = [p[1] for p in self.path]
        gen_x = [p[0] for p in self.generated_trajectory]
        gen_y = [p[1] for p in self.generated_trajectory]

        avg_error, max_error, errors = self.calculate_tracking_error()
        if errors:
            self.report_performance(avg_error, max_error, errors)

        # Carpeta de salida
        out_dir = Path.home() / 'qcar_workspace' / 'OptiTrack' / 'src' / 'resultados' / 'pure_pursuit_fisico'
        out_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = f"L{self.lookahead}_V{self.v_ref}_k{self.k_gain}"

        # -------- FIGURA 1: Trayectoria + Error (como el primer código) --------
        fig1 = plt.figure(figsize=(12, 5))
        ax1 = fig1.add_subplot(1, 2, 1)
        ax1.plot(exp_x, exp_y, label="Referencia", linewidth=2)
        ax1.plot(gen_x, gen_y, label="Real", linewidth=2)
        if len(gen_x) > 1:
            ax1.scatter(gen_x[0], gen_y[0], s=120, c='green', marker='o', label='Inicio real')
            ax1.scatter(gen_x[-1], gen_y[-1], s=120, c='red', marker='x', label='Final real')

        ax1.axis('equal')
        ax1.grid(True)
        ax1.legend()
        ax1.set_title("Trayectoria referencia vs real")
        ax1.set_xlabel("X [m]")
        ax1.set_ylabel("Y [m]")

        if errors:
            ax2 = fig1.add_subplot(1, 2, 2)
            ax2.plot(errors)
            ax2.set_title("Error de seguimiento (punto a punto)")
            ax2.set_xlabel("Índice de muestra")
            ax2.set_ylabel("Error [m]")
            ax2.grid(True)

        fig_path1 = out_dir / f"trayectoria_{timestamp}_{tag}.png"
        fig1.savefig(fig_path1, dpi=300)
        plt.close(fig1)
        print(f"Gráfica de trayectoria guardada en: {fig_path1}")

        # -------- FIGURA 2: Estilo segundo script (velocidad y yaw vs tiempo) --------
        if self.log_t:
            fig2, axes = plt.subplots(2, 2, figsize=(12, 10))

            # 1) Trayectorias
            axes[0, 0].plot(exp_x, exp_y, '-r', label='Referencia', linewidth=2)
            axes[0, 0].plot(gen_x, gen_y, '-b', label='Real', linewidth=1.5)
            axes[0, 0].set_xlabel('X [m]')
            axes[0, 0].set_ylabel('Y [m]')
            axes[0, 0].set_title('Comparación de trayectorias')
            axes[0, 0].legend()
            axes[0, 0].grid(True)
            axes[0, 0].axis('equal')

            # 2) Error vs tiempo
            if errors:
                axes[0, 1].plot(self.log_t[:len(errors)], errors, '-g', linewidth=1.5)
                axes[0, 1].axhline(y=avg_error, linestyle='--',
                                   label=f'Promedio: {avg_error:.3f} m')
                axes[0, 1].set_xlabel('Tiempo [s]')
                axes[0, 1].set_ylabel('Error [m]')
                axes[0, 1].set_title('Error de seguimiento vs tiempo')
                axes[0, 1].legend()
                axes[0, 1].grid(True)

            # 3) Velocidad (comandada)
            axes[1, 0].plot(self.log_t, [v * 3.6 for v in self.log_v], '-m', linewidth=1.5)
            axes[1, 0].set_xlabel('Tiempo [s]')
            axes[1, 0].set_ylabel('Velocidad [km/h]')
            axes[1, 0].set_title('Perfil de velocidad (comando)')
            axes[1, 0].grid(True)

            # 4) Orientación (yaw)
            axes[1, 1].plot(self.log_t, [math.degrees(th) for th in self.log_yaw],
                            '-c', linewidth=1.5)
            axes[1, 1].set_xlabel('Tiempo [s]')
            axes[1, 1].set_ylabel('Orientación [°]')
            axes[1, 1].set_title('Orientación del vehículo')
            axes[1, 1].grid(True)

            plt.tight_layout()
            fig_path2 = out_dir / f"analisis_trayectoria_{timestamp}_{tag}.png"
            fig2.savefig(fig_path2, dpi=300, bbox_inches='tight')
            plt.close(fig2)
            print(f"Gráficas de análisis guardadas en: {fig_path2}")

        # Guardar CSV detallado
        self.save_trajectory_csv(errors, out_dir, timestamp, tag)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("Ctrl+C recibido, deteniendo QCar y cerrando nodo.")
        pass
    finally:
        # try:
        #     node.stop_qcar()
        # except Exception as e:
        #     node.get_logger().warn(f"No se pudo enviar STOP en shutdown: {e}")

        node.plot_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

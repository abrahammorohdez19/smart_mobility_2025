# Pure Pursuit con modelo Ackermann para AMR
# Comunicación CAN: Envío de comandos por serial
# Autor: Git: Marmanja

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sm_interfaces.msg import Amr

import math
import csv
from pathlib import Path
from datetime import datetime
import serial
import time

import numpy as np
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# ==========================
# IDs CAN DEL PROTOCOLO
# ==========================
class CANID:
    """
    IDs de comandos enviados por CAN.
    El microcontrolador interpreta cada ID como:
    - 100: dirección      (0–100%)
    - 200: freno          (0–100%)
    - 300: motor PWM      (0–100%)
    - 320: dirección motor (0 = reversa, 1 = adelante)
    """
    CMD_STEERING = 100
    CMD_BRAKE = 200
    CMD_MOTOR_PWM = 300
    CMD_MOTOR_DIR = 320
    
# ==========================
# CLASES AUXILIARES
# ==========================
class PPState:
    """Estado del vehículo usado en Pure Pursuit."""
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.xr = x      # posición X real
        self.yr = y      # posición Y real
        self.theta = yaw # orientación yaw
        self.v = v       # velocidad (normalizada 0–1)

    def calc_distance(self, px, py):
        """Distancia Euclidiana a un punto."""
        return math.hypot(self.xr - px, self.yr - py)

class TargetCourse:
    """Maneja la trayectoria y búsqueda del punto objetivo (lookahead)."""
    def __init__(self, path_points, k_gain, base_lookahead):
        self.cx = [p[0] for p in path_points]     # lista X de trayectoria
        self.cy = [p[1] for p in path_points]     # lista Y de trayectoria
        self.k = k_gain                           # ganancia dinámica del lookahead
        self.Lfc = base_lookahead                 # lookahead base
        self.old_nearest_point_index = None       # último punto más cercano

    def search_target_index(self, state: PPState):
        """ Busca el punto objetivo según lookahead dinámico."""
        if self.old_nearest_point_index is None:
            dists = [
                math.hypot(state.xr - cx_i, state.yr - cy_i)
                for cx_i, cy_i in zip(self.cx, self.cy)
            ]
            ind = int(np.argmin(dists))
            self.old_nearest_point_index = ind
        
        else:
            # Continuar avanzando en la trayectoria
            ind = self.old_nearest_point_index
            distance_this = state.calc_distance(self.cx[ind], self.cy[ind])

            # Mientras haya puntos más cercanos hacia adelante
            while ind + 1 < len(self.cx):
                distance_next = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if distance_this < distance_next:
                    break
                ind += 1
                distance_this = distance_next

            self.old_nearest_point_index = ind

        # Lookahead dinámico dependiente de la velocidad
        v = max(state.v, 0.0)
        Lf = self.k * v + self.Lfc

        # Buscar el punto objetivo final
        target_ind = ind
        while target_ind < len(self.cx) - 1:
            if state.calc_distance(self.cx[target_ind], self.cy[target_ind]) >= Lf:
                break
            target_ind += 1

        return target_ind, Lf

# ==========================
# NODO PURE PURSUIT CON CAN
# ==========================
class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ----------------- PARÁMETROS -----------------
        # Parámetros configurables
        self.declare_parameter('path_csv', '')
        self.declare_parameter('lookahead', 3.0)
        self.declare_parameter('k_gain', 1.0) 
        self.declare_parameter('v_ref', 70.0)  # velocidad en %PWM
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('steer_min_deg', -15.0)
        self.declare_parameter('steer_max_deg', 15.0)
        self.declare_parameter('wheelbase', 1.5)
        
        # Parámetros para trayectoria circular
        self.declare_parameter('circle_radius', 3)
        self.declare_parameter('circle_points', 800)

        # Parámetros para trayectoria senoidal
        self.declare_parameter('sine_length', 6.0)
        self.declare_parameter('sine_points', 800)
        self.declare_parameter('sine_amplitude', 6.0)
        self.declare_parameter('sine_frequency', 2.0)

        # Leer parámetros
        path_csv = self.get_parameter('path_csv').value
        self.lookahead = self.get_parameter('lookahead').value
        self.v_ref = self.get_parameter('v_ref').value
        self.k_gain = self.get_parameter('k_gain').value
        self.L = self.get_parameter('wheelbase').value
        
        serial_port = self.get_parameter('serial_port').value
        serial_baudrate = int(self.get_parameter('serial_baudrate').value)
        self.steer_min_deg = self.get_parameter('steer_min_deg').value
        self.steer_max_deg = self.get_parameter('steer_max_deg').value

        # ----------------- INICIALIZAR SERIAL -----------------
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=serial_baudrate,
                timeout=0.1
            )
            time.sleep(2)  # tiempo para que arduino/Microcontrolador inicialice
            self.get_logger().info(f'✓ Conexión CAN establecida: {serial_port} @ {serial_baudrate}')
        except Exception as e:
            self.get_logger().error(f'✗ Error al abrir puerto serial: {e}')
            self.serial_conn = None

        # ----------------- CARGA DEL PATH -----------------
        if path_csv.strip():
            self.path = self.load_path(path_csv)      # cargar desde CSV
        else:
            #self.path = self.generate_circle_path()  # trayectoria circular generada en código
            self.path = self.generate_sine_path()     # trayectoria senoidal generada en código

        # Crear curso objetivo
        if self.path:
            self.target_course = TargetCourse(self.path, self.k_gain, self.lookahead)
        else:
            self.target_course = None

        # ----------------- ESTADOS Y LOGS -----------------
        self.current_pose = None
        self.state = PPState()
        self.target_ind = 0
        self.generated_trajectory = []
        self.last_objective = None

        # Logs de datos
        self.log_t = []
        self.log_x = []
        self.log_y = []
        self.log_yaw = []
        self.log_v = []
        self.log_steer_deg = []
        self.log_steer_cmd = []

        self.start_time = self.get_clock().now()
        self.last_pose_time = self.get_clock().now()

        # ----------------- SUBSCRIPTOR DE POSE -----------------
        qos_pose = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pose_sub = self.create_subscription(
            Vector3Stamped,
            '/amr/pose',
            self.pose_callback,
            qos_pose
        )

        # ----------------- TIMER DE CONTROL (50Hz) -----------------
        self.timer = self.create_timer(0.02, self.control_loop)

        # Info de inicio
        self.get_logger().info("=" * 60)
        self.get_logger().info("  PURE PURSUIT AMR CON PROTOCOLO CAN INICIADO")
        self.get_logger().info("=" * 60)

    # ==========================
    # CARGA Y GENERACIÓN DE PATH
    # ==========================
    def load_path(self, csv_file):
        """Carga un CSV con las columnas x,y."""
        path_points = []
        p = Path(csv_file)

        if not p.exists():
            self.get_logger().error(f'CSV no encontrado: {csv_file}')
            return path_points

        with p.open('r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    path_points.append((float(row['x']), float(row['y'])))
                except:
                    continue

        return path_points

    def generate_circle_path(self):
        """Genera una trayectoria circular."""
        R = self.get_parameter('circle_radius').value
        N = int(self.get_parameter('circle_points').value)

        path_points = []
        for i in range(N):
            theta = 2.0 * math.pi * i / N
            x = R * math.cos(theta)
            y = R * math.sin(theta) + R
            path_points.append((x, y))

        return path_points
    
    def generate_sine_path(self):
        """Genera una trayectoria senoidal."""
        length = self.get_parameter('sine_length').value
        N = int(self.get_parameter('sine_points').value)
        A = self.get_parameter('sine_amplitude').value
        freq = self.get_parameter('sine_frequency').value

        path_points = []
        for i in range(N):
            t = (i / (N - 1)) * length
            x = t
            y = A * math.sin(2 * math.pi * freq * t / length)
            path_points.append((x, y))

        return path_points

    # ==========================
    # CALLBACK DE POSE
    # ==========================
    def pose_callback(self, msg: Vector3Stamped):
        """ Actualiza la pose actual del AMR."""
        self.current_pose = msg
        self.last_pose_time = self.get_clock().now()
        # Información de verificación de la pose recibida
        self.get_logger().info(f"Pose recibida: x={msg.vector.x:}, y={msg.vector.y:}, yaw={msg.vector.z:}")

    # ==========================
    # ENVÍO DE COMANDOS CAN
    # ==========================
    def map_steer_to_can(self, steer_deg):
        """Convierte ángulo de dirección en grados a comando CAN (0–100)."""
        steer_deg = max(self.steer_min_deg, min(self.steer_max_deg, steer_deg))
        can_value = int(
            ((steer_deg - self.steer_min_deg) /
            (self.steer_max_deg - self.steer_max_deg)) * 100
        )
        return max(0, min(100, can_value))

    def send_can_command(self, can_id, value=None):
        """Envía un comando CAN mediante serial.
        Formato:   "ID valor\n" (El espacio entre el ID y el valor es muy importante)
        """
        if self.serial_conn is None or not self.serial_conn.is_open:
            return False

        try:
            if value is None:
                command_str = str(can_id)
            else:
                command_str = f"{can_id} {value}"

            command_str += "\n"
            self.serial_conn.write(command_str.encode())
            return True

        except Exception as e:
            self.get_logger().error(f'Error enviando CAN: {e}')
            return False

    # ==========================
    # BUCLE DE CONTROL
    # ==========================
    def control_loop(self):
        """Lazo de control ejecutado a 50Hz.
        Calcula el ángulo de dirección, lo envía por CAN y registra la trayectoria.
        """
        now = self.get_clock().now()

        if self.current_pose is None or not self.path or self.target_course is None:
            return

        # Extraer pose
        x_r = self.current_pose.vector.x
        y_r = self.current_pose.vector.y
        yaw = self.current_pose.vector.z

        # Actualizar estado interno para PP
        self.state.xr = x_r
        self.state.yr = y_r
        self.state.theta = yaw
        self.state.v = abs(self.v_ref / 100.0)

        # Calcular ángulo Pure Pursuit
        delta_rad = self.compute_pure_pursuit_delta()
        delta_deg = math.degrees(delta_rad)

        # Mapear a comando CAN
        steer_can = self.map_steer_to_can(delta_deg)

        # Enviar comandos CAN
        self.send_can_command(f"{CANID.CMD_STEERING} {steer_can}")
        # La velocidad necesita doble comando para activarse 
        self.send_can_command(f"{CANID.CMD_MOTOR_PWM} {int(self.v_ref)}")
        self.send_can_command(f"{CANID.CMD_MOTOR_PWM} {int(self.v_ref)}")
        self.send_can_command(f"{CANID.CMD_MOTOR_DIR} {1}")   # Siempre adelante

        # Guardar trayectoria generada
        self.generated_trajectory.append((x_r, y_r))

        # Logs para análisis
        t = (now.nanoseconds - self.start_time.nanoseconds) * 1e-9
        self.log_t.append(t)
        self.log_x.append(x_r)
        self.log_y.append(y_r)
        self.log_yaw.append(yaw)
        self.log_v.append(self.v_ref)
        self.log_steer_deg.append(delta_deg)
        self.log_steer_cmd.append(steer_can)

    # ==========================
    # CÁLCULO PURE PURSUIT
    # ==========================
    def compute_pure_pursuit_delta(self):
        """Calcula el ángulo de dirección (delta) basado en el punto objetivo."""
        ind, Lf = self.target_course.search_target_index(self.state)

        if self.target_ind >= ind:
            ind = self.target_ind
        self.target_ind = ind

        tx = self.target_course.cx[ind]
        ty = self.target_course.cy[ind]

        # Log de cambio de objetivo (información sobre el punto a seguir)
        if self.last_objective is None or ind != self.last_objective:
            self.last_objective = ind
            self.get_logger().info(
                f"Objetivo={ind} | pos=({tx:.3f}, {ty:.3f}) | Lf={Lf:.3f} m"
            )

        alpha = math.atan2(ty - self.state.yr, tx - self.state.xr) - self.state.theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # normalizar

        delta = math.atan2(2.0 * self.L * math.sin(alpha), Lf)
        return delta

    # ==========================
    # STOP
    # ==========================
    def stop_amr(self):
        """Detiene el AMR colocando PWM=0 y dirección al centro."""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.send_can_command(f"{CANID.CMD_STEERING} 50")
                # La velocidad necesita doble comando para activarse 
                self.send_can_command(f"{CANID.CMD_MOTOR_PWM} 0")
                self.send_can_command(f"{CANID.CMD_MOTOR_PWM} 0")
                self.get_logger().info('STOP enviado')
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'Error en STOP: {e}')

    # ==========================
    # MÉTRICAS Y ANÁLISIS DE TRAYECTORIA
    # ==========================
    def find_closest_point(self, point, traj):
        """Busca el punto más cercano a 'point' dentro de la trayectoria 'traj'."""
        min_dist = float('inf')
        closest = None
        for ref in traj:
            d = math.hypot(point[0] - ref[0], point[1] - ref[1])
            if d < min_dist:
                min_dist = d
                closest = ref
        return min_dist, closest

    def calculate_tracking_error(self):
        """Calcula errores entre trayectoria generada y referencia."""
        if not self.generated_trajectory or not self.path:
            return 0.0, 0.0, []

        errors = []
        for gen_point in self.generated_trajectory:
            min_dist, _ = self.find_closest_point(gen_point, self.path)
            errors.append(min_dist)

        return float(np.mean(errors)), float(np.max(errors)), errors

    def calculate_similarity_percentage(self, errors, tolerance=0.5):
        """% de puntos que siguen la trayectoria dentro de cierta tolerancia."""
        if not errors:
            return 0.0
        return sum(1 for e in errors if e <= tolerance) / len(errors) * 100.0

    def report_performance(self, avg_error, max_error, errors):
        """Imprime evaluación cualitativa del seguimiento."""
        sim05 = self.calculate_similarity_percentage(errors, 0.5)
        sim10 = self.calculate_similarity_percentage(errors, 1.0)
        std = float(np.std(errors))

        print("\n" + "=" * 50)
        print("  EVALUACIÓN DE TRAYECTORIA")
        print("=" * 50)
        print(f"Error promedio:      {avg_error:.3f} m")
        print(f"Error máximo:        {max_error:.3f} m")
        print(f"Desv. estándar:      {std:.3f} m")
        print(f"% dentro de 50cm:     {sim05:.1f}%")
        print(f"% dentro de 100cm:    {sim10:.1f}%")
        print("=" * 50 + "\n")


    def save_trajectory_csv(self, errors, out_dir, timestamp, tag):
        csv_path = out_dir / f"trayectoria_{tag}_{timestamp}.csv"

        with csv_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 't', 'x_real', 'y_real', 'yaw', 'v_cmd',
                           'steer_deg', 'steer_can', 'x_ref', 'y_ref', 'error'])

            for i, (xr, yr) in enumerate(self.generated_trajectory):
                t = self.log_t[i] if i < len(self.log_t) else ''
                yaw = self.log_yaw[i] if i < len(self.log_yaw) else ''
                v_cmd = self.log_v[i] if i < len(self.log_v) else ''
                steer_deg = self.log_steer_deg[i] if i < len(self.log_steer_deg) else ''
                steer_can = self.log_steer_cmd[i] if i < len(self.log_steer_cmd) else ''
                err = errors[i] if i < len(errors) else ''

                _, closest = self.find_closest_point((xr, yr), self.path)
                xc, yc = closest if closest else ('', '')
                writer.writerow([i, t, xr, yr, yaw, v_cmd, steer_deg, steer_can, xc, yc, err])

        print(f" CSV guardado: {csv_path}")

    def plot_results(self):
        if not self.path or not self.generated_trajectory:
            print("No hay datos para graficar")
            return

        exp_x = [p[0] for p in self.path]
        exp_y = [p[1] for p in self.path]
        gen_x = [p[0] for p in self.generated_trajectory]
        gen_y = [p[1] for p in self.generated_trajectory]

        avg_error, max_error, errors = self.calculate_tracking_error()
        if errors:
            self.report_performance(avg_error, max_error, errors)

        out_dir = Path.home() / 'Workspaces' / 'smart_mobility_qcar_ros2' / 'resultados' / 'pure_pursuit_amr' # Lugar donde guarda las gráficas y csv
        out_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = f"L{self.lookahead}_V{int(self.v_ref)}_k{self.k_gain}"

        if self.log_t:
            fig, axes = plt.subplots(2, 3, figsize=(18, 10))

            # Trayectorias
            axes[0, 0].plot(exp_x, exp_y, '-r', label='Referencia', linewidth=2)
            axes[0, 0].plot(gen_x, gen_y, '-b', label='Real', linewidth=1.5)
            if len(gen_x) > 1:
                axes[0, 0].scatter(gen_x[0], gen_y[0], s=120, c='green', marker='o', label='Inicio')
                axes[0, 0].scatter(gen_x[-1], gen_y[-1], s=120, c='red', marker='x', label='Final')
            axes[0, 0].set_xlabel('X [m]')
            axes[0, 0].set_ylabel('Y [m]')
            axes[0, 0].set_title('Trayectorias')
            axes[0, 0].legend()
            axes[0, 0].grid(True)
            axes[0, 0].axis('equal')

            # Error
            if errors:
                axes[0, 1].plot(self.log_t[:len(errors)], errors, '-g')
                axes[0, 1].axhline(y=avg_error, linestyle='--', label=f'Promedio: {avg_error:.3f}m')
                axes[0, 1].set_xlabel('Tiempo [s]')
                axes[0, 1].set_ylabel('Error [m]')
                axes[0, 1].set_title('Error de seguimiento')
                axes[0, 1].legend()
                axes[0, 1].grid(True)

            # Steering grados
            if self.log_steer_deg:
                axes[0, 2].plot(self.log_t[:len(self.log_steer_deg)], self.log_steer_deg, '-r')
                axes[0, 2].axhline(y=0, linestyle='--', color='gray', alpha=0.5)
                axes[0, 2].set_xlabel('Tiempo [s]')
                axes[0, 2].set_ylabel('Steering [°]')
                axes[0, 2].set_title('Ángulo de dirección')
                axes[0, 2].grid(True)

            # Velocidad
            axes[1, 0].plot(self.log_t, self.log_v, '-m')
            axes[1, 0].set_xlabel('Tiempo [s]')
            axes[1, 0].set_ylabel('PWM [%]')
            axes[1, 0].set_title('Comando de velocidad')
            axes[1, 0].grid(True)

            # Orientación
            axes[1, 1].plot(self.log_t, [math.degrees(th) for th in self.log_yaw], '-c')
            axes[1, 1].set_xlabel('Tiempo [s]')
            axes[1, 1].set_ylabel('Yaw [°]')
            axes[1, 1].set_title('Orientación')
            axes[1, 1].grid(True)

            # Comando CAN
            if self.log_steer_cmd:
                axes[1, 2].plot(self.log_t[:len(self.log_steer_cmd)], self.log_steer_cmd, '-b')
                axes[1, 2].axhline(y=50, linestyle='--', color='gray', alpha=0.5, label='Centro')
                axes[1, 2].set_xlabel('Tiempo [s]')
                axes[1, 2].set_ylabel('Comando CAN [0-100]')
                axes[1, 2].set_title('Comando dirección')
                axes[1, 2].legend()
                axes[1, 2].grid(True)
                axes[1, 2].set_ylim([-5, 105])

            plt.tight_layout()
            fig_path = out_dir / f"analisis_{timestamp}_{tag}.png"
            fig.savefig(fig_path, dpi=300, bbox_inches='tight')
            plt.close(fig)
            print(f"Gráficas guardadas: {fig_path}")

        self.save_trajectory_csv(errors, out_dir, timestamp, tag)
      
    def _del_(self):
        """Cierra el puerto serial al destruir el nodo."""
        if hasattr(self, 'serial_conn') and self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
                print('Conexión serial cerrada')
            except:
                pass


# ==========================
# MAIN
# ==========================
def main(args=None):
    #marml 2025 
    rclpy.init(args=args)
    node = PurePursuitNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_amr()         # detener robot
        node.plot_results()     # graficar análisis de seguimiento
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

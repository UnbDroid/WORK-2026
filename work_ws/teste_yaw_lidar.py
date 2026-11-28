#!/usr/bin/env python3
"""Teste do yaw pelo lidar, fora da prova, em qualquer mesa ou parede reta.

Uso (com o launch do robô rodando: robot_state_publisher + lidar + filtro):
  python3 teste_yaw_lidar.py                    # só mostra as leituras (robô não se mexe)
  python3 teste_yaw_lidar.py --corrigir         # PUBLICA angular.z para testar o sentido da correção
  python3 teste_yaw_lidar.py --corrigir --sinal -1
  python3 teste_yaw_lidar.py --sem-tf --offset 180 --invertido   # só se o TF não existir

Por padrão usa o TF base_link <- frame do lidar (o mesmo do URDF/rf2o).
A conta do yaw é a mesma do vision_node.
"""
import argparse
import collections
import numpy as np
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# ---------- mesmas funções do vision_node ----------
def quat_para_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def resumo_tf(R):
    inv = R[2, 2] < 0
    psi = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return (-psi if inv else psi), bool(inv)


def para_robo(r, cru, tf_lidar, offset, invertido):
    if tf_lidar is not None:
        R, t = tf_lidar
        q = R @ np.stack([r * np.cos(cru), r * np.sin(cru), np.zeros_like(r)]) + t[:, None]
        return q[0], q[1]
    ang = cru + offset
    if invertido:
        ang = -ang
    return r * np.cos(ang), r * np.sin(ang)


def reta_ransac(x, y, tol=0.015, iteracoes=150):
    n = len(x)
    if n < 2:
        return None
    rng = np.random.default_rng(0)
    i = rng.integers(0, n, iteracoes)
    j = rng.integers(0, n, iteracoes)
    ok = np.abs(y[i] - y[j]) > 0.02
    i, j = i[ok], j[ok]
    if len(i) == 0:
        return None
    a = (x[i] - x[j]) / (y[i] - y[j])
    b = x[i] - a * y[i]
    dist = np.abs(x[None, :] - (a[:, None] * y[None, :] + b[:, None])) / np.sqrt(1 + a[:, None] ** 2)
    inl = dist < tol
    return inl[int(np.argmax(inl.sum(axis=1)))]
# ---------------------------------------------------


class TesteYaw(Node):
    def __init__(self, a):
        super().__init__("teste_yaw_lidar")
        self.a = a
        self.offset = np.radians(a.offset)
        self.setor = np.radians(a.setor)
        self.hist = collections.deque(maxlen=20)
        self.base_scans = []
        self.base = None
        self.tf_lidar = None
        self.estado = None
        if not a.sem_tf:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, a.topic, self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10) if a.corrigir else None
        self.create_timer(0.5, self.imprimir)
        if a.corrigir:
            self.create_timer(0.1, self.corrigir)
            self.get_logger().warn("MODO --corrigir: o robô VAI GIRAR. Ctrl+C para parar.")
        self.get_logger().info(f"Esperando {a.topic} ... (não mexa na frente do robô nos primeiros 2 s)")

    def cb(self, msg):
        if not self.a.sem_tf and self.tf_lidar is None:
            try:
                tr = self.tf_buffer.lookup_transform(self.a.base_frame, msg.header.frame_id, rclpy.time.Time())
                t = tr.transform.translation
                self.tf_lidar = (quat_para_R(tr.transform.rotation), np.array([t.x, t.y, t.z]))
                off, inv = resumo_tf(self.tf_lidar[0])
                self.get_logger().info(
                    f"TF {self.a.base_frame} <- {msg.header.frame_id} OK: equivale a offset={off:.1f}° invertido={inv}, "
                    f"lidar em x={t.x:.3f} y={t.y:.3f} z={t.z:.3f} m")
            except Exception as e:
                self.get_logger().warn(f"Ainda sem TF ({e.__class__.__name__}). Está rodando o robot_state_publisher?",
                                       throttle_duration_sec=3.0)
                return

        r = np.asarray(msg.ranges, dtype=float)
        cru = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        v = np.isfinite(r) & (r > max(msg.range_min, self.a.dist_min)) & (r < msg.range_max)
        # cena de referência (sem mão) para achar onde algo mudou
        if self.base is None:
            self.base_scans.append(np.where(v, r, np.nan))
            if len(self.base_scans) >= 15:
                self.base = np.nanmedian(np.array(self.base_scans), axis=0)
                self.get_logger().info("Referência gravada. Agora pode colocar a mão.")
        x, y = para_robo(r, cru, self.tf_lidar, self.offset, self.a.invertido)

        # yaw: mesmos filtros do vision_node
        perto = v & (r < self.a.dist_max)
        m = perto & (x > 0) & (np.abs(np.arctan2(y, x)) < self.setor)
        o = dict(n=int(m.sum()), n_bons=0, yaw=None, dist=None, residuo_mm=None)
        if m.sum() >= self.a.min_pontos:
            bom = reta_ransac(x[m], y[m])
            if bom is not None:
                o["n_bons"] = int(bom.sum())
                if bom.sum() >= self.a.min_pontos:
                    xs, ys = x[m][bom], y[m][bom]
                    aa, bb = np.polyfit(ys, xs, 1)
                    o.update(yaw=float(np.arctan(aa)), dist=float(bb),
                             residuo_mm=float(np.std(xs - (aa * ys + bb)) * 1000))
        if o["yaw"] is not None:
            self.hist.append(np.degrees(o["yaw"]))
        self.estado = (r, cru, v, x, y, o)

    def wz(self):
        if self.estado is None or self.estado[5]["yaw"] is None:
            return 0.0
        yaw = self.estado[5]["yaw"]
        if abs(yaw) < np.radians(0.5) or abs(yaw) > np.radians(25):
            return 0.0
        return float(np.clip(-self.a.sinal * self.a.k * yaw, -self.a.max_wz, self.a.max_wz))

    def corrigir(self):
        t = Twist()
        t.angular.z = self.wz()
        self.pub.publish(t)

    def imprimir(self):
        if self.estado is None:
            return
        r, cru, v, x, y, o = self.estado
        ang = np.degrees(np.arctan2(y, x))

        def direcao(alvo):
            d = np.abs((ang - alvo + 180) % 360 - 180)
            s = v & (d < 5)
            return np.min(np.hypot(x[s], y[s])) if s.any() else float("nan")

        diag = (f"frente={direcao(0):.2f}m  esq={direcao(90):.2f}m  dir={direcao(-90):.2f}m  "
                f"trás={direcao(180):.2f}m")
        if self.base is not None:
            queda = self.base - np.where(v, r, np.nan)      # >0 onde algo ficou mais perto
            if np.isfinite(queda).any() and np.nanmax(queda) > 0.08:
                i = int(np.nanargmax(queda))
                diag += (f" | MUDOU: ângulo cru {np.degrees(cru[i]):.0f}° -> no robô {ang[i]:+.0f}° "
                         f"({self.base[i]:.2f} -> {r[i]:.2f} m)")
            else:
                diag += " | nada mudou na cena"
        if o["yaw"] is None:
            yaw_txt = f"SEM BORDA (pontos no setor={o['n']}, na reta={o['n_bons']}, mínimo={self.a.min_pontos})"
        else:
            ruido = np.std(self.hist) if len(self.hist) > 5 else float("nan")
            yaw_txt = (f"yaw={np.degrees(o['yaw']):+6.2f}°  ruído=±{ruido:.2f}°  dist_borda={o['dist']:.3f}m  "
                       f"pontos={o['n_bons']}/{o['n']}  resíduo={o['residuo_mm']:.1f}mm")
        if self.pub is not None:
            yaw_txt += f"  -> wz={self.wz():+.2f} rad/s"
        self.get_logger().info(f"{yaw_txt}\n    {diag}")

    def parar(self):
        if self.pub is not None:
            for _ in range(5):
                self.pub.publish(Twist())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/scan_filtered")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--sem-tf", action="store_true", help="não usa TF; usa --offset/--invertido")
    p.add_argument("--offset", type=float, default=0.0, help="graus (só com --sem-tf)")
    p.add_argument("--invertido", action="store_true", help="lidar de cabeça para baixo (só com --sem-tf)")
    p.add_argument("--setor", type=float, default=20.0, help="meia abertura do setor da frente (graus)")
    p.add_argument("--dist-min", type=float, default=0.10, help="m, ignora o próprio robô")
    p.add_argument("--dist-max", type=float, default=1.5)
    p.add_argument("--min-pontos", type=int, default=15)
    p.add_argument("--corrigir", action="store_true", help="publica angular.z em /cmd_vel")
    p.add_argument("--sinal", type=float, default=1.0)
    p.add_argument("--k", type=float, default=0.8)
    p.add_argument("--max-wz", type=float, default=0.15, help="mais baixo que no vision_node, por segurança")
    a, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    n = TesteYaw(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.parar()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
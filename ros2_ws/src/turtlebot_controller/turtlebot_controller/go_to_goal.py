"""TurtleBot3'ü Gazebo'da belirli bir (x, y) noktasına götüren ROS2 node'u.

Akış (Publisher -> Topic -> Message -> Subscriber):
  * /odom  (nav_msgs/Odometry)   : robotun konumunu OKUR   (subscriber)
  * /cmd_vel (geometry_msgs/Twist): robota hız komutu YAYINLAR (publisher)

Koordinatlar 'odom' çerçevesindedir: robot Gazebo'da doğduğu anda odom
başlangıcı (0, 0) ve robotun baktığı yön x ekseni (yaw = 0) olur.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle):
    """Açıyı [-pi, pi] aralığına getirir."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    """Quaternion'dan yaw (z ekseni etrafındaki dönüş) açısını hesaplar."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # --- Parametreler (komut satırından değiştirilebilir) ---
        self.declare_parameter('goal_x', 1.5)           # metre (odom çerçevesi)
        self.declare_parameter('goal_y', 1.0)           # metre (odom çerçevesi)
        self.declare_parameter('goal_tolerance', 0.05)  # hedefe varış mesafesi (m)
        self.declare_parameter('max_linear', 0.15)      # m/s
        self.declare_parameter('max_angular', 0.8)      # rad/s
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        # --- Publisher: hız komutu ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # --- Subscriber: konum bilgisi ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Robotun anlık durumu
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got_odom = False
        self.start_checked = False

        # Durum bayrakları
        self.reached = False
        self.finished = False      # main() döngüsü bunu görünce node'u kapatır
        self.stop_count = 0

        # 10 Hz kontrol döngüsü
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'Hedef: ({self.goal_x:.2f}, {self.goal_y:.2f}) m, '
            f'tolerans: {self.tolerance:.2f} m. /odom bekleniyor...')

    # ----------------------------------------------------------------         
    def odom_callback(self, msg):
        """/odom her geldiğinde robotun x, y ve yaw değerini günceller."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.got_odom = True

    # ------------------------------------------------------------------
    def check_goal_not_in_line(self):
        """Ödev şartı: hedef, başlangıç noktasıyla aynı doğrultuda olmamalı.

        Robot başlangıçta yaw yönüne bakar. Hedefe giden doğru bu yönle
        (ileri ya da geri) ~10 dereceden az farklıysa hedef aynı doğrultudadır.
        """
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        diff = abs(normalize_angle(math.atan2(dy, dx) - self.yaw))
        limit = math.radians(10.0)
        if diff < limit or diff > math.pi - limit:
            return False
        self.get_logger().info(
            f'Başlangıç: ({self.x:.2f}, {self.y:.2f}), hedefe açı farkı: '
            f'{math.degrees(diff):.1f} derece -> hedef aynı doğrultuda değil.')
        return True

    # ------------------------------------------------------------------
    def stop_robot(self):
        """Tüm hızları sıfır olan Twist yayınlar."""
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.got_odom:
            return  # henüz konum yok, hareket etme

        if not self.start_checked:
            self.start_checked = True
            if not self.check_goal_not_in_line():
                self.get_logger().error(
                    'Hedef başlangıçla aynı doğrultuda! goal_x/goal_y değiştirin.')
                self.stop_robot()
                self.finished = True
                return

        # Hedef sonrası: sıfır hızı birkaç kez gönder (komut kaybına karşı)
        if self.reached:
            self.stop_robot()
            self.stop_count += 1
            if self.stop_count >= 10:
                self.get_logger().info(
                    f'Robot durdu. Son konum: ({self.x:.2f}, {self.y:.2f})')
                self.finished = True
            return

        # 1) Hedefe uzaklık ve hedef yönü
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.hypot(dx, dy)

        # 2) Hedefe ulaşıldı mı?
        if distance < self.tolerance:
            self.reached = True
            self.stop_robot()
            self.get_logger().info(
                f'Hedefe ulaşıldı! Kalan mesafe: {distance:.3f} m. Hız sıfırlanıyor.')
            return

        # 3) Açı hatası: robotun baktığı yön ile hedef yönü arasındaki fark
        angle_error = normalize_angle(math.atan2(dy, dx) - self.yaw)

        cmd = Twist()
        if abs(angle_error) > 0.3:
            # Hedeften çok farklı yöne bakıyorsa önce yerinde dön
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_angular,
                                min(self.max_angular, 1.5 * angle_error))
        else:
            # Hedefe bakıyorsa ilerle, giderken yönü düzelt
            speed = 0.5 * distance
            cmd.linear.x = max(0.03, min(self.max_linear, speed))
            cmd.angular.z = max(-self.max_angular,
                                min(self.max_angular, 1.5 * angle_error))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Ne olursa olsun çıkarken robotu durdur
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

    main()

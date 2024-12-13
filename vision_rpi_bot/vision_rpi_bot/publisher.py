#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')

        # Publisher tanımı
        self.publisher_ = self.create_publisher(PolygonStamped, '/user_defined_polygons', 1)

        # Kullanıcıdan giriş al ve poligonları hazırla
        self.polygons = self.get_user_defined_polygons()

        # Zamanlayıcı tanımı (1 Hz'de poligon yayınlayacak)
        self.timer = self.create_timer(1.0, self.publish_polygons)

        self.get_logger().info("Polygon publisher is ready!")

    def get_user_defined_polygons(self):
        polygons = []

        for i in range(2):  # İki poligon için döngü
            print(f"\nEnter points for Polygon {i+1}:")
            polygon = PolygonStamped()
            polygon.header.frame_id = 'map'  # Harita çerçevesinde

            for j in range(4):  # Her poligon için 4 nokta al
                while True:
                    try:
                        print(f"Point {j+1}:")
                        x = float(input("Enter x: "))
                        y = float(input("Enter y: "))
                        z = 0.0  # Z değerini sabit tutuyoruz
                        point = Point32(x=x, y=y, z=z)
                        polygon.polygon.points.append(point)
                        break
                    except ValueError:
                        print("Invalid input. Please enter numeric values for x and y.")

            polygons.append(polygon)

        return polygons

    def publish_polygons(self):
        for i, polygon in enumerate(self.polygons):
            # Mesaj başlığını güncelle
            polygon.header.stamp = self.get_clock().now().to_msg()

            # Mesajı yayınla
            self.publisher_.publish(polygon)
            self.get_logger().info(f"Published Polygon {i+1}")

def main(args=None):
    rclpy.init(args=args)
    node = PolygonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


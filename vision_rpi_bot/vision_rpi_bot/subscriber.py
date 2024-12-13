#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')

        # Publisher tanımı
        self.publisher_ = self.create_publisher(PolygonStamped, '/user_defined_polygons', 10)

        # Poligonları tanımla
        self.polygons = self.create_polygons()

        # İlk zaman damgasını belirle
        self.initial_stamp = self.get_clock().now().to_msg()

        # Zamanlayıcı tanımı (1 Hz'de poligon yayınlayacak)
        self.timer = self.create_timer(1.0, self.publish_polygons)

        self.get_logger().info("Polygon publisher is ready!")

    def create_polygons(self):
        polygons = []

        # Poligon 1 (örnek: kare)
        polygon1 = PolygonStamped()
        polygon1.header.frame_id = 'map'
        polygon1.polygon.points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=1.0, y=0.0, z=0.0),
            Point32(x=1.0, y=1.0, z=0.0),
            Point32(x=0.0, y=1.0, z=0.0)
        ]
        polygons.append(polygon1)

        # Poligon 2 (örnek: farklı bir açıda dörtgen)
        polygon2 = PolygonStamped()
        polygon2.header.frame_id = 'map'
        polygon2.polygon.points = [
            Point32(x=2.0, y=2.0, z=0.0),
            Point32(x=3.0, y=2.0, z=0.0),
            Point32(x=3.5, y=3.0, z=0.0),
            Point32(x=2.5, y=3.5, z=0.0)
        ]
        polygons.append(polygon2)

        return polygons

    def publish_polygons(self):
        for i, polygon in enumerate(self.polygons):
            # Zaman damgasını başlangıç zamanına sabitle
            polygon.header.stamp = self.initial_stamp

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


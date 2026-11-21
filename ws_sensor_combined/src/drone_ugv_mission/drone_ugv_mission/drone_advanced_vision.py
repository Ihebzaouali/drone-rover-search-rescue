#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool          # ← ADDED: to signal person_locator_node
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class DetectedObject:
    def __init__(self, name, x, y, w, h, conf, method):
        self.name = name
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.conf = conf
        self.method = method


class DroneAdvancedVision(Node):
    def __init__(self):
        super().__init__('drone_advanced_vision')

        # --- PARAMÈTRES ---
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('camera_topic', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image')
        self.declare_parameter('ai_conf_threshold', 0.25)

        model_name    = self.get_parameter('yolo_model').get_parameter_value().string_value
        camera_topic  = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.ai_conf  = self.get_parameter('ai_conf_threshold').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.get_logger().info("Chargement de l'Intelligence Artificielle...")
        self.model = YOLO(model_name)

        # --- CONFIGURATION OPENCV (Couleurs Gazebo) ---
        self.couleurs_config = {
            'Bloc Bleu':      {'lower': [90,  50,  50], 'upper': [140, 255, 255], 'color': (255, 0,   0),   'min_area': 200},
            'Cylindre Jaune': {'lower': [15,  50,  50], 'upper': [40,  255, 255], 'color': (0,   255, 255), 'min_area': 200},
            'UGV (Orange)':   {'lower': [0,   50,  50], 'upper': [20,  255, 255], 'color': (0,   165, 255), 'min_area': 200},
            'Obstacle Haut':  {'lower': [120, 40,  40], 'upper': [170, 255, 255], 'color': (128, 0,   128), 'min_area': 200},
        }

        # --- ROS 2 SUBSCRIBERS ---
        self.subscription = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )

        # --- ROS 2 PUBLISHERS ---
        # Debug image — unchanged from your original
        self.publisher_ = self.create_publisher(
            Image, '/drone/vision_avancee_debug', 10
        )

        # ── ADDED ────────────────────────────────────────────────────────────
        # Publishes True on this topic whenever YOLO detects a "person" in
        # the current frame.  person_locator_node subscribes to this topic
        # and converts the drone's position to a Nav2 goal for the UGV.
        # Publishes False when no person is visible, so person_locator_node
        # can clear its detection state.
        self.person_pub = self.create_publisher(Bool, '/drone/person_detected', 10)
        # ─────────────────────────────────────────────────────────────────────

        self.frame_count = 0
        self.get_logger().info(
            "✅ Vision Prête ! Ouvrez rqt_image_view sur /drone/vision_avancee_debug"
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur Bridge: {e}")
            return

        toutes_detections = []

        # Prétraitement pour OpenCV
        blurred   = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # ==========================================
        # 1. DÉTECTION OPENCV (Par Couleurs)
        # ==========================================
        for nom, config in self.couleurs_config.items():
            mask = cv2.inRange(
                hsv_image,
                np.array(config['lower']),
                np.array(config['upper'])
            )
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in contours:
                if cv2.contourArea(cnt) > config['min_area']:
                    x, y, w, h = cv2.boundingRect(cnt)
                    toutes_detections.append(
                        DetectedObject(nom, x, y, w, h, cv2.contourArea(cnt), 'OpenCV')
                    )

        # ==========================================
        # 2. DÉTECTION YOLOv8 (Intelligence Artificielle)
        # ==========================================
        # ── ADDED: track whether a person appears in this frame ──────────────
        person_detected_this_frame = False
        # ─────────────────────────────────────────────────────────────────────

        results = self.model(cv_image, verbose=False, conf=self.ai_conf)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                cls  = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls]

                toutes_detections.append(
                    DetectedObject(class_name, x1, y1, x2 - x1, y2 - y1, conf, 'YOLO')
                )

                # ── ADDED ─────────────────────────────────────────────────────
                # "person" is the standard COCO class name for humans.
                # The wheelchair person in your Gazebo world is detected
                # under this class name by YOLOv8.
                if class_name == 'person':
                    person_detected_this_frame = True
                    self.get_logger().info(
                        f'[PERSON DETECTED] confidence={conf:.2f}  '
                        f'bbox=({x1},{y1},{x2},{y2})',
                        throttle_duration_sec=1.0   # log max once per second
                    )
                # ─────────────────────────────────────────────────────────────

        # ── ADDED: publish detection signal every frame ───────────────────────
        # True  → person is visible right now   → person_locator_node fires
        # False → no person visible              → person_locator_node resets
        detection_msg = Bool()
        detection_msg.data = person_detected_this_frame
        self.person_pub.publish(detection_msg)
        # ─────────────────────────────────────────────────────────────────────

        # Terminal log (once every 30 frames)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"Analyse en cours : {len(toutes_detections)} objets détectés."
            )

        # ==========================================
        # 3. DESSIN DES CADRES SUR L'IMAGE
        # ==========================================
        for obj in toutes_detections:
            if obj.method == 'YOLO':
                # ── ADDED: highlight the person box in a distinct colour ──────
                if obj.name == 'person':
                    color      = (0, 0, 255)          # red = person of interest
                    label_text = f"IA: PERSONNE ({int(obj.conf * 100)}%)"
                else:
                    color      = (0, 255, 0)          # green = other YOLO objects
                    label_text = f"IA: {obj.name} ({int(obj.conf * 100)}%)"
                # ─────────────────────────────────────────────────────────────
            else:
                color      = self.couleurs_config.get(obj.name, {}).get('color', (0, 0, 255))
                label_text = f"CVP: {obj.name}"

            cv2.rectangle(
                cv_image,
                (obj.x, obj.y),
                (obj.x + obj.w, obj.y + obj.h),
                color, 2
            )

            font = cv2.FONT_HERSHEY_SIMPLEX
            (text_width, text_height), _ = cv2.getTextSize(label_text, font, 0.5, 2)
            text_y = obj.y - 5 if obj.y - 20 > 0 else obj.y + 15

            cv2.rectangle(
                cv_image,
                (obj.x, text_y - text_height - 5),
                (obj.x + text_width, text_y + 2),
                color, -1
            )
            cv2.putText(
                cv_image, label_text,
                (obj.x, text_y),
                font, 0.5, (255, 255, 255), 2, cv2.LINE_AA
            )

        # ==========================================
        # 4. ENVOI À RQT_IMAGE_VIEW
        # ==========================================
        try:
            msg_debug = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.publisher_.publish(msg_debug)
        except Exception as e:
            self.get_logger().error(f"Erreur Publication: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DroneAdvancedVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

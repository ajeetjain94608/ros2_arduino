# ROS2 + Arduino MPU6050 Integration with FreeRTOS

This repository contains a ROS2 package for integrating an **Arduino Nano** with an **MPU6050 sensor** and **LED control via UART**.

## 📌 Features
✅ FreeRTOS for multitasking on Arduino  
✅ MPU6050 sensor data streaming  
✅ LED control via ROS2  
✅ UART-based communication  
✅ ROS2 topics for sensor (`/mpu_data`) and LED control (`/led_control`)  

## 🚀 How to Run
1. **Flash Arduino Code** (`mpu6050_led_control.ino`)  
2. **Run ROS2 Node**  
   ```bash
   ros2 run arduino_uart serial_node

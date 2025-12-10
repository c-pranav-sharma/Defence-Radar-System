# Defence Radar System Using STM32F401CCUX

![STM32](https://img.shields.io/badge/Microcontroller-STM32F401CCUX-blue)
![Language](https://img.shields.io/badge/Language-Embedded%20C-orange)
![Hardware](https://img.shields.io/badge/Hardware-HC--SR04%20%7C%20OLED%20SSD1306%20%7C%20Servo-green)

## 📜 Abstract
> The **Defence Radar System** is an embedded real-time object detection and tracking project designed to simulate radar-like surveillance. The system employs an ultrasonic sensor mounted on a servo motor to continuously scan a semi-circular area. When an obstacle is detected, the system calculates its distance and visually represents its position on a 0.96-inch OLED display as a radar blip. Simultaneously, the onboard LED toggles to indicate detection. This project demonstrates the integration of sensor data acquisition, PWM servo control, and I²C graphics rendering on an ARM Cortex-M4 microcontroller.

---

## 📖 Introduction
In modern defense applications, radar systems are crucial for detecting and tracking objects. While military radars use radio waves, this project creates a scaled-down embedded prototype using ultrasonic technology to simulate similar functionality.

### 🎯 Objectives
* **Real-Time Scanning:** Scan a semi-circular area using a servo-mounted sensor.
* **Distance Measurement:** Accurately measure object distance using the HC-SR04 sensor.
* **Precision Actuation:** Control angular movement using PWM signals.
* **Visual Interface:** Plot real-time radar "blips" on an OLED screen via I²C.
* **Alert System:** Toggle an LED indicator when an object is detected within range.
* **Integration:** Combine all modules under the control of the STM32F401CCUX.

---

## 🛠️ System Design & Hardware

### Components Required
| Component | Role |
| :--- | :--- |
| **STM32F401CCUX** | Main microcontroller (ARM Cortex-M4). |
| **HC-SR04 Sensor** | Ultrasonic distance measurement (Pulse-Echo). |
| **Servo Motor (SG90)** | Rotates sensor 0°–180° for scanning. |
| **OLED (SSD1306)** | 128x64 display for radar visualization. |
| **LED Indicator** | Visual alert for object detection. |

### 🔌 Hardware Pinout & Schematic
The system connects peripherals to the STM32F401CCUX as follows:

| Peripheral | STM32 Pin | Function |
| :--- | :--- | :--- |
| **Ultrasonic Trig** | `PB0` | Output Trigger Pulse |
| **Ultrasonic Echo** | `PB1` | Input Echo Measurement |
| **Servo Signal** | `PA0` (TIM2) | PWM Output (Channel 1) |
| **OLED SCL** | `PB6` | I²C Clock |
| **OLED SDA** | `PB7` | I²C Data |
| **Status LED** | `PC13` | Active Low Output |

> **Note:** The distance is calculated using the formula:  
> `Distance (cm) = (Echo Time in µs × 0.0343) / 2`

---

## 💻 Software Implementation

### Architecture
The firmware is written in **Embedded C** using bare-metal register programming (no HAL) for maximum performance. It follows a "Super-Loop" architecture with interrupt-driven updates.

### Key Functions
1.  **Ultrasonic Reading:** Sends a 10µs pulse and measures the echo width using a timer.
2.  **Servo Sweep:** Uses a cosine function tied to the system tick to generate smooth PWM values (1ms–2ms pulse width).
3.  **Radar Rendering:** Draws a static background (arcs/lines) and updates dynamic object "blips" using a double-buffer approach to prevent flickering.

### Logic Flow (Pseudo-Code)
```c
while (1) {
    if (g_update_flag) { // Triggered by TIM4 interrupt (~20ms)
        g_update_flag = false;

        // 1. Measure Distance (Every 40ms to avoid echo overlap)
        if (time_elapsed > 40ms) distance = getUltrasonicDistance();

        // 2. Update Alert LED
        if (distance > 0 && distance < RANGE) LED_ON();
        else LED_OFF();

        // 3. Graphics Pipeline
        clear_buffer();
        calculate_servo_angle();
        add_blip_to_map(angle, distance);
        draw_radar_background();
        draw_sweep_line();
        
        // 4. Render to OLED
        ssd1306_UpdateScreen();
    }
}

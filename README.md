# ALC-Guard: IoT-based Alcohol Intake Risk Awareness System

**Coursework for Sensing and Internet of Things** **Imperial College London**

**Author:** Jingyang Liu
**CID:** 02252219

---

## Project Overview

ALC-Guard is a closed-loop IoT smart cup system designed to track alcohol intake and provide real-time risk awareness. By fusing data from weight, tilt, temperature, and gas sensors, the system intelligently distinguishes between **Drinking**, **Sipping**, **Refilling**, and **Pouring Out**.

It provides dual-channel feedback via an **RGB LED on the cup** (Physical) and a **Web Dashboard** (Digital) to help users manage their drinking habits without disruption.

---

## Key Features

* **Smart Event Classification:** Uses an FSM on ESP32 to identify valid drinking vs. noise/refills.
* **Alcohol Validation:** Filters out non-alcoholic drinks using an MQ-3 gas sensor.
* **Context-Aware Night Mode:** Suppresses LED alerts during sleep hours.
* **Dual Feedback:** Real-time LED alerts and web-based risk visualization (Units/hr & Total Volume).

---

## Project Structure

```text
ALC_Guard_Project/
├── app.py              # Flask Backend (Data processing & Local Server)
├── requirements.txt    # Python dependencies list
├── index.html          # Web Dashboard UI (Chart.js visualization)
├── events.csv          # Local data storage (Auto-generated)
├── Final.ino           # ESP32 Firmware (C++ source code)
└── README.md           # Project Documentation

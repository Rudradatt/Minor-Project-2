
# 🤖 Minor Project 2 – AI-Powered Lab Assistant Robot

## 📌 Project Overview
This project presents an **AI-powered Assistant Robot** designed to autonomously guide users inside a laboratory environment.  
The system integrates **Computer Vision, Robotics, IoT, and AI-based Question Answering** to deliver a smart, interactive lab assistant experience.

The project is divided into **two independent modules**, each with its own setup and execution process.

---

## 🧠 Key Features
- 👤 **Real-time Human Tracking** using YOLOv8
- 🎯 **Smooth Navigation** via Kalman Filtering
- ⚙️ **ESP32-based Motor Control** for safe and precise movement
- 💬 **AI-based Question & Answer System**
- 🔊 **Audio Responses** for user interaction
- 🌐 **Flask Web Application**
- 📚 **FAISS Vector Search + LLM** for intelligent query handling

## 📦 Module 1: Lab Assistant Robot

**Folder:** `Lab Assistant code/`

### Description
This module handles the **autonomous movement** of the robot inside the laboratory.  
It uses **YOLOv8** for detecting and tracking humans and applies **Kalman Filtering** to ensure smooth and stable navigation.

### Highlights
- Real-time object detection
- Autonomous following behavior
- Independent execution


---

## 📦 Module 2: Question & Answer (Q/A) System

**Folder:** `QA.zip/`  
**Main File:** `app1.py`

### Description
This module implements an **AI-powered Q/A system** that answers laboratory-related questions.  
It uses **Flask** as the backend, **FAISS** for vector-based retrieval, and an **LLM** to generate accurate answers, along with **audio output**.

### Highlights
- Intelligent query answering
- Vector search using FAISS
- Audio-based responses
- Flask web interface
- Independent execution

---

## ⚠️ Important Notes
- ✅ Both modules are **independent**
- 📄 Each module contains its **own README**
- 🛠️ Follow **module-specific instructions** carefully to avoid errors
- 🚫 Do not mix execution steps between modules

---

## 🚀 Technologies Used
- Python
- YOLOv8 (Ultralytics)
- OpenCV
- ESP32
- Flask
- FAISS
- Large Language Models (LLM)
- Text-to-Speech (TTS)

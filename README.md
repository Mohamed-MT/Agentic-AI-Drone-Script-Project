# AI Drone Flight Agent | Natural Language Drone Control

Control a simulated drone using plain English.

This Python script uses multiple AI agents to understand instructions, plan missions, perform safety checks, execute flight commands and generate flight summaries.

Instead of manually programming every movement, you can simply type what you want the drone to do and the system will decide how to carry it out.

Everything runs inside a simulated environment (SITL), so **no physical drone is required**.

Examples:

```text
Take off to 20 metres.

Fly north 100 metres.

Fly over the hospital at 50 metres and return home.

Circle the current position.

Stop and hold position.

Resume the mission.
```

---

## What does this script do?

This project demonstrates how multiple AI agents can work together to control an autonomous system.

Each agent has its own responsibility, allowing complex tasks to be broken down into smaller and safer actions.

The system can:

* Understand natural language instructions
* Plan multi-step missions
* Perform safety checks before flying
* Execute drone commands
* Monitor the drone state
* Generate flight summaries
* Learn user preferences over time
* Load and execute reusable mission files using MCP

Everything runs inside a simulated environment, making it safe to experiment with and easy to learn.

---
<img width="3836" height="2024" alt="Screenshot 2026-06-19 232909-Picsart-AiImageEnhancer" src="https://github.com/user-attachments/assets/95ac562c-43b1-4d52-8db6-2eb6c06a7427" />
A natural language instruction is interpreted by the AI agent, converted into executable drone actions, and then carried out inside the ArduPilot SITL simulator.

## How it works

Whenever you enter an instruction, the system follows this process:

```text
User instruction

↓

Flight Agent

↓

Mission Planning

↓

Safety Validation

↓

Command Queue

↓

Drone Execution

↓

Flight Summary
```

The drone can continue flying in the background while you provide additional instructions.

---

## AI Agents

Several AI agents work together behind the scenes.

### Flight Agent

The main agent that understands your instructions and decides what actions need to be performed.

Example:

```text
Fly north 100 metres and return home.
```

### Mission Planner

Breaks larger requests into smaller steps.

Example:

```text
Fly over the hospital, circle twice, then return home.
```

becomes:

```text
1. Take off

2. Fly to the hospital

3. Circle the location

4. Return home

5. Land
```

### Safety Validator

Checks that missions are safe before execution.

Examples include:

* Maximum altitude limits
* Minimum altitude limits
* Speed constraints

Unsafe missions are rejected automatically.

### Session Summary Agent

Creates a readable summary once a mission is complete.

Example:

```text
The drone took off to 20 metres, flew north 100 metres, circled the target area and returned home successfully.
```

### Learning Machine

Learns from previous interactions, such as:

* Preferred altitude
* Preferred speed
* Frequently used locations
* Common mission patterns

---

## Technologies Used

This project uses:

* Python
* DroneKit
* ArduPilot SITL
* Agno
* OpenRouter
* PyMAVLink

Optional:

* Unreal Engine (telemetry visualisation)

---

## Environment Setup

Before running the script, you will need:

* Python 3.10 or newer
* An OpenRouter API key
* A DroneKit and ArduPilot SITL environment

If you do not already have the environment set up, follow this guide first:

🔗 https://github.com/igsxf22/flight_manual/blob/main/win_install_dronekit_2026.md

Once the environment is working:

1. Create a new Python file.

2. Copy and paste the script into the file.

3. At the beginning of the script, add your OpenRouter API key where it says:

```python
your_api_key_here
```

4. Save the file.

---

## Running the Script

1. Start ArduPilot SITL.

2. Make sure the simulator is running on:

```text
tcp:127.0.0.1:5763
```

3. Run the Python file.

```bash
python filename.py
```

You can now interact with the drone using natural language.

---

## Example Instructions

### Basic Flight

```text
Arm the drone.

Take off to 20 metres.

Land the drone.

Return home.
```

### Navigation

```text
Fly north 100 metres.

Fly east 200 metres.

Go to the hospital.

Circle the current position.
```

### Multi-Step Missions

```text
Take off to 30 metres.

Fly to the hospital.

Circle twice.

Return home.

Land.
```

Or simply:

```text
Fly over the hospital at 50 metres and return home.
```

### Continuous Flight

```text
Keep moving north.
```

Stop movement:

```text
Stop
```

Resume:

```text
Resume
```
## Mission Files (MCP)

The script includes an MCP (Model Context Protocol) integration that allows you to create reusable mission files.

Instead of typing the same instructions repeatedly, you can create a mission file, save it inside the `missions` folder and then ask the agent to use it.

This is useful for:

- Frequently used missions
- Testing predefined scenarios
- Creating reusable flight routines
- Organising larger multi-step missions

### Example

Create a text file inside the `missions` folder.

Example:

```text
missions/

hospital_patrol.txt
```

Inside the file:

```text
Take off to 30 metres.

Fly to the hospital.

Circle twice.

Return home.

Land.
```

Then ask the agent to use that mission.

Example:

```text
Run hospital_patrol mission.
```

or

```text
Load the hospital_patrol mission.
```

The agent will read the file and execute the instructions step by step.

> Mission files can be edited at any time, allowing you to build a library of reusable drone missions.
---
## Files Created Automatically

The first time the script runs, it will create several folders and files used to store missions, logs and reports.

These include:

- missions/
- logs/
- reports/
- drone_sessions.db

These are generated automatically and do not require any manual setup.

## Available Commands

| Command          | Description                   |
| ---------------- | ----------------------------- |
| `/status`        | Show live drone status        |
| `/battery`       | Show battery information      |
| `/position`      | Show GPS position             |
| `/locations`     | Show available locations      |
| `/model`         | Show the active AI model      |
| `/model <model>` | Switch AI models              |
| `/state`         | Show mission state            |
| `/report`        | Generate a flight report      |
| `/memory`        | Show learned preferences      |
| `/mission`       | Run the full mission workflow |

---

## Available Locations

The simulation includes several predefined locations:

```text
home
airfield
runway 35
runway 17
hospital
prison
camp a
camp b
reserve
residence 1
residence 2
creek south
location 1
```

---

## Safety Notes

This project was designed for simulation and research purposes.

Current limits:

```text
Maximum altitude: 120 m

Minimum altitude: 2 m

Maximum speed: 15 m/s
```

Battery values are simulated and should not be used for decision making.

Always review instructions before adapting the system for use with a real drone.

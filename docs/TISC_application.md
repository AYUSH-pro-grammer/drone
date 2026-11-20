# TISC 2026 application: Challenge 09, Drone Based Monitoring

**Team:** The Burnouts (Newton School of Technology, Bengaluru, S-VYASA)
**Contact:** ayushpro1428@gmail.com
**Department:** Energy

## Description submitted (1,500 character limit)

Problem: Telangana's feeders, lines and substations are still inspected mostly on foot. This is slow and subjective, so corrosion, conductor sag, vegetation encroachment, cracked insulators and hot joints are often found only after a trip or outage.

Our solution, GridDrishti, is a low-cost inspection drone paired with a ground-station AI. The drone streams live video, GPS and thermal data (MLX90640 IR sensor) to a laptop. Our AI pipeline detects:
- corrosion and cracked insulators (YOLO trained on public UAV power-line datasets such as InsPLAD, plus local images)
- vegetation inside clearance zones
- thermal hotspots, flagged by temperature rise over similar components
- conductor sag, estimated from the line's curve

Every defect is geo-tagged and given a severity score from its type, size, temperature rise and closeness to live parts. Defects appear on a map and in a ranked work list (P1 act now to P4 monitor) that exports for field crews.

Affordable and owned: parts cost about Rs 25,000 per drone, against lakhs for imported inspection drones. We are building our own ESP32 + MPU6050 flight controller to own the full stack, with a path to ArduPilot GPS waypoint missions for repeatable feeder surveys.

Stage: Prototype. The F450 drone is built on a KK2.1.5 controller. Our custom ESP32 flight controller is in bench testing. Live video-to-laptop defect detection, severity ranking and mapping work on test assets. Next: a pilot on one DISCOM feeder and substation.

## Key dates
- Applications close: 20 November 2026
- Demo Day: 1 December 2026, T-Hub, Hyderabad
- Startup Showcase: 7–9 December 2026

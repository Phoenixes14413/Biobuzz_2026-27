# AprilTag Detection for FTC - Complete Guide

## 📋 Overview
This guide will help you implement and train AprilTag detection for your FTC robot. AprilTags are visual fiducial markers that allow your robot to determine its position and orientation relative to the tag.

## 🎯 What You've Got

I've created 3 OpModes for you:

### 1. **AprilTagTrainingTeleOp** - START HERE! 
- **Purpose**: Test and train your AprilTag detection
- **Features**: Real-time tag detection with detailed information
- **Use this to**: Verify your camera works and tags are detected

### 2. **AprilTagDetectionAuto**
- **Purpose**: Basic autonomous navigation to AprilTags
- **Features**: Automatically drives toward detected tags
- **Use this to**: Simple autonomous positioning

### 3. **AprilTagNavigationAdvanced**
- **Purpose**: Advanced navigation with specific tag targeting
- **Features**: Search for specific tag IDs, precise positioning
- **Use this to**: Competition-level autonomous navigation

## 🚀 Quick Start Guide

### Step 1: Hardware Requirements
- ✅ Webcam (connected to Control Hub or Robot Controller)
- ✅ Printed AprilTags (download from: https://github.com/AprilRobotics/apriltag-imgs)
- ✅ Motors configured in your robot (optional for training)

### Step 2: Camera Configuration
1. Open the **FTC Robot Controller** app
2. Go to **Configure Robot**
3. Add a **Webcam** device
4. Name it **"Webcam 1"** (or update the code to match your camera name)

### Step 3: Print AprilTags
1. Go to: https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11
2. Download and print some tags (recommended: IDs 1-10)
3. Print on white paper, ensure tags are flat
4. Recommended size: 4-6 inches square for testing

### Step 4: Test Detection (TRAINING MODE)
1. Build and deploy the code to your robot
2. Select **"AprilTag Training TeleOp"** from the TeleOp menu
3. Press **INIT**, then **START**
4. Point your camera at an AprilTag
5. Watch the Driver Station screen for detection info

## 📊 Understanding the Output

When you run **AprilTagTrainingTeleOp**, you'll see:

```
═══════════════════════════════
   APRILTAG TRAINING MODE
═══════════════════════════════
# Tags Detected: 1

--- DETECTED TAGS ---
Tag ID: 2
Family: tag36h11
Center: X=320.5, Y=240.2
Hamming: 0
Decision Margin: 156.32

Position (FTC Pose):
  Range: 18.24 inches      ← Distance to tag
  Bearing: -5.2°           ← Left/right angle
  Yaw: 3.1°                ← Rotation angle
  X: 1.5 in                ← Horizontal offset
  Y: 2.3 in                ← Vertical offset
  Z: 18.1 in               ← Forward distance
```

### Key Metrics:
- **Range**: Distance from camera to tag (in inches)
- **Bearing**: Angle left (-) or right (+) of tag
- **Yaw**: How much the tag is rotated
- **Decision Margin**: Higher = better detection (>50 is good, >100 is excellent)

## 🎮 Training Your AprilTag Detection

### Exercise 1: Basic Detection
1. Place an AprilTag on a wall at robot height
2. Run **AprilTagTrainingTeleOp**
3. Move the robot to different distances (1-4 feet)
4. Verify detection works at each distance
5. Note the maximum detection range

### Exercise 2: Angle Testing
1. Keep tag straight ahead
2. Rotate the robot left and right
3. Find the maximum angle for detection (usually ~60°)
4. Check that bearing values update correctly

### Exercise 3: Lighting Conditions
1. Test in different lighting (bright, dim, mixed)
2. Ensure Decision Margin stays above 50
3. If margin is low, improve lighting or move closer

### Exercise 4: Multiple Tags
1. Place 2-3 tags in view
2. Verify all tags are detected
3. Note that the system reports each tag separately

## 🤖 Using Autonomous Navigation

### Basic Auto (AprilTagDetectionAuto)
1. Ensure motors are configured:
   - frontLeft
   - frontRight
   - backLeft
   - backRight
2. Select **"AprilTag Detection Auto"** from Autonomous menu
3. Place robot 2-3 feet from an AprilTag
4. Press **INIT**, then **START**
5. Robot will drive toward the tag

### Advanced Auto (AprilTagNavigationAdvanced)
1. Edit the code to set your target tag ID:
   ```java
   private int TARGET_TAG_ID = 2; // Change this number
   ```
2. Place multiple tags in the field
3. Robot will search for the specific tag and navigate to it

## ⚙️ Configuration & Customization

### Adjusting Motor Names
If your motors have different names, update this section in the code:
```java
frontLeft = hardwareMap.get(DcMotor.class, "YOUR_MOTOR_NAME");
frontRight = hardwareMap.get(DcMotor.class, "YOUR_MOTOR_NAME");
backLeft = hardwareMap.get(DcMotor.class, "YOUR_MOTOR_NAME");
backRight = hardwareMap.get(DcMotor.class, "YOUR_MOTOR_NAME");
```

### Adjusting Camera Name
If your camera has a different name, update:
```java
.setCamera(hardwareMap.get(WebcamName.class, "YOUR_CAMERA_NAME"))
```

### Tuning Navigation Parameters
In the autonomous files, adjust these for your robot:
```java
private static final double DESIRED_DISTANCE = 12.0; // Target distance (inches)
private static final double SPEED_GAIN = 0.02;       // Forward/backward speed
private static final double STRAFE_GAIN = 0.015;     // Left/right speed
private static final double TURN_GAIN = 0.01;        // Rotation speed
private static final double MAX_AUTO_SPEED = 0.5;    // Max forward speed
```

## 🐛 Troubleshooting

### No Tags Detected
- ✅ Check camera is plugged in and configured
- ✅ Ensure tag is printed clearly on white paper
- ✅ Verify good lighting conditions
- ✅ Make sure tag is facing camera directly
- ✅ Move closer (start within 2 feet)

### Low Decision Margin (<50)
- ✅ Improve lighting
- ✅ Ensure tag is flat (not bent or wrinkled)
- ✅ Move closer to tag
- ✅ Clean camera lens

### Robot Doesn't Move in Auto
- ✅ Check motor configuration names match code
- ✅ Verify motors are connected
- ✅ Check motor directions are correct
- ✅ Ensure AprilTag is detected (check telemetry)

### Wrong Pose Information
- ✅ Verify camera is mounted straight
- ✅ Check tag size matches reality (default: 4 inches)
- ✅ Ensure tag is not distorted when printed

## 📖 FTC AprilTag Usage for Competition

### Common FTC AprilTag Applications:
1. **Signal Sleeve Detection**: Identify randomization element
2. **Backdrop Alignment**: Precisely align to score
3. **Autonomous Positioning**: Navigate to specific field positions
4. **Relocalization**: Know where robot is on field

### FTC-Specific Tag IDs:
- Tags 1-10: Often used for game-specific elements
- Check your season's game manual for official tag placements

## 🎓 Next Steps

1. **Master Training Mode**: Spend time understanding detection ranges and angles
2. **Test Autonomous**: Try basic autonomous navigation
3. **Tune Parameters**: Adjust speed/distance for your robot
4. **Add Logic**: Implement game-specific behaviors based on tag detection
5. **Multi-Tag Navigation**: Use advanced mode to navigate between tags

## 📚 Additional Resources

- FTC AprilTag Documentation: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
- AprilTag Library: https://github.com/AprilRobotics/apriltag
- FTC SDK: https://github.com/FIRST-Tech-Challenge/FtcRobotController
- Print Tags: https://github.com/AprilRobotics/apriltag-imgs

## 💡 Pro Tips

1. **Lighting Matters**: AprilTag detection works best in consistent, bright lighting
2. **Flat Tags Only**: Bent or curved tags won't detect reliably
3. **Size Matters**: Larger tags detect from further away
4. **Start Simple**: Master detection before attempting navigation
5. **Log Everything**: Use telemetry to debug issues
6. **Test Early**: Don't wait until competition to test AprilTags!

---

## 🎯 Training Checklist

- [ ] Camera configured in Robot Controller
- [ ] AprilTags printed (at least 3 different IDs)
- [ ] Training TeleOp tested successfully
- [ ] Maximum detection range determined
- [ ] Maximum detection angle measured
- [ ] Lighting conditions tested
- [ ] Motors configured (if using autonomous)
- [ ] Basic autonomous navigation tested
- [ ] Navigation parameters tuned for your robot
- [ ] Multi-tag detection verified

---

**Good luck with your AprilTag implementation! 🚀**

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "ShootBall", group = "Concept")
public class ShootBall extends LinearOpMode {

    // Detected spot colors (persist after detection)
    private int spotOneDetectedColor = COLOR_NONE;
    private int spotTwoDetectedColor = COLOR_NONE;
    private int spotThreeDetectedColor = COLOR_NONE;

    // locked flags for whether a spot has been scanned and saved
    private boolean spotOneLocked = false;
    private boolean spotTwoLocked = false;
    private boolean spotThreeLocked = false;

    // The shooting order set by AprilTag detection (class-level, not shadowed)
    private int shootFirstColor = COLOR_NONE;
    private int shootSecondColor = COLOR_NONE;
    private int shootThirdColor = COLOR_NONE;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    TouchSensor positionOne;
    TouchSensor positionTwo;
    TouchSensor positionThree;

    NormalizedColorSensor colorSensor;

    // HSV hue ranges (0 - 360)
    private static final float GREEN_HUE_MIN = 85;
    private static final float GREEN_HUE_MAX = 165;
    private static final float PURPLE_HUE_MIN = 225;
    private static final float PURPLE_HUE_MAX = 365;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public static final int COLOR_NONE = 0;
    public static final int COLOR_GREEN = 1;
    public static final int COLOR_PURPLE = 2;

    // Stability requirements per spot.
    final int stableNeeded = 5;

    // Per-spot counters to avoid cross-talk between spots during scanning.
    int greenCountSpot1 = 0, purpleCountSpot1 = 0;
    int greenCountSpot2 = 0, purpleCountSpot2 = 0;
    int greenCountSpot3 = 0, purpleCountSpot3 = 0;

    @Override
    public void runOpMode() {

        initAprilTag();

        // hardware map
        positionOne = hardwareMap.get(TouchSensor.class, "mag_sensor_one");
        positionTwo = hardwareMap.get(TouchSensor.class, "mag_sensor_two");
        positionThree = hardwareMap.get(TouchSensor.class, "mag_sensor_three");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addLine("Ready - Press A to begin launch sequence once scanned");
        telemetry.update();

        // For single-press detection of gamepad1.A
        boolean prevA = false;
        boolean launchInProgress = false;

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Update AprilTag telemetry & set shootFirst/Second/Third
                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Pause/resume vision streaming controls
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Reset everything if X pressed
                if (gamepad1.x) {
                    resetAllSpots();
                    clearShootingOrder();
                    launchInProgress = false;
                }

                // SCAN each spot independently if not locked
                scanSpotOne();
                scanSpotTwo();
                scanSpotThree();

                // Telemetry for scanning state
                telemetry.addData("Spot 1 Locked", spotOneLocked);
                telemetry.addData("Spot 1 Color", colorName(spotOneDetectedColor));
                telemetry.addData("Spot 2 Locked", spotTwoLocked);
                telemetry.addData("Spot 2 Color", colorName(spotTwoDetectedColor));
                telemetry.addData("Spot 3 Locked", spotThreeLocked);
                telemetry.addData("Spot 3 Color", colorName(spotThreeDetectedColor));

                telemetry.addData("Shooting Order", colorName(shootFirstColor) + " , " + colorName(shootSecondColor) + " , " + colorName(shootThirdColor));
                telemetry.addData("Press X to reset", "");
                telemetry.update();

                // Start the launch sequence on a rising edge of gamepad1.A
                if (gamepad1.a && !prevA) {
                    // Only allow launching if we have a valid shooting order and at least one spot locked
                    if ((shootFirstColor != COLOR_NONE) && (spotOneLocked || spotTwoLocked || spotThreeLocked)) {
                        launchInProgress = true;
                        telemetry.addLine("Launch sequence triggered (A pressed)");
                        telemetry.update();
                        // Run the telemetry-only launch routine (executes the logic once)
                        runLaunchOnce();
                        launchInProgress = false;
                    } else {
                        telemetry.addLine("Cannot start launch: ensure AprilTag detected and at least one spot scanned");
                        telemetry.update();
                    }
                }
                prevA = gamepad1.a;

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }

    // ---- SCANNING HELPERS (per-spot stability using separate counters) ----
    private void scanSpotOne() {
        if (spotOneLocked) return;

        NormalizedRGBA c = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(c.toColor(), hsv);
        float hue = hsv[0];

        int detectedColor = COLOR_NONE;
        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            greenCountSpot1++;
            purpleCountSpot1 = 0;
            if (greenCountSpot1 >= stableNeeded) detectedColor = COLOR_GREEN;
        } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            purpleCountSpot1++;
            greenCountSpot1 = 0;
            if (purpleCountSpot1 >= stableNeeded) detectedColor = COLOR_PURPLE;
        } else {
            greenCountSpot1 = purpleCountSpot1 = 0;
            detectedColor = COLOR_NONE;
        }

        if (positionOne.isPressed() && (detectedColor == COLOR_GREEN || detectedColor == COLOR_PURPLE)) {
            spotOneDetectedColor = detectedColor;
            spotOneLocked = true;
            telemetry.addLine("Spot 1 saved: " + colorName(spotOneDetectedColor));
            telemetry.update();
        }
    }

    private void scanSpotTwo() {
        if (spotTwoLocked) return;

        NormalizedRGBA c = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(c.toColor(), hsv);
        float hue = hsv[0];

        int detectedColor = COLOR_NONE;
        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            greenCountSpot2++;
            purpleCountSpot2 = 0;
            if (greenCountSpot2 >= stableNeeded) detectedColor = COLOR_GREEN;
        } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            purpleCountSpot2++;
            greenCountSpot2 = 0;
            if (purpleCountSpot2 >= stableNeeded) detectedColor = COLOR_PURPLE;
        } else {
            greenCountSpot2 = purpleCountSpot2 = 0;
            detectedColor = COLOR_NONE;
        }

        if (positionTwo.isPressed() && (detectedColor == COLOR_GREEN || detectedColor == COLOR_PURPLE)) {
            spotTwoDetectedColor = detectedColor;
            spotTwoLocked = true;
            telemetry.addLine("Spot 2 saved: " + colorName(spotTwoDetectedColor));
            telemetry.update();
        }
    }

    private void scanSpotThree() {
        if (spotThreeLocked) return;

        NormalizedRGBA c = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(c.toColor(), hsv);
        float hue = hsv[0];

        int detectedColor = COLOR_NONE;
        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            greenCountSpot3++;
            purpleCountSpot3 = 0;
            if (greenCountSpot3 >= stableNeeded) detectedColor = COLOR_GREEN;
        } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            purpleCountSpot3++;
            greenCountSpot3 = 0;
            if (purpleCountSpot3 >= stableNeeded) detectedColor = COLOR_PURPLE;
        } else {
            greenCountSpot3 = purpleCountSpot3 = 0;
            detectedColor = COLOR_NONE;
        }

        if (positionThree.isPressed() && (detectedColor == COLOR_GREEN || detectedColor == COLOR_PURPLE)) {
            spotThreeDetectedColor = detectedColor;
            spotThreeLocked = true;
            telemetry.addLine("Spot 3 saved: " + colorName(spotThreeDetectedColor));
            telemetry.update();
        }
    }

    // ---- Launch logic (telemetry-only rotation guidance + launch messages) ----
    // This runs the three-step launch logic one time (per A press).
    private void runLaunchOnce() {
        // First
        if (shootFirstColor != COLOR_NONE) {
            performLaunchStep(shootFirstColor, 1);
        } else {
            telemetry.addLine("No first shoot color set.");
            telemetry.update();
        }

        // Second
        if (shootSecondColor != COLOR_NONE) {
            performLaunchStep(shootSecondColor, 2);
        } else {
            telemetry.addLine("No second shoot color set.");
            telemetry.update();
        }

        // Third
        if (shootThirdColor != COLOR_NONE) {
            performLaunchStep(shootThirdColor, 3);
        } else {
            telemetry.addLine("No third shoot color set.");
            telemetry.update();
        }

        telemetry.addLine("Launch sequence (telemetry-only) complete.");
        telemetry.update();
    }

    // performLaunchStep: looks for targetColor in the three spots and telemetry-guides rotation / launching.
    // stepIndex is for telemetry only (1st, 2nd, 3rd).
    private void performLaunchStep(int targetColor, int stepIndex) {
        telemetry.addLine(String.format("=== Step %d: Target %s ===", stepIndex, colorName(targetColor)));

        // If a spot holds the target and we are physically at that spot (touch pressed) -> launch
        if (spotOneDetectedColor == targetColor && positionOne.isPressed()) {
            telemetry.addLine("At Spot 1 and target found — Launch " + stepIndex);
            resetSpotOne(); // emulate launch: clear the stored value
            telemetry.update();
            return;
        } else if (spotOneDetectedColor == targetColor && !positionOne.isPressed()) {
            telemetry.addLine("Target at Spot 1. Rotate to Spot One.");
            telemetry.update();
            return;
        }

        if (spotTwoDetectedColor == targetColor && positionTwo.isPressed()) {
            telemetry.addLine("At Spot 2 and target found — Launch " + stepIndex);
            resetSpotTwo();
            telemetry.update();
            return;
        } else if (spotTwoDetectedColor == targetColor && !positionTwo.isPressed()) {
            telemetry.addLine("Target at Spot 2. Rotate to Spot Two.");
            telemetry.update();
            return;
        }

        if (spotThreeDetectedColor == targetColor && positionThree.isPressed()) {
            telemetry.addLine("At Spot 3 and target found — Launch " + stepIndex);
            resetSpotThree();
            telemetry.update();
            return;
        } else if (spotThreeDetectedColor == targetColor && !positionThree.isPressed()) {
            telemetry.addLine("Target at Spot 3. Rotate to Spot Three.");
            telemetry.update();
            return;
        }

        // If we get here, no locked spot matches target
        telemetry.addLine("Target color not found among scanned spots.");
        telemetry.update();
    }

    // ---- Reset helpers ----
    private void resetSpotOne() {
        spotOneDetectedColor = COLOR_NONE;
        spotOneLocked = false;
    }

    private void resetSpotTwo() {
        spotTwoDetectedColor = COLOR_NONE;
        spotTwoLocked = false;
    }

    private void resetSpotThree() {
        spotThreeDetectedColor = COLOR_NONE;
        spotThreeLocked = false;
    }

    private void resetAllSpots() {
        resetSpotOne();
        resetSpotTwo();
        resetSpotThree();
        // also reset counters
        greenCountSpot1 = purpleCountSpot1 = 0;
        greenCountSpot2 = purpleCountSpot2 = 0;
        greenCountSpot3 = purpleCountSpot3 = 0;
    }

    private void clearShootingOrder() {
        shootFirstColor = COLOR_NONE;
        shootSecondColor = COLOR_NONE;
        shootThirdColor = COLOR_NONE;
    }

    private String colorName(int c) {
        switch (c) {
            case COLOR_GREEN:
                return "GREEN";
            case COLOR_PURPLE:
                return "PURPLE";
            default:
                return "NONE";
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                // Keep your lens intrinsics (example values from your original code)
                .setLensIntrinsics(686.02605736, 686.02605736, 341.26208637, 208.068001489)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // not used in your config, but left for completeness
        }

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    /**
     * Add telemetry about AprilTag detections and set the shoot order (now properly sets class-level vars)
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Default to NONE until we see a known tag
        int localFirst = COLOR_NONE;
        int localSecond = COLOR_NONE;
        int localThird = COLOR_NONE;
        boolean foundKnownTag = false;

        for (AprilTagDetection detection : currentDetections) {

            // Look for a specific tag ID and set shooting order
            if (detection.id == 21) {
                localFirst = COLOR_GREEN;
                localSecond = COLOR_PURPLE;
                localThird = COLOR_PURPLE;
                telemetry.addLine("Pattern: GPP");
                foundKnownTag = true;
            } else if (detection.id == 22) {
                localFirst = COLOR_PURPLE;
                localSecond = COLOR_GREEN;
                localThird = COLOR_PURPLE;
                telemetry.addLine("Pattern: PGP");
                foundKnownTag = true;
            } else if (detection.id == 23) {
                localFirst = COLOR_PURPLE;
                localSecond = COLOR_PURPLE;
                localThird = COLOR_GREEN;
                telemetry.addLine("Pattern: PPG");
                foundKnownTag = true;
            } else {
                // unknown tag; do nothing special
            }

            if (detection.metadata != null) {
                StringBuilder detectionDetails = new StringBuilder();
                detectionDetails.append(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                detectionDetails.append(String.format("\nXYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                detectionDetails.append(String.format("\nPRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                detectionDetails.append(String.format("\nRBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addLine(detectionDetails.toString());
            }
        }

        // Update class-level shoot order if we found a known tag
        if (foundKnownTag) {
            shootFirstColor = localFirst;
            shootSecondColor = localSecond;
            shootThirdColor = localThird;
        }

        // Add key info to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}

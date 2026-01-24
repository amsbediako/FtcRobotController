package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Daisy One-Way Control", group = "Final")
public class ShootPot extends LinearOpMode {

    // --- Hardware ---

    private DcMotor rightShooter;

    private DcMotor leftShooter;
    CRServo axleServo;
    AnalogInput axlePot;
    NormalizedColorSensor colorSensor;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    // --- Constants ---
    static final double TOLERANCE = 0.08;  // Increased slightly for one-way reliability
    static final double SERVO_POWER = -0.7; // Speed for one-way travel

    // Target Voltages
    static final double POSITION_ONE = 0.0;
    static final double POSITION_TWO = 0.83;
    static final double POSITION_THREE = 1.64;
    static final double[] POSITIONS = {POSITION_ONE, POSITION_TWO, POSITION_THREE};

    // --- State & Logic ---
    public static final int COLOR_NONE = 0, COLOR_GREEN = 1, COLOR_PURPLE = 2;
    private static final float GREEN_HUE_MIN = 85, GREEN_HUE_MAX = 165;
    private static final float PURPLE_HUE_MIN = 225, PURPLE_HUE_MAX = 365;

    int[] spotColors = {COLOR_NONE, COLOR_NONE, COLOR_NONE};
    boolean[] spotLocked = {false, false, false};
    int[] shootingOrder = {COLOR_NONE, COLOR_NONE, COLOR_NONE};

    int greenCount = 0, purpleCount = 0;
    final int STABLE_NEEDED = 5;

    @Override
    public void runOpMode() {
        axleServo = hardwareMap.get(CRServo.class, "turnServo");
        axlePot = hardwareMap.get(AnalogInput.class, "axlePot");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        rightShooter = hardwareMap.get(DcMotor.class, "shoot_right");
        leftShooter = hardwareMap.get(DcMotor.class, "shoot_left");

        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        if (colorSensor instanceof SwitchableLight) ((SwitchableLight) colorSensor).enableLight(true);
        initAprilTag();

        telemetry.addLine("Ready - ONE-WAY MODE ACTIVE");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double currentVoltage = axlePot.getVoltage();
            int currentPosIndex = getClosestPosition(currentVoltage);

            updateAprilTagOrder();


            if (currentPosIndex != -1 && !spotLocked[currentPosIndex]) {
                scanCurrentSpot(currentPosIndex);
            }

            // Movement Controls
            if (gamepad1.a) moveToVoltage(POSITION_ONE);
            else if (gamepad1.b) moveToVoltage(POSITION_TWO);
            else if (gamepad1.x) moveToVoltage(POSITION_THREE);
            else if (gamepad1.y && currentPosIndex != -1) {
                int nextIndex = (currentPosIndex + 1) % POSITIONS.length;
                moveToVoltage(POSITIONS[nextIndex]);
            }

            if (gamepad1.start) resetSystem();
            if (gamepad1.right_bumper) runAutoLaunch();
            if (gamepad1.dpad_up) {
                startLauncher();
        } else{
                stopLauncher();
            }

            updateTelemetry(currentVoltage, currentPosIndex);
        }
        visionPortal.close();
    }

    private void moveToVoltage(double target) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < 3.0) { // 3-second safety timeout
            double current = axlePot.getVoltage();

            // Check if we reached target
            if (Math.abs(target - current) < TOLERANCE) {
                break;
            }

            // ALWAYS POSITIVE POWER - Only moves forward
            axleServo.setPower(SERVO_POWER);

            if (gamepad1.left_bumper) break; // Manual emergency stop

            telemetry.addData("Seeking Target", target);
            telemetry.addData("Current Volt", "%.3f", current);
            telemetry.update();
        }
        axleServo.setPower(0);
    }

    private int getClosestPosition(double voltage) {
        for (int i = 0; i < POSITIONS.length; i++) {
            if (Math.abs(voltage - POSITIONS[i]) < TOLERANCE) return i;
        }
        return -1;
    }

    private void scanCurrentSpot(int index) {
        NormalizedRGBA c = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(c.toColor(), hsv);
        float hue = hsv[0];

        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            greenCount++; purpleCount = 0;
            if (greenCount >= STABLE_NEEDED) {
                spotColors[index] = COLOR_GREEN;
                spotLocked[index] = true;
            }
        } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            purpleCount++; greenCount = 0;
            if (purpleCount >= STABLE_NEEDED) {
                spotColors[index] = COLOR_PURPLE;
                spotLocked[index] = true;
            }
        } else {
            greenCount = 0; purpleCount = 0;
        }
    }

    private void runAutoLaunch() {
        if (shootingOrder[0] == COLOR_NONE) return;

        for (int targetColor : shootingOrder) {
            for (int i = 0; i < 3; i++) {
                if (spotColors[i] == targetColor && spotLocked[i]) {
                    moveToVoltage(POSITIONS[i]);
                    shootBall();
                    sleep(800);
                    spotColors[i] = COLOR_NONE;
                    spotLocked[i] = false;
                    break;
                }
            }
        }
    }
    private void shootBall() {
        ///open lancher door
        startLauncher();
        sleep(500);
        double currentVoltage = axlePot.getVoltage();
        int currentPosIndex = getClosestPosition(currentVoltage);

        // If we are between spots, find the mathematically closest index
        if (currentPosIndex == -1) {
            double minDiff = Double.MAX_VALUE;
            for (int i = 0; i < POSITIONS.length; i++) {
                double diff = Math.abs(currentVoltage - POSITIONS[i]);
                if (diff < minDiff) {
                    minDiff = diff;
                    currentPosIndex = i;
                }
            }
        }

        if (currentPosIndex != -1) {
            spotColors[currentPosIndex] = COLOR_NONE;
            spotLocked[currentPosIndex] = false;
        }

        // Determine the next position in the sequence (0 -> 1 -> 2 -> 0)
        int nextIndex = (currentPosIndex + 1) % POSITIONS.length;

        // Execute the movement using your one-way logic
        moveToVoltage(POSITIONS[nextIndex]);
        sleep(500);


        stopLauncher();
        /// close launcher door
    }

    private void startLauncher(){
        rightShooter.setPower(.7);
        leftShooter.setPower(.7);
    }
    private void stopLauncher(){
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    private void resetSystem() {
        for (int i = 0; i < 3; i++) {
            spotColors[i] = COLOR_NONE;
            spotLocked[i] = false;
        }
        greenCount = 0; purpleCount = 0;
    }

    private void updateAprilTagOrder() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 21) {
                shootingOrder[0] = COLOR_GREEN; shootingOrder[1] = COLOR_PURPLE; shootingOrder[2] = COLOR_PURPLE;

            } else if (detection.id == 22) {
                shootingOrder[0] = COLOR_PURPLE; shootingOrder[1] = COLOR_GREEN; shootingOrder[2] = COLOR_PURPLE;

            } else if (detection.id == 23) {
                shootingOrder[0] = COLOR_PURPLE; shootingOrder[1] = COLOR_PURPLE; shootingOrder[2] = COLOR_GREEN;

            }
        }
        telemetry.update();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void updateTelemetry(double volt, int pos) {
        telemetry.addData("Voltage", "%.2f", volt);
        telemetry.addData("Position", pos == -1 ? "MOVING" : (pos + 1));
        telemetry.addLine("--- Spots ---");
        for (int i=0; i<3; i++) {
            telemetry.addData("Spot " + (i+1), spotLocked[i] ? colorName(spotColors[i]) : "EMPTY");
        }
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 21) {
                telemetry.addData("Pattern", "GPP");
            } else if (detection.id == 22) {
                telemetry.addData("Pattern", "PGP");
            } else if (detection.id == 23) {
                telemetry.addData("Pattern", "PPG");
            }
        }
        telemetry.update();
        telemetry.update();

    }

    private void shootColor(int targetColor) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        axleServo.setPower(SERVO_POWER); // always forward

        while (opModeIsActive() && timer.seconds() < 4.0) { // safety timeout
            double voltage = axlePot.getVoltage();
            int posIndex = getClosestPosition(voltage);

            // Only consider detection if we're at a valid position
            if (posIndex != -1) {
                NormalizedRGBA c = colorSensor.getNormalizedColors();
                float[] hsv = new float[3];
                Color.colorToHSV(c.toColor(), hsv);
                float hue = hsv[0];

                boolean greenDetected =
                        targetColor == COLOR_GREEN &&
                                hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX;

                boolean purpleDetected =
                        targetColor == COLOR_PURPLE &&
                                hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX;

                if (greenDetected || purpleDetected) {
                    axleServo.setPower(0);

                    // Lock this spot
                    spotColors[posIndex] = targetColor;
                    spotLocked[posIndex] = true;

                    // Shoot one ball
                    shootBall();

                    // Clear after shot

                    return;
                }
            }

            telemetry.addData("Seeking", colorName(targetColor));
            telemetry.addData("Voltage", "%.2f", voltage);
            telemetry.update();
        }

        axleServo.setPower(0); // timeout safety stop
    }


    private String colorName(int c) {
        if (c == COLOR_GREEN) return "GREEN";
        if (c == COLOR_PURPLE) return "PURPLE";
        return "NONE";
    }
}
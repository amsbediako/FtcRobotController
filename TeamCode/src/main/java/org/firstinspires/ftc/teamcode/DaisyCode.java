package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.graphics.Color;

@TeleOp(name = "Daisy Code", group = "Test")
public class DaisyCode extends LinearOpMode {

    // --- Hardware ---
    CRServo axleServo;
    AnalogInput axlePot;
    NormalizedColorSensor colorSensor;

    //potentiometer offset


    // --- Potentiometer calibration ---
    static final double POT_MIN_VOLTAGE = 0;
    static final double POT_MAX_VOLTAGE = 3.3;
    static final double POT_MAX_VOLTAGE_ACTUAL = 2.2;

    // Servo control
    static final double TOLERANCE = 0.02;
    static final double SERVO_POWER = 0.3;

    // Predefined positions
    static final double POSITION_ONE = 0 ;
    static final double POSITION_TWO = 1.466 ;
    static final double POSITION_THREE = 0.7333;
    static final double[] POSITIONS = {POSITION_ONE, POSITION_TWO, POSITION_THREE};

    // Color detection
    public static final int COLOR_NONE = 0;
    public static final int COLOR_GREEN = 1;
    public static final int COLOR_PURPLE = 2;

    private static final float GREEN_HUE_MIN = 85;
    private static final float GREEN_HUE_MAX = 165;
    private static final float PURPLE_HUE_MIN = 225;
    private static final float PURPLE_HUE_MAX = 365;

    // --- State ---
    Double targetVoltage = null;
    boolean moving = false;
    String targetName = "";

    int[] spotColors = {COLOR_NONE, COLOR_NONE, COLOR_NONE};
    final int stableNeeded = 5;
    int greenCount = 0;
    int purpleCount = 0;
    float[] hsv = new float[3];

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        axleServo = hardwareMap.get(CRServo.class, "turnServo");
        axlePot = hardwareMap.get(AnalogInput.class, "axlePot");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // --- Read potentiometer ---
            double voltage = axlePot.getVoltage();

            // --- Detect which position servo is closest to ---
            int currentPositionIndex = 0;
            double minDiff = Double.MAX_VALUE;

            for (int i = 0; i < POSITIONS.length; i++) {
                double diff = Math.abs(voltage - POSITIONS[i]);
                if (diff < minDiff) {
                    minDiff = diff;
                    currentPositionIndex = i;
                }
            }


            // --- Read color sensor ---
            NormalizedRGBA c = colorSensor.getNormalizedColors();
            Color.colorToHSV(c.toColor(), hsv);
            float hue = hsv[0];

            int detectedColor = COLOR_NONE;
            if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
                greenCount++;
                purpleCount = 0;
                if (greenCount >= stableNeeded) detectedColor = COLOR_GREEN;
            } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
                purpleCount++;
                greenCount = 0;
                if (purpleCount >= stableNeeded) detectedColor = COLOR_PURPLE;
            } else {
                greenCount = purpleCount = 0;
                detectedColor = COLOR_NONE;
            }

            // --- Lock detected color to current position ---
            if (currentPositionIndex != -1 && detectedColor != COLOR_NONE && spotColors[currentPositionIndex] == COLOR_NONE) {
                spotColors[currentPositionIndex] = detectedColor;
            }

            // --- Handle manual movement buttons ---
            if (!moving) {
                if (gamepad1.a) { targetVoltage = POSITION_ONE; targetName = "Position 1"; moving = true; }
                else if (gamepad1.b) { targetVoltage = POSITION_TWO; targetName = "Position 2"; moving = true; }
                else if (gamepad1.x) { targetVoltage = POSITION_THREE; targetName = "Position 3"; moving = true; }
                else if (gamepad1.y && currentPositionIndex != -1) { // move to next position
                    int nextIndex = (currentPositionIndex + 1) % POSITIONS.length;
                    targetVoltage = POSITIONS[nextIndex];
                    targetName = "Position " + (nextIndex + 1);
                    moving = true;
                }
            }

            // --- Move servo toward target ---
            if (moving && targetVoltage != null) {
                double diff = targetVoltage - voltage;
                if (diff < 0) diff += (POT_MAX_VOLTAGE - POT_MIN_VOLTAGE);

                if (diff > TOLERANCE) axleServo.setPower(SERVO_POWER);
                else {
                    axleServo.setPower(0);
                    moving = false;
                    targetVoltage = null;
                }
            } else axleServo.setPower(0);

            // --- Compute angle ---
            double angleDeg = 360.0 * (voltage - POT_MIN_VOLTAGE) / (POT_MAX_VOLTAGE_ACTUAL - POT_MIN_VOLTAGE) ;
            if (angleDeg < 0) angleDeg += 360;
            if (angleDeg >= 360) angleDeg -= 360;

            // --- Reset colors with right bumper ---
            if (gamepad1.right_bumper) {
                for (int i = 0; i < spotColors.length; i++) spotColors[i] = COLOR_NONE;
            }

            // --- Telemetry ---
            telemetry.addData("Angle (deg)", "%.1f", angleDeg);
            for (int i = 0; i < spotColors.length; i++) {
                telemetry.addData("Position " + (i+1) + " Color", colorName(spotColors[i]));
            }
            if (moving) telemetry.addData("Moving to", targetName);
            telemetry.addData("Press Right Bumper to reset", "");
            telemetry.addData("Press Y to go to next position", "");
            telemetry.update();
        }
    }

    private String colorName(int c) {
        switch (c) {
            case COLOR_GREEN: return "GREEN";
            case COLOR_PURPLE: return "PURPLE";
            default: return "NONE";
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Potentiometer Test", group = "Test")
public class PotentiometerTest extends LinearOpMode {

    CRServo axleServo;
    AnalogInput axlePot;

    // ---- CALIBRATION VALUES ----
    static final double POT_MIN_VOLTAGE = 0;
    static final double POT_MAX_VOLTAGE = 3.3;
    static final double POT_MAX_VOLTAGE_ACTUAL = 2.2;

    // Voltages corresponding to positions
    double positionOne = 0;       // 0°
    double positionTwo = 0.7333;  // 120°
    double positionThree = 1.466; // 240°

    static final double TOLERANCE = 0.02; // volts tolerance for stopping
    static final double SERVO_POWER = -0.3; // only one direction

    // --- State variables ---
    Double targetVoltage = null; // the voltage we want to move to
    boolean moving = false;       // are we currently moving?
    String targetName = "";       // name of the target position

    @Override
    public void runOpMode() {

        axleServo = hardwareMap.get(CRServo.class, "turnServo");
        axlePot = hardwareMap.get(AnalogInput.class, "axlePot");

        waitForStart();

        while (opModeIsActive()) {

            double voltage = axlePot.getVoltage();

            // --- Check button presses to set target ---
            if (gamepad1.a && !moving) {
                targetVoltage = positionOne;
                targetName = "Position 1";
                moving = true;
            } else if (gamepad1.b && !moving) {
                targetVoltage = positionTwo;
                targetName = "Position 2";
                moving = true;
            } else if (gamepad1.x && !moving) {
                targetVoltage = positionThree;
                targetName = "Position 3";
                moving = true;
            }

            // --- Move servo toward target (one direction, wrap around) ---
            if (moving && targetVoltage != null) {
                // Calculate forward distance accounting for wrap-around
                double diff = targetVoltage - voltage;
                if (diff < 0) {
                    diff += (POT_MAX_VOLTAGE - POT_MIN_VOLTAGE);
                }

                if (diff > TOLERANCE) {
                    axleServo.setPower(SERVO_POWER); // always same direction
                } else {
                    axleServo.setPower(0);
                    moving = false; // stop moving when target reached
                }
            } else {
                axleServo.setPower(0);
            }

            // --- Compute angle 0°–360° ---
            double angleDeg = 360.0 * (voltage - POT_MIN_VOLTAGE) / (POT_MAX_VOLTAGE_ACTUAL - POT_MIN_VOLTAGE);
            if (angleDeg < 0) angleDeg += 360.0;
            if (angleDeg >= 360.0) angleDeg -= 360.0;

            // --- Telemetry ---
            telemetry.addData("Angle (deg)", "%.1f", angleDeg);

            if (moving) {
                telemetry.addData("Moving to", targetName);
            } else if (targetVoltage != null) {
                telemetry.addData("At Position", targetName);
                targetVoltage = null; // clear after showing reached
            }

            telemetry.update();
        }
    }
}

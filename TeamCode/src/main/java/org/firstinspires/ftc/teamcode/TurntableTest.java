package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TurntableTest extends LinearOpMode {

    private DcMotor turntableMotor;
    private SparkFunOTOS myOtos;

    // ===== ENCODER CONSTANTS =====
    public static final double MOTOR_ENCODER_PPR = 7.0;
    public static final double GEAR_RATIO = 90.0 / 15.0; // 6:1
    public static final double COUNTS_PER_OUTPUT_REV = MOTOR_ENCODER_PPR * GEAR_RATIO; // 42
    public static final double COUNTS_PER_DEGREE = COUNTS_PER_OUTPUT_REV / 360.0; // 0.1167

    // ===== FIELD CONSTANTS =====
    private static final double TARGET_FIELD_X = 5.5 * 24;
    private static final double TARGET_FIELD_Y = 5.5 * 24;
    private static final double ROBOT_START_FIELD_X = 2.5 * 24;
    private static final double ROBOT_START_FIELD_Y = 1.0;

    @Override
    public void runOpMode() {

        turntableMotor = hardwareMap.get(DcMotor.class, "camera_motor");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

        // Reset turret encoder
        turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.b) {
                double errorDeg = computeTurretErrorDegrees();
                moveTurretByError(errorDeg, 0.8);
            }

            telemetry.addData("Turret Encoder", turntableMotor.getCurrentPosition());
            telemetry.addData(
                    "Turret Angle (deg)",
                    turntableMotor.getCurrentPosition() / COUNTS_PER_DEGREE
            );
            telemetry.update();
        }
    }

    /**
     * Computes the angular error (degrees) between robot heading and target.
     * Positive = rotate turret CCW
     */
    private double computeTurretErrorDegrees() {

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        if (pos == null) return 0.0;

        double robotX = pos.x + ROBOT_START_FIELD_X;
        double robotY = pos.y + ROBOT_START_FIELD_Y;

        double robotHeading = normalizeAngle(pos.h);

        double dx = TARGET_FIELD_X - robotX;
        double dy = TARGET_FIELD_Y - robotY;

        double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        double error = angleToTarget - robotHeading;

        error = normalizeAngle(error);

        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading", robotHeading);
        telemetry.addData("Angle To Target", angleToTarget);
        telemetry.addData("Turret Error", error);

        return error;
    }

    /**
     * Moves turret by a RELATIVE angle error
     */
    private void moveTurretByError(double errorDegrees, double maxPower) {

        if (Math.abs(errorDegrees) < 1.5) {
            turntableMotor.setPower(0);
            return;
        }

        double currentDeg =
                turntableMotor.getCurrentPosition() / COUNTS_PER_DEGREE;

        double targetDeg = currentDeg + errorDegrees;
        int targetCounts = (int) Math.round(targetDeg * COUNTS_PER_DEGREE);

        turntableMotor.setTargetPosition(targetCounts);
        turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turntableMotor.setPower(Math.abs(maxPower));
    }

    /**
     * Normalizes angle to [-180, 180)
     */
    private double normalizeAngle(double angle) {
        angle = ((angle + 180) % 360 + 360) % 360 - 180;
        return angle;
    }
}

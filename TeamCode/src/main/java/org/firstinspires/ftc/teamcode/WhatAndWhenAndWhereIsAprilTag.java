package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "WhatAndWhereAndWhenIsAprilTag ", group = "Concept")
public class WhatAndWhenAndWhereIsAprilTag extends LinearOpMode {

    // --- PID CLASS ---
    public class PID {
        private double kP, kI, kD;
        private double integral = 0.0;
        private double lastError = 0.0;
        private double derivativeFilter = 0.0;
        private final double derivTau = 0.02;

        public PID(double kP, double kI, double kD) {
            this.kP = kP; this.kI = kI; this.kD = kD;
        }

        public double update(double error, double dt) {
            if (dt <= 0) return 0.0;

            integral += error * dt;

            double rawDeriv = (error - lastError) / dt;
            double alpha = dt / (derivTau + dt);
            derivativeFilter += alpha * (rawDeriv - derivativeFilter);

            lastError = error;

            return (kP * error) + (kI * integral) + (kD * derivativeFilter);
        }

        public void reset() {
            integral = 0.0; lastError = 0.0; derivativeFilter = 0.0;
        }

        public void clampIntegral(double min, double max) {
            if (integral > max) integral = max;
            if (integral < min) integral = min;
        }
    }

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor cameraMotor;

    @Override
    public void runOpMode() {

        // --- HARDWARE INIT ---
        cameraMotor = hardwareMap.get(DcMotor.class, "camera_motor");
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cameraMotor.setDirection(DcMotor.Direction.REVERSE); // adjust if needed

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        // --- PID for tracking ---
        PID turnPID = new PID(0.012, 0.0002, 0.0012); // tuned for motor→camera gear

        final double MAX_POWER = 0.25;        // max motor power
        final double DEADBAND_DEG = 4.0;      // camera degrees deadband

        // --- SWEEP PARAMETERS ---
        double filteredBearing = 0.0;
        final double LPF_ALPHA = 0.3;         // smooth bearing filter
        double sweepAngle = 0.0;              // approximate camera angle
        boolean sweepRight = true;
        final double SWEEP_POWER = 0.3;       // motor power for sweep
        final double SWEEP_MAX_ANGLE = 90.0;  // camera sweep limit
        final double EDGE_SLOW_DISTANCE = 20.0; // slow near edges

        // --- GEAR RATIO ---
        final double GEAR_RATIO = 90.0 / 16.0; // motor-to-camera

        telemetry.addLine("Ready - press START");
        telemetry.update();
        waitForStart();

        // start turret centered
        cameraMotor.setPower(0.0);
        sleep(100); // allow motor to settle

        long lastNs = System.nanoTime();

        while (opModeIsActive()) {
            long nowNs = System.nanoTime();
            double dt = (nowNs - lastNs) / 1e9; // seconds
            lastNs = nowNs;

            // --- GET APRILTAG DETECTIONS ---
            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection tag24 = null;
            for (AprilTagDetection d : detections) {
                if (d.id == 24) { tag24 = d; break; }
            }

            if (tag24 != null && tag24.ftcPose != null) {
                // -------- TRACK MODE --------
                double rawBearing = tag24.ftcPose.bearing;
                filteredBearing = LPF_ALPHA * filteredBearing + (1.0 - LPF_ALPHA) * rawBearing;
                double bearingError = filteredBearing;

                // PID scaled for motor → camera gear ratio
                double motorError = bearingError * GEAR_RATIO;
                double power = turnPID.update(motorError, dt);
                turnPID.clampIntegral(-10, 10); // limit integral

                // clamp power
                power = Math.max(Math.min(power, MAX_POWER), -MAX_POWER);

                // deadband
                if (Math.abs(bearingError) <= DEADBAND_DEG || Math.abs(power) < 0.02) {
                    cameraMotor.setPower(0.0);
                    turnPID.reset();
                    telemetry.addLine("CENTERED ✔");
                } else {
                    cameraMotor.setPower(power);
                    telemetry.addLine(String.format("TRACKING TAG 24, POWER: %.3f", power));
                }

            } else {
                // -------- SWEEP MODE --------
                turnPID.reset();

                // increment sweep angle (camera degrees)
                double sweepIncrement = SWEEP_POWER * dt * 100;
                sweepAngle += sweepRight ? sweepIncrement : -sweepIncrement;

                // reverse at sweep limits
                if (sweepAngle >= SWEEP_MAX_ANGLE) { sweepRight = false; sweepAngle = SWEEP_MAX_ANGLE; }
                else if (sweepAngle <= -SWEEP_MAX_ANGLE) { sweepRight = true; sweepAngle = -SWEEP_MAX_ANGLE; }

                // slow down near edges
                double distanceToEdge = sweepRight ? SWEEP_MAX_ANGLE - sweepAngle : sweepAngle + SWEEP_MAX_ANGLE;
                double scale = Math.min(1.0, distanceToEdge / EDGE_SLOW_DISTANCE);
                scale = Math.max(0.2, scale);

                // apply motor power directly
                double appliedPower = SWEEP_POWER * scale;
                cameraMotor.setPower(sweepRight ? appliedPower : -appliedPower);

                telemetry.addLine(String.format("SWEEPING... Angle: %.1f°, Power: %.3f", sweepAngle, appliedPower));

                sleep(15);
            }

            telemetry.update();
        }

        // --- CLEANUP ---
        cameraMotor.setPower(0.0);
        visionPortal.close();
    }
}

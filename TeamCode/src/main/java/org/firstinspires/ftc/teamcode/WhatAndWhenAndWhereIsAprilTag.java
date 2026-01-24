package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "WhatAndWhenAndWhereIsAprilTag", group = "Concept")
public class WhatAndWhenAndWhereIsAprilTag extends LinearOpMode {

    //CLASSES
    public class PID {
        private double kP, kI, kD;
        private double integral = 0.0;
        private double lastError = 0.0;
        private double derivativeFilter = 0.0;
        private final double derivTau = 0.02;

        public PID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
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
            integral = 0.0;
            lastError = 0.0;
            derivativeFilter = 0.0;
        }

        public void clampIntegral(double min, double max) {
            integral = Math.max(min, Math.min(max, integral));
        }
    }

    //MOTORS ETC

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor cameraMotor;

    private PID turnPID = new PID(0.006, 0.00005, 0.00045);

    private double filteredBearing = 0.0;

    //CONSTANTS

    private static final int TARGET_TAG_ID = 24;
    private static final double MAX_POWER = 0.20;
    private static final double DEADBAND_DEG = 4.0;
    private static final double LPF_ALPHA = 0.6;

    //WOULD GO IN TELEOP

    public void runOpMode() {
        initHardware();
        initVision();

        telemetry.addLine("Ready - press START");
        telemetry.update();
        waitForStart();

        long lastNs = System.nanoTime();

        while (opModeIsActive()) {
            double dt = getDeltaTime(lastNs);
            lastNs = System.nanoTime();

            AprilTagDetection tag = getTagById(TARGET_TAG_ID);

            if (tag != null && tag.ftcPose != null) {
                handleTagTracking(tag, dt);
            } else {
                stopTurret();
                telemetry.addLine("Tag 24 NOT FOUND");
            }

            telemetry.update();
            sleep(15);
        }

        visionPortal.close();
    }

    //METHODS 

    private void initHardware() {
        cameraMotor = hardwareMap.get(DcMotor.class, "camera_motor");
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cameraMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initVision() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    private void handleTagTracking(AprilTagDetection tag, double dt) {
        double rawBearing = tag.ftcPose.bearing;
        filteredBearing = lowPassFilter(filteredBearing, rawBearing);
        double error = filteredBearing;

        if (Math.abs(error) <= DEADBAND_DEG) {
            stopTurret();
            telemetry.addLine("CENTERED ✔");
        } else {
            applyPID(error, dt);
        }

        telemetry.addData("rawBearing", rawBearing);
        telemetry.addData("filteredBearing", filteredBearing);
    }

    private void applyPID(double error, double dt) {
        double power = turnPID.update(error, dt);
        turnPID.clampIntegral(-100.0, 100.0);

        power = clamp(power, -MAX_POWER, MAX_POWER);
        if (Math.abs(power) < 0.02) power = 0.0;

        cameraMotor.setPower(power);
        telemetry.addData("pidPower", power);
    }

    private void stopTurret() {
        cameraMotor.setPower(0.0);
        turnPID.reset();
    }


    private AprilTagDetection getTagById(int id) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == id) return d;
        }
        return null;
    }

    private double lowPassFilter(double previous, double current) {
        return (LPF_ALPHA * previous) + ((1.0 - LPF_ALPHA) * current);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double getDeltaTime(long lastNs) {
        return (System.nanoTime() - lastNs) / 1e9;
    }
}













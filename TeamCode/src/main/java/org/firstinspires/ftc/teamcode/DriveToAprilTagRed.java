package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="DriveToAprilTagRed", group="Concept")
public class DriveToAprilTagRed extends LinearOpMode {

    // Distance to stop from the tag
    final double DESIRED_DISTANCE = 5.0;

    // Control gains
    final double SPEED_GAIN  = 0.02;
    final double STRAFE_GAIN = 0.02;
    final double TURN_GAIN   = 0.015;

    // Power limits
    final double MAX_AUTO_SPEED  = 0.5;
    final double MAX_AUTO_STRAFE = 0.5;
    final double MAX_AUTO_TURN   = 0.3;

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 24;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() {

        initAprilTag();

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Drive directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);


        if (USE_WEBCAM)
            setManualExposure(6, 250);

        telemetry.addLine("Press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            boolean targetFound = false;
            desiredTag = null;

            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection tag : detections) {
                if (tag.id == DESIRED_TAG_ID && tag.metadata != null) {
                    targetFound = true;
                    desiredTag = tag;
                    break;
                }
            }

            double drive = 0;
            double strafe = 0;
            double turn = 0;

            if (targetFound) {

                double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;

                // Correct strafing — based on real horizontal displacement
                double strafeError = desiredTag.ftcPose.x;

                // Stabilize bearing to avoid noisy oscillations
                double headingError = desiredTag.ftcPose.bearing;
                if (Math.abs(headingError) < 1.0) headingError = 0;

                drive  = Range.clip(rangeError  * SPEED_GAIN,  -MAX_AUTO_SPEED,  MAX_AUTO_SPEED);
                strafe = Range.clip(strafeError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                turn   = Range.clip(headingError * TURN_GAIN,  -MAX_AUTO_TURN,   MAX_AUTO_TURN);

                telemetry.addLine("AUTO DRIVING TO TAG");
                telemetry.addData("Drive", "%.2f", drive);
                telemetry.addData("Strafe", "%.2f", strafe);
                telemetry.addData("Turn", "%.2f", turn);

            } else {
                telemetry.addLine("NO TAG — Use joysticks to find tag");
            }

            telemetry.update();

           moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /** Mecanum kinematics */
    public void moveRobot(double x, double y, double yaw) {

        double fl = x - y - yaw;
        double fr = x + y + yaw;
        double bl = x + y - yaw;
        double br = x - y + yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));

        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    /** AprilTag initialization */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {

        if (visionPortal == null) return;

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
                && !isStopRequested()) {
            sleep(10);
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }

        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        gainControl.setGain(gain);
        sleep(20);
    }
}

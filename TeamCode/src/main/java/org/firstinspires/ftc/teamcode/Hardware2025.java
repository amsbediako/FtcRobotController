package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Translation2d;
//import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Hardware2025 {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor leftSlide = null;
    public DcMotor rightSlide = null;
    public DcMotor arm = null;

    // Define IMU object and headings (Make it private so it can't be accessed externally)
    public IMU imu = null;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;

    // Run time (public)
    private final ElapsedTime runtime = new ElapsedTime();
   public DistanceSensor sensorDistance;

    // Color sensing
    public enum sampleColor {RED, YELLOW, BLUE, NONE} //color sensing enum

    private NormalizedColorSensor colorSensor;
    private float colorSensorGain = 20;

    //Touch sensor
    public TouchSensor touchSensor;
    // Magnetic sensing
    public TouchSensor magneticSensor;

    public enum SlidePosition {ZERO, NONE}

    // Slide positions
    public final double WALL_POSITION = 0;
    public final double LOW_POSITION = 6;
    public final double HIGH_POSITION = 25;
    public SlidePosition slideTargetPosition = SlidePosition.NONE;
    public final double ROBOT_AT_BAR = 18.2;
    public final double SCORING_POSITION = 18.0;
    public final double HANG_POSITION = 3.4;

    public void setSlideTargetPosition(SlidePosition slideTargetPosition) {
        this.slideTargetPosition = slideTargetPosition;
    }

    //Drive constants
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: our Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 100.0 / 25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double COUNTS_PER_REVOLUTION_SLIDE = 560;
    static final double SLIDE_GEAR_REDUCTION = 2;
    static final double COUNTS_PER_INCH_SLIDE = (COUNTS_PER_REVOLUTION_SLIDE) /
            (1.25984 * Math.PI * SLIDE_GEAR_REDUCTION);

    private double turnSpeed = 0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 5.0;
    static final double OPEN_SERVO_CLAW = 0.2;
    static final double CLOSE_SERVO_CLAW = 0.03;
    private static final double BEAK_OPEN = 0.3;
    private static final double BEAK_CLOSE = .5;
    private int slideTarget;
    private int rightSlideTarget;
    private int leftSlideTarget;
    private int armTarget;
    private double slideTimeout;
    private double armTimeout;

    // Claw and beak servos and sensors
    Servo clawServo;
    Servo beakServo;


    // Create an instance of the otos sensor
    SparkFunOTOS myOtos;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware2025(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        leftSlide = myOpMode.hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = myOpMode.hardwareMap.get(DcMotor.class, "right_slide");
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");


        myOtos = myOpMode.hardwareMap.get(SparkFunOTOS.class, "sensor_otos"); //Otos sensor
        //configureOtos();

        // Define and Initialize sensors
        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        magneticSensor = myOpMode.hardwareMap.get(TouchSensor.class, "magnetic_sensor");
        touchSensor = myOpMode.hardwareMap.get(TouchSensor.class, "sensor_touch");
        sensorDistance = myOpMode.hardwareMap.get(DistanceSensor.class, "distance_sensor");
        clawServo = myOpMode.hardwareMap.get(Servo.class, "claw_servo");
        beakServo = myOpMode.hardwareMap.get(Servo.class, "beak_servo");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setDirection(DcMotor.Direction.FORWARD);
       // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        resetHeading();
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    // end method initTfod()

    //Configuring the otos sensor- for more detailed comments, refer to the SensorSparkFunOTOS.java file
    public void configureOtos() {
        myOpMode.telemetry.addLine("Configuring OTOS...");
        myOpMode.telemetry.update();

        // Set the desired units for linear and angular measurements- currently set to the default inches and degrees
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        //Specify the offset for the sensor relative to the center of the robot.- chnsge
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(.25, 4.5, 0);
        myOtos.setOffset(offset);

        // Set the linear and angular scalars, to compensate for scaling issues with the sensor measurements.
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();

        // Reset the tracking algorithm
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS reports the robot is at origin. If you do not start at the origin, set the OTOS location to match.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        myOpMode.telemetry.addLine("OTOS configured! Press start to get position data!");
        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        myOpMode.telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        myOpMode.telemetry.update();
    }

    private double calculatePID(double error, double prevError, double integral, double kP, double kI, double kD, double loopDelay) {
        double derivative = (error - prevError) / loopDelay; //this is in seconds- should it be
        // Clip derivative to prevent spikes
        //derivative = Range.clip(derivative, -0.1, 0.1);
        double output = (kP * error) + (kI * integral) + (kD * derivative);
        Log.i("FTC18 calculatePID", String.format("PID output- error %.2f, integral: %.2f, derivative: %.2f, output: %.2f", error, integral, derivative, output));
        return output;
    }

//    public void driveByOtos(double distanceToTravelX, double distanceToTravelY, double targetHeadingDegrees, double timeout) {
//        runtime.reset();
//        //distanceToTravelX *= 0.32;
//        //distanceToTravelY *= 1.1;
//        // Get initial position from OTOS sensor
//        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//        Translation2d currentT = new Translation2d(pos.x, pos.y);
//        Rotation2d currentR = Rotation2d.fromDegrees(pos.h);
//        Pose2d current = new Pose2d(currentT, currentR);
//
//        // Compute target position
//        Translation2d targetT = currentT.plus(new Translation2d(distanceToTravelX, distanceToTravelY));
//        Rotation2d targetR = Rotation2d.fromDegrees(targetHeadingDegrees);
//        Pose2d target = new Pose2d(targetT, targetR);
//
//        // Use smaller thresholds for a smoother preciser stop
//        double thresholdDistance = 1.0;
//        double angleThreshold = 0.05; //radians should be smaller
//        double correctionX = 0, correctionY = 0, correctionH = 0;
//
//        double integralX = 0, prevErrorX = 0;
//        double integralY = 0, prevErrorY = 0;
//        double integralH = 0, prevErrorH = 0;
//
//        // PID constants (tweak these as needed)
//        //not sure if integeral works
//        double kP_X = 0.25, kI_X = 0, kD_X = 0.02;
//        double kP_Y = 0.25, kI_Y = 0, kD_Y = 0.02;
//        double kP_H = .25, kI_H = 0, kD_H = 0.0;
//
//        // So the robot can overcome friction
//        double minPower = 0.2;
//
//        double lastTime = runtime.seconds();
//        double toTravelPrevA = 0;
//        double toTravelPrevB = 0;
//        double toTravelPrevC = 0;
//
//        while (runtime.seconds() < timeout && myOpMode.opModeIsActive()) {
//            double now = runtime.seconds();
//
//            double loopDelay = (now - lastTime);  // seconds
//            lastTime = now;
//            if (loopDelay < 0.001) {
//                loopDelay = 0.04;  //to prevent it being so small there is a NaN error- maybe not needed now
//            }
////            pos = myOtos.getPosition();
////
////
////            currentT = new Translation2d(pos.x, pos.y);
////            currentR = Rotation2d.fromDegrees(pos.h);
////            current = new Pose2d(currentT, currentR);
////
////            Translation2d toTravelT = target.getTranslation().minus(current.getTranslation());
////            Rotation2d toTravelR = target.getRotation().minus(current.getRotation());
////
////            double toTravelD = current.getTranslation().getDistance(target.getTranslation());
////
////
////            // If within thresholds, exit the loop
////            //if (toTravelAvg < thresholdDistance && toTravelD < thresholdDistance && Math.abs(toTravelR.getRadians()) < angleThreshold) {
////
////            if (Math.abs(toTravelPrevC) < thresholdDistance && Math.abs(toTravelPrevB) < thresholdDistance && Math.abs(toTravelPrevA) < thresholdDistance && Math.abs(toTravelD) < thresholdDistance && Math.abs(toTravelR.getRadians()) < angleThreshold) {
////
////                    Log.i("FTC18 driveByOtos", String.format("Leaving loop - X: %.2f, Y: %.2f, H: %.2f", pos.x, pos.y, pos.h));
////                    break;
////                }
////
////            toTravelPrevC = toTravelPrevB;
////            toTravelPrevB = toTravelPrevA;
////            toTravelPrevA = toTravelD;
////
////
////
////
////            double errorX = toTravelT.getX();
////            double errorY = toTravelT.getY();
////            double errorH = toTravelR.getRadians();
////
////            // Anti-windup: only accumulate integral if error is significant (deadband)
////            if (Math.abs(errorX) > 0.05) {
////                integralX += errorX * loopDelay;
////            } else {
////                integralX = 0;
////            }
////            if (Math.abs(errorY) > 0.05) {
////                integralY += errorY * loopDelay;
////            } else {
////                integralY = 0;
////            }
////            if (Math.abs(errorH) > 0.05) {
////                integralH += errorH * loopDelay;
////            } else {
////                integralH = 0;
////            }
////
////            correctionX = calculatePID(errorX, prevErrorX, integralX, kP_X, kI_X, kD_X, loopDelay);
////            correctionY = calculatePID(errorY, prevErrorY, integralY, kP_Y, kI_Y, kD_Y, loopDelay);
////            correctionH = calculatePID(errorH, prevErrorH, integralH, kP_H, kI_H, kD_H, loopDelay);
////
////            // Normalize X-Y correction if needed
////            double magnitude = Math.hypot(correctionX, correctionY);
////            if (magnitude > 1.0) {
////                correctionX /= magnitude;
////                correctionY /= magnitude;
////            }
////
////            double scale = Math.min(1.0, toTravelD / (thresholdDistance * 2.0));
////            correctionX *= scale;
////            correctionY *= scale;
//
//            // Apply a deadband to small correction outputs
//           // if (Math.abs(correctionX) < 0.5) correctionX = 0;
//            //if (Math.abs(correctionY) < 0.5) correctionY = 0;
//            //if (Math.abs(correctionH) < 0.25) correctionH = 0;
//
//
//            correctionX = Math.signum(correctionX) * Math.max(Math.abs(correctionX), minPower);
//            correctionY = Math.signum(correctionY) * Math.max(Math.abs(correctionY), minPower);
//
//            correctionX = Range.clip(correctionX, -1.0, 1.0);
//            correctionY = Range.clip(correctionY, -1.0, 1.0);
//            correctionH = Range.clip(correctionH, -1.0, 1.0);
//
//            Log.i("FTC18 driveByOtos[loop]", String.format("Drive: X=%.2f, Y=%.2f, H=%.2f", correctionX, correctionY, correctionH));
//           //negative h and it now goes in the right direction
//            driveRobotFC(-correctionY, -correctionX, -correctionH);
//
//           prevErrorX = errorX;
//           prevErrorY = errorY;
//         prevErrorH = errorH;
//
//
//
//            Log.i("FTC18 driveByOtos", String.format("Current Pos - X: %.2f, Y: %.2f, H: %.2f", pos.x, pos.y, pos.h));
//            Log.i("FTC18 driveByOtos", String.format("Target Pos  - X: %.2f, Y: %.2f, H: %.2f", target.getTranslation().getX(), target.getTranslation().getY(), target.getRotation().getDegrees()));
//            Log.i("FTC18 driveByOtos", String.format("Distance to Target: %.2f, Heading Error: %.2f", toTravelD, errorH));
//
//            myOpMode.telemetry.addData("Current Position", "X=%.2f Y=%.2f H=%.2f", pos.x, pos.y, pos.h);
//            myOpMode.telemetry.addData("Target Position", "X=%.2f Y=%.2f H=%.2f", target.getTranslation().getX(), target.getTranslation().getY(), target.getRotation().getDegrees());
//            myOpMode.telemetry.addData("Distance to Target", "%.2f", toTravelD);
//            myOpMode.telemetry.addData("Heading Error", "%.2f", errorH);
//            myOpMode.telemetry.addData("Corrections", "X=%.2f Y=%.2f H=%.2f", correctionX, correctionY, correctionH);
//            myOpMode.telemetry.update();
//
//           // myOpMode.sleep(20);
//
//        }
//        stop();
//
//        myOpMode.sleep(200);
//    }


//    public void driveByOtos(double distanceToTravelX, double distanceToTravelY, double targetHeadingDegrees, double timeout) {
//        runtime.reset();
//        //distanceToTravelX *= 0.32;
//        //distanceToTravelY *= 1.1;
//        // Get initial position from OTOS sensor
//        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//        Translation2d currentT = new Translation2d(pos.x, pos.y);
//        Rotation2d currentR = Rotation2d.fromDegrees(pos.h);
//        Pose2d current = new Pose2d(currentT, currentR);
//
//        // Compute target position
//        Translation2d targetT = currentT.plus(new Translation2d(distanceToTravelX, distanceToTravelY));
//        Rotation2d targetR = Rotation2d.fromDegrees(targetHeadingDegrees);
//        Pose2d target = new Pose2d(targetT, targetR);
//
//        // Use smaller thresholds for a smoother preciser stop
//        double thresholdDistance = 1.0;
//        double angleThreshold = 0.05; //radians should be smaller
//        double correctionX = 0, correctionY = 0, correctionH = 0;
//
//        double integralX = 0, prevErrorX = 0;
//        double integralY = 0, prevErrorY = 0;
//        double integralH = 0, prevErrorH = 0;
//
//        // PID constants (tweak these as needed)
//        //not sure if integeral works
//        double kP_X = 0.25, kI_X = 0, kD_X = 0.02;
//        double kP_Y = 0.25, kI_Y = 0, kD_Y = 0.02;
//        double kP_H = .25, kI_H = 0, kD_H = 0.0;
//
//        // So the robot can overcome friction
//        double minPower = 0.2;
//
//        double lastTime = runtime.seconds();
//        double toTravelPrevA = 0;
//        double toTravelPrevB = 0;
//        double toTravelPrevC = 0;
//
//        while (runtime.seconds() < timeout && myOpMode.opModeIsActive()) {
//            double now = runtime.seconds();
//
//            double loopDelay = (now - lastTime);  // seconds
//            lastTime = now;
//            if (loopDelay < 0.001) {
//                loopDelay = 0.04;  //to prevent it being so small there is a NaN error- maybe not needed now
//            }
//            pos = myOtos.getPosition();
//
//
//            currentT = new Translation2d(pos.x, pos.y);
//            currentR = Rotation2d.fromDegrees(pos.h);
//            current = new Pose2d(currentT, currentR);
//
//            Translation2d toTravelT = target.getTranslation().minus(current.getTranslation());
//            Rotation2d toTravelR = target.getRotation().minus(current.getRotation());
//
//            double toTravelD = current.getTranslation().getDistance(target.getTranslation());
//
//
//            // If within thresholds, exit the loop
//            //if (toTravelAvg < thresholdDistance && toTravelD < thresholdDistance && Math.abs(toTravelR.getRadians()) < angleThreshold) {
//
//            if (Math.abs(toTravelPrevC) < thresholdDistance && Math.abs(toTravelPrevB) < thresholdDistance && Math.abs(toTravelPrevA) < thresholdDistance && Math.abs(toTravelD) < thresholdDistance && Math.abs(toTravelR.getRadians()) < angleThreshold) {
//
//                Log.i("FTC18 driveByOtos", String.format("Leaving loop - X: %.2f, Y: %.2f, H: %.2f", pos.x, pos.y, pos.h));
//                break;
//            }
//
//            toTravelPrevC = toTravelPrevB;
//            toTravelPrevB = toTravelPrevA;
//            toTravelPrevA = toTravelD;
//
//
//
//
//            double errorX = toTravelT.getX();
//            double errorY = toTravelT.getY();
//            double errorH = toTravelR.getRadians();
//
//            // Anti-windup: only accumulate integral if error is significant (deadband)
//            if (Math.abs(errorX) > 0.05) {
//                integralX += errorX * loopDelay;
//            } else {
//                integralX = 0;
//            }
//            if (Math.abs(errorY) > 0.05) {
//                integralY += errorY * loopDelay;
//            } else {
//                integralY = 0;
//            }
//            if (Math.abs(errorH) > 0.05) {
//                integralH += errorH * loopDelay;
//            } else {
//                integralH = 0;
//            }
//
//            correctionX = calculatePID(errorX, prevErrorX, integralX, kP_X, kI_X, kD_X, loopDelay);
//            correctionY = calculatePID(errorY, prevErrorY, integralY, kP_Y, kI_Y, kD_Y, loopDelay);
//            correctionH = calculatePID(errorH, prevErrorH, integralH, kP_H, kI_H, kD_H, loopDelay);
//
//            // Normalize X-Y correction if needed
//            double magnitude = Math.hypot(correctionX, correctionY);
//            if (magnitude > 1.0) {
//                correctionX /= magnitude;
//                correctionY /= magnitude;
//            }
//
//            double scale = Math.min(1.0, toTravelD / (thresholdDistance * 2.0));
//            correctionX *= scale;
//            correctionY *= scale;
//
//            // Apply a deadband to small correction outputs
//            // if (Math.abs(correctionX) < 0.5) correctionX = 0;
//            //if (Math.abs(correctionY) < 0.5) correctionY = 0;
//            //if (Math.abs(correctionH) < 0.25) correctionH = 0;
//
//
//            correctionX = Math.signum(correctionX) * Math.max(Math.abs(correctionX), minPower);
//            correctionY = Math.signum(correctionY) * Math.max(Math.abs(correctionY), minPower);
//
//            correctionX = Range.clip(correctionX, -1.0, 1.0);
//            correctionY = Range.clip(correctionY, -1.0, 1.0);
//            correctionH = Range.clip(correctionH, -1.0, 1.0);
//
//            Log.i("FTC18 driveByOtos[loop]", String.format("Drive: X=%.2f, Y=%.2f, H=%.2f", correctionX, correctionY, correctionH));
//            //negative h and it now goes in the right direction
//            driveRobotFC(-correctionY, -correctionX, -correctionH);
//
//            prevErrorX = errorX;
//            prevErrorY = errorY;
//            prevErrorH = errorH;
//
//
//
//            Log.i("FTC18 driveByOtos", String.format("Current Pos - X: %.2f, Y: %.2f, H: %.2f", pos.x, pos.y, pos.h));
//            Log.i("FTC18 driveByOtos", String.format("Target Pos  - X: %.2f, Y: %.2f, H: %.2f", target.getTranslation().getX(), target.getTranslation().getY(), target.getRotation().getDegrees()));
//            Log.i("FTC18 driveByOtos", String.format("Distance to Target: %.2f, Heading Error: %.2f", toTravelD, errorH));
//
//            myOpMode.telemetry.addData("Current Position", "X=%.2f Y=%.2f H=%.2f", pos.x, pos.y, pos.h);
//            myOpMode.telemetry.addData("Target Position", "X=%.2f Y=%.2f H=%.2f", target.getTranslation().getX(), target.getTranslation().getY(), target.getRotation().getDegrees());
//            myOpMode.telemetry.addData("Distance to Target", "%.2f", toTravelD);
//            myOpMode.telemetry.addData("Heading Error", "%.2f", errorH);
//            myOpMode.telemetry.addData("Corrections", "X=%.2f Y=%.2f H=%.2f", correctionX, correctionY, correctionH);
//            myOpMode.telemetry.update();
//
//            // myOpMode.sleep(20);
//
//        }
//        stop();
//
//        myOpMode.sleep(200);
//    }


    public void driveUntilTouch(double speed) {
        while (!touchSensor.isPressed() && myOpMode.opModeIsActive()) {
            //check neg vs pos vs axial vs lateral speed
            driveRobotFC(0,speed,0);
        }
    }


    public void driveWithOtos() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition(); // Get position

        myOpMode.telemetry.addData("X coordinate", pos.x); // Log the position to the telemetry
        myOpMode.telemetry.addData("Y coordinate", pos.y);
        myOpMode.telemetry.addData("Heading angle", pos.h);

        //myOpMode.telemetry.update();  Update the telemetry on the driver station
    }
    //Goes straight by encoder (takes distance)
    public void straightByEncoder(double speed, double distance, double timeout) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if (myOpMode.opModeIsActive()) {
            DcMotor.RunMode oldMotorMode = leftFrontDrive.getMode();

            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display the data for the driver.
                myOpMode.telemetry.addData("Running to", " lf:%7d lb:%7d rf:%7d rb:%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at", " at lf:%7d lb:%7d rf:%7d rb:%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                //myOpMode.telemetry.update();
            }

            stop();
            setMotorMode(oldMotorMode);
            myOpMode.sleep(500);
        }
    }

    public void strafeByEncoder(double speed, double distance, double timeout) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if (myOpMode.opModeIsActive()) {
            DcMotor.RunMode oldMotorMode = leftFrontDrive.getMode();

            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            // For strafing, left side motors move in opposite direction to right side motors
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH); // Reversed
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH); // Reversed
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display the data for the driver.
                myOpMode.telemetry.addData("Running to", " lf:%7d lb:%7d rf:%7d rb:%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at", " at lf:%7d lb:%7d rf:%7d rb:%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                //myOpMode.telemetry.update();
            }

            stop();
            setMotorMode(oldMotorMode);
            myOpMode.sleep(500);
        }
    }

    public void driveDiagonalByEncoder(double speed, double distance, double timeout) {
        // Break down the diagonal movement into straight and strafe components
        double straightDistance = distance * Math.cos(Math.PI / 4);  // Diagonal is 45 degrees, so cos(45) = 1/sqrt(2)
        double strafeDistance = distance * Math.sin(Math.PI / 4);    // Similarly, sin(45) = 1/sqrt(2)

        // First, move forward/straight by the appropriate amount
        straightByEncoder(speed, straightDistance, timeout);

        // Then, strafe by the appropriate amount
        strafeByEncoder(speed, strafeDistance, timeout);
    }

    //Drives for a set amount of time (takes time)
    public void driveTimed(double axial, double lateral, double yaw, double time) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRobot(axial, lateral, yaw);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            //myOpMode.telemetry.update();
        }
        stop();
    }

    //strafe
    public void strafe(double strafe_power) {
        driveRobot(0.0, strafe_power, 0.0);
    }

    //strafe for a set amount of time (takes time)
    public void strafeTimed(double lateral, double time) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRobot(0, lateral, 0);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            myOpMode.telemetry.update();
        }
        stop();
    }

    //drives robot
    public void driveRobot(double axial, double lateral, double yaw) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        myOpMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        myOpMode.telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }

    //field centric
    public void driveRobotFC(double axial, double lateral, double yaw) {
        double y = axial;
        double x = lateral;
        double rx = yaw;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //radians vs degrees
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;  // Counteract imperfect strafing
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        Log.i("FTC18 driveRobotFC", String.format("fl=%f fr=%f", frontLeftPower, frontRightPower));
        Log.i("FTC18 driveRobotFC", String.format("bl=%f br=%f", backLeftPower, backRightPower));
        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    public void setMotorMode(DcMotor.RunMode motorMode) {
        leftFrontDrive.setMode(motorMode);
        leftBackDrive.setMode(motorMode);
        rightFrontDrive.setMode(motorMode);
        rightBackDrive.setMode(motorMode);
    }

    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public void resetYaw() {
        imu.resetYaw();
    }
//added -180 to headingerror thing (Math.abs(headingError) > HEADING_THRESHOLD)
    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        while (myOpMode.opModeIsActive() && (((Math.abs(headingError - 180)) > HEADING_THRESHOLD))) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, (getRawHeading() - headingOffset)); // Clip the speed to the maximum permitted value.
            myOpMode.telemetry.update();
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            driveRobot(0, 0, turnSpeed);
        }
        stop();
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;
        // Determine the heading current error
        headingError = targetHeading - robotHeading;
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void stop() {
        driveRobot(0, 0, 0);
    }

    public void straight(double power) {
        driveRobot(power, 0, 0);
    }

    public void straightTimed(double power, double time) {
        driveTimed(power, 0, 0, time);
    }


    // Claw stuff
    public void openClaw() {
        clawServo.setPosition(OPEN_SERVO_CLAW);
    }

    public void closeClaw() {
        clawServo.setPosition(CLOSE_SERVO_CLAW);
    }

    // Beak stuff
    public void openBeak() {
        beakServo.setPosition(BEAK_OPEN);
    }

    public void closeBeak() {
        beakServo.setPosition(BEAK_CLOSE);
    }

    public void moveArm(double power) {
        arm.setPower(power);
    }

    public void armAuto(double power, double time) {
        arm.setPower(power);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            myOpMode.telemetry.update();
        }
        arm.setPower(0.0);
    }


    public void moveLeftSlide(double power) {
        leftSlide.setPower(power);
    }

    public void moveRightSlide(double power) {
        rightSlide.setPower(power);
    }

    public void moveSlideTimed(double power, double time) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            myOpMode.telemetry.update();
        }
        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);
    }

    public void driveDiagonalForTime(double forwardPower, double strafePower, double time) {
        driveTimed(forwardPower, strafePower, 0, time);
    }

    public sampleColor getColor() {
        colorSensor.setGain(colorSensorGain);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] > 1 && hsvValues[0] < 75) {
            myOpMode.telemetry.addData("Red", "%.3f", hsvValues[0]);
            return sampleColor.RED;
        } else if (hsvValues[0] > 75 && hsvValues[0] < 130) {
            myOpMode.telemetry.addData("Yellow", "%.3f", hsvValues[0]);
            return sampleColor.YELLOW;
        } else if (hsvValues[0] > 150 && hsvValues[0] < 280) {
            myOpMode.telemetry.addData("Blue", "%.3f", hsvValues[0]);
            return sampleColor.BLUE;

        }
        myOpMode.telemetry.addData("no color found", 0);
        return sampleColor.NONE;
    }

    public int getLeftPosition(){
        return leftSlide.getCurrentPosition();
    }

    public int getRightPosition(){
        return rightSlide.getCurrentPosition();
    }

    public void relativeSlideByEncoder(double speed, double distance, double timeout) {
        // Determine new target position, and pass to motor controller
        slideTimeout = timeout;
        leftSlideTarget = leftSlide.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_SLIDE);
        rightSlideTarget = rightSlide.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_SLIDE);
        leftSlide.setTargetPosition(leftSlideTarget);
        rightSlide.setTargetPosition(rightSlideTarget);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        leftSlide.setPower(Math.abs(speed));
        rightSlide.setPower(Math.abs(speed));
    }

    public void startSlideByEncoder(double speed, double position, double timeout) {
        // Determine new target position, and pass to motor controller
        slideTimeout = timeout;
        slideTarget = (int) (position * COUNTS_PER_INCH_SLIDE);
        leftSlide.setTargetPosition(slideTarget);
        rightSlide.setTargetPosition(slideTarget);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        leftSlide.setPower(Math.abs(speed));
        rightSlide.setPower(Math.abs(speed));
    }

    public void waitForSlide(double speed, double position, double timeout) {
        startSlideByEncoder(speed, position, timeout);
        while (!isSlideDone()){
        }
    }

    public boolean isSlideDone() {
        if ((runtime.seconds() < slideTimeout) && (leftSlide.isBusy() && (rightSlide.isBusy()))) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", " st:%7d ", slideTarget);
            myOpMode.telemetry.addData("Currently at", " at st:%7d", leftSlide.getCurrentPosition());
            myOpMode.telemetry.update();
            Log.i("FTC18 isSlideDone", "is not done");
            myOpMode.sleep(200);
            return false;

        } else {
            stopSlideEncoder();
            Log.i("FTC18 isSlideDone", "is done");
            return true;
        }
    }

    //checks whether the slide is going in the right direction
    public boolean slideBelowZero(){
        if (magneticSensor.isPressed()) {

            if (getSlidePower() < 0.0) {
                stopSlideEncoder();
                resetSlideEncoder();
                //gives drivers a warning about the slide
                myOpMode.telemetry.addData("Warning", "Check Slide");
            }
            return true;
        }else return false;
    }

    public boolean slideWasReset() {
        if (getSlideCurrent() == SlidePosition.ZERO){
            resetSlideEncoder();
            myOpMode.telemetry.addData("Silde", "was reset");
            return true;
        } else return false;
    }

    public void stopSlideEncoder() {
        leftSlide.setPower(0.01);
        rightSlide.setPower(0.01);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void goToZeroPosition(){
        leftSlide.setPower(0.3);
        rightSlide.setPower(0.3);
        if (magneticSensor.isPressed()){
            resetSlideEncoder();
        }
    }

    public boolean isLeftSlideBusy() {
        return leftSlide.isBusy();
    }

    public boolean isRightSlideBusy() {
        return rightSlide.isBusy();
    }

    public boolean isSlideBusy() {
        return isRightSlideBusy() && isLeftSlideBusy();
    }

    public SlidePosition getSlideCurrent() {
        if (magneticSensor.isPressed()) {
            myOpMode.telemetry.addData("LinearSlide", "Is at zero");
            return SlidePosition.ZERO;
        } else myOpMode.telemetry.addData("linearSlide", "Is not at zero");

        return null;
    }

    public void resetSlideEncoder() {

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(magneticSensor.isPressed()) {
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }
        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getSlidePower() {
        return leftSlide.getPower();
    }

    public void startArmByEncoder(double speed, double position, double timeout) {
        // Determine new target position, and pass to motor controller
        armTimeout = timeout;
        armTarget = (int) (position * COUNTS_PER_INCH_SLIDE);
        arm.setTargetPosition(armTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        arm.setPower(Math.abs(speed));


    }

    public void holdArmEncoder() {
        //We may not need the encoder lines- it may be sufficent to just hold the power at a very low value
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armTarget = arm.getCurrentPosition();
       // arm.setTargetPosition(armTarget);
    }

    public void stopArm() {
        arm.setPower( 0.01);
    }

    //auto methods!
    public void scoreSpecimen() {
        //moving the slide
        startSlideByEncoder(.5, 25.6,15);
        waitForSlide(.5,25.6,15);
        driveUntilTouch(.5); //forward or backward?
        startSlideByEncoder(.5,15,15);
        waitForSlide(.5,15,15);
        openClaw();
        startSlideByEncoder(.5, HIGH_POSITION, 15);
    }

    public void pushSampleFar() {
        straightByEncoder(.5, -2.8, 15);
        strafeByEncoder(.5, -21, 15);
    }

    public void pushSampleClose() {
        strafeByEncoder(.5, 23, 15);
        straightByEncoder(.5, 2.2, 15);
        strafeByEncoder(1, -23, 15);
    }



    public double getDistanceFromBar() {
        double distanceFromBar = sensorDistance.getDistance(DistanceUnit.CM);
        Log.i("FTC18", String.format("getDistanceFromBar = %f", distanceFromBar));
        return distanceFromBar;
    }

    public void scoreOnHighBar(){

        startSlideByEncoder(1, 25.3, 30);

        while ((getDistanceFromBar() > ROBOT_AT_BAR) && (myOpMode.opModeIsActive())) {
            straight(.5);

        if (getDistanceFromBar() < SCORING_POSITION){
           stopRobot();
        }
        startSlideByEncoder(1,18,10);
        waitForSlide(.5, 18, 10);
        openClaw();
        straightByEncoder(1,-5,10);
    }}

    public void goToDistance(double distanceToGo) {
        double distanceCurrent = getDistanceFromBar();
        if (distanceToGo < distanceCurrent) {
            while ((distanceToGo < (.5 * distanceCurrent)) && myOpMode.opModeIsActive()) {
                straight(-.5);
                distanceCurrent = getDistanceFromBar();
            }
            Log.i("FTC18", String.format("second while lessthan enter = %f", distanceToGo));
            while ((distanceToGo < (distanceCurrent)) && myOpMode.opModeIsActive()) {
                straight(-.3);
                distanceCurrent = getDistanceFromBar();
            }

            Log.i("FTC18", String.format("second while lessthan leave = %f", distanceToGo));
        }

        else if (distanceToGo > distanceCurrent) {
           while ((distanceToGo > (.8 * distanceCurrent)) && myOpMode.opModeIsActive()) {
                straight(.6);
               distanceCurrent = getDistanceFromBar();

            }
            Log.i("FTC18", String.format("second while greaterthan enter = %f", distanceToGo));

            while ((distanceToGo > (distanceCurrent)) && myOpMode.opModeIsActive()) {
                straight(.4);
                distanceCurrent = getDistanceFromBar();
            }
            Log.i("FTC18", String.format("second while lessthan leave = %f", distanceToGo));
        }
        straight(0);

    }

    public void ram(){

        driveRobotFC(0,1,0); //or pos? or lateral? not yaw
        closeClaw();
    }

    public void stopRobot(){
        leftFrontDrive.setPower(0.01);
        leftBackDrive.setPower(0.01);
        rightFrontDrive.setPower(0.01);
        rightBackDrive.setPower(0.01);
    }
}








package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Hardware2026 {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    //---------------------------------------Drive-Motors-----------------------------------------//
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //-------------------------------------Non-Drive-Mechanisms-----------------------------------//

    //---TURNTABLE---//
    public DcMotor turntableMotor;

    //---LAUNCHER---//
    public DcMotor shooterRight;
    public DcMotor shooterLeft;
    public Servo launcherDoor;

    //---INTAKE--//
    public CRServo spinTakeLeft;
    public CRServo spinTakeRight;

    //---DAISY---//
    public CRServo daisy;
    public AnalogInput positionSensor;
    public NormalizedColorSensor colorSensorLeft;
    public NormalizedColorSensor colorSensorRight;


    //----------------------------------------------IMU-------------------------------------------//
    // Define IMU object and headings (Make it private so it can't be accessed externally)
    public IMU imu = null;

    // Define some other variables for turning
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;

    //-----------------------------------------AprilTags------------------------------------------//
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection lastDetection = null;
    public static final boolean USE_WEBCAM = true;

    //-------------------------------------------Time---------------------------------------------//
    private final ElapsedTime runtime = new ElapsedTime();

    //-------------------------------------Drive-Control------------------------------------------//

   //---CONSTANTS---//
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: our Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 100.0 / 25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

                    //    static final double COUNTS_PER_DEGREE = 39.0/90.0;
    private double turnSpeed = 0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 5.0;
    public int TARGET_TAG_ID = 24;
    private final double BEARING_TOLERANCE = 0.1;


    //--------------------------------Launcher-Door-Constants-------------------------------------//
    public final double LAUNCHER_DOOR_OPEN = 0.5;
    public final double LAUNCHER_DOOR_CLOSED = 0.4;

    //------------------------------------Daisy-Constants-----------------------------------------//
    static final double TOLERANCE = 0.08;  // Increased slightly for one-way reliability

    static final double SERVO_POWER = -0.7; // Speed for one-way travel

    static final double POSITION_ONE = 0.0;
    static final double POSITION_TWO = 1.84;
    static final double POSITION_THREE = 0.88;
    static final double[] POSITIONS = {POSITION_ONE, POSITION_TWO, POSITION_THREE};

    public static final int COLOR_NONE = 0, COLOR_GREEN = 1, COLOR_PURPLE = 2;

    private static final float GREEN_HUE_MIN = 85, GREEN_HUE_MAX = 165;
    private static final float PURPLE_HUE_MIN = 225, PURPLE_HUE_MAX = 365;

    int[] spotColors = {COLOR_NONE, COLOR_NONE, COLOR_NONE};
    boolean[] spotLocked = {false, false, false};
    int[] shootingOrder = {COLOR_NONE, COLOR_NONE, COLOR_NONE};

    int greenCount = 0, purpleCount = 0;
    final int STABLE_NEEDED = 5;

    ElapsedTime daisyTimer = new ElapsedTime();
    boolean daisyIsSpinning = false;
    int daisySpinDuration = 0;

    boolean daisyMoving = false;
    double daisyTarget = 0;

    //-----------------------------------------------OTOS-----------------------------------------//
    SparkFunOTOS myOtos;

    //--------------------------------------------------------------------------------------------//
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware2026(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    //------------------------------------------INIT----------------------------------------------//
    public void init() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");//port2
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");//port0
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");//port1
        turntableMotor = myOpMode.hardwareMap.get(DcMotor.class, "camera_motor");//port2
        shooterRight = myOpMode.hardwareMap.get(DcMotor.class, "shooter_right");//port1
        shooterLeft = myOpMode.hardwareMap.get(DcMotor.class, "shooter_left");//port0
//        spinTakeRight = myOpMode.hardwareMap.get(CRServo.class, "spin_take_right");
//        spinTakeLeft = myOpMode.hardwareMap.get(CRServo.class, "spin_take_left");
        daisy = myOpMode.hardwareMap.get(CRServo.class, "daisy");
        launcherDoor = myOpMode.hardwareMap.get(Servo.class, "launcherDoor");
        positionSensor = myOpMode.hardwareMap.get(AnalogInput.class, "position");
        colorSensorLeft = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "left_color_sensor");
        colorSensorRight = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "right_color_sensor");


        turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntableMotor.setPower(0);

        myOtos = myOpMode.hardwareMap.get(SparkFunOTOS.class, "sensor_otos"); //Otos sensor
        configureOtos();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        turntableMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

//        if (colorSensor instanceof SwitchableLight) ((SwitchableLight) colorSensor).enableLight(true);

        initAprilTag();

        resetHeading();
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(
                        686.02605736, 686.02605736,
                        341.26208637, 208.068001489
                )
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);


        visionPortal = builder.build();




    }

    //---------------------------------------AprilTags--------------------------------------------//
    public AprilTagDetection getLatestDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.size() > 0)
            lastDetection = detections.get(0);
        return lastDetection;
    }

    public boolean checkIfDetected(int tagID) {
        boolean tagFound = false; // This boolean will be true if the specific tag is detected

        List<AprilTagDetection> detections = aprilTag.getDetections();

        // Step through the list of detections and check for the desired tag ID
        for (AprilTagDetection detection : detections) {
            if (detection.id == tagID) {
                // The specific tag was found
                tagFound = true;
                break; // No need to continue the loop once found
            }
        }
        return tagFound;
    }

    public double getTagBearing() {
        AprilTagDetection tag = getLatestDetection();
        if (tag != null) {
            return tag.ftcPose.bearing;
        }
        return Double.NaN;
    }

    public void stopCamera() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();   // stop sending frames to processors
            visionPortal.close();           // releases webcam and processors
        }
    }

    //--------------------------------------OTOS-Methods------------------------------------------//
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

    public void driveWithOtos() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition(); // Get position

        myOpMode.telemetry.addData("X coordinate", pos.x); // Log the position to the telemetry
        myOpMode.telemetry.addData("Y coordinate", pos.y);
        myOpMode.telemetry.addData("Heading angle", pos.h);

        //myOpMode.telemetry.update();  Update the telemetry on the driver station
    }
    //Goes straight by encoder (takes distance)


    //----------------------------------Drive-Methods-(no-OTOS)-----------------------------------//
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

    public void driveVectorByEncoder(double speed, double forwardInches, double strafeInches, double turnDegrees, double timeout) {
        if (!myOpMode.opModeIsActive()) return;

        DcMotor.RunMode oldMotorMode = leftFrontDrive.getMode();

        // Reset encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Convert turn degrees into wheel "inches"
        // TUNE this constant for your robot radius
        final double INCHES_PER_DEGREE = 0.10;
        double turnInches = turnDegrees * INCHES_PER_DEGREE;

        // Convert translational motion to wheel distances
        // Standard mecanum forward/strafe equations
        double lfDist =  forwardInches + strafeInches + turnInches;
        double rfDist =  forwardInches - strafeInches - turnInches;
        double lbDist =  forwardInches - strafeInches + turnInches;
        double rbDist =  forwardInches + strafeInches - turnInches;

        // Convert to encoder counts
        int lfTarget = leftFrontDrive.getCurrentPosition() + (int)(lfDist * COUNTS_PER_INCH);
        int rfTarget = rightFrontDrive.getCurrentPosition() + (int)(rfDist * COUNTS_PER_INCH);
        int lbTarget = leftBackDrive.getCurrentPosition()  + (int)(lbDist * COUNTS_PER_INCH);
        int rbTarget = rightBackDrive.getCurrentPosition() + (int)(rbDist * COUNTS_PER_INCH);

        // Apply target positions
        leftFrontDrive.setTargetPosition(lfTarget);
        rightFrontDrive.setTargetPosition(rfTarget);
        leftBackDrive.setTargetPosition(lbTarget);
        rightBackDrive.setTargetPosition(rbTarget);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Normalize power so no motor exceeds speed
        double max = Math.max(Math.max(Math.abs(lfDist), Math.abs(rfDist)), Math.max(Math.abs(lbDist), Math.abs(rbDist)));
        double scale = 1.0;

        leftFrontDrive.setPower(Math.abs(speed * scale));
        leftBackDrive.setPower(Math.abs(speed * scale));
        rightFrontDrive.setPower(Math.abs(speed * scale));
        rightBackDrive.setPower(Math.abs(speed * scale));

        runtime.reset();
        while (myOpMode.opModeIsActive() &&
                runtime.seconds() < timeout &&
                (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                        leftBackDrive.isBusy()  || rightBackDrive.isBusy())) {

            myOpMode.telemetry.addData("Targets", "LF:%d RF:%d LB:%d RB:%d",
                    lfTarget, rfTarget, lbTarget, rbTarget);
            myOpMode.telemetry.addData("Current", "LF:%d RF:%d LB:%d RB:%d",
                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            // myOpMode.telemetry.update();
        }

        stop();
        setMotorMode(oldMotorMode);
        myOpMode.sleep(300);
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

    public void driveDiagonalForTime(double forwardPower, double strafePower, double time) {
        driveTimed(forwardPower, strafePower, 0, time);
    }
    public void stopRobot(){
        leftFrontDrive.setPower(0.01);
        leftBackDrive.setPower(0.01);
        rightFrontDrive.setPower(0.01);
        rightBackDrive.setPower(0.01);
    }

///-----------------------------------------Decode 2026 -----------------------------------------///

    //------------------------------------Turntable-----------------------------------------------//
    private double filteredBearing = 0.0;
    private static final double MAX_POWER = 0.20;
    private static final double DEADBAND_DEG = 4.0;
    private static final double LPF_ALPHA = 0.6;


    public static final double MOTOR_ENCODER_PPR = 7.0;
    public static final double GEAR_RATIO = 90.0 / 15.0; // 6.0
    public static final double COUNTS_PER_OUTPUT_REV = MOTOR_ENCODER_PPR * GEAR_RATIO; // 42
    public static final double COUNTS_PER_DEGREE = COUNTS_PER_OUTPUT_REV / 360.0; // 0.1166667


    public double lowPassFilter(double previous, double current) {
        return (LPF_ALPHA * previous) + ((1.0 - LPF_ALPHA) * current);
    }
    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    public double getDeltaTime(long lastNs) {
        return (System.nanoTime() - lastNs) / 1e9;
    }


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
    private PID turnPID = new PID(0.006, 0.00005, 0.00045);
    public void applyPID(double error, double dt) {
        double power = turnPID.update(error, dt);
        turnPID.clampIntegral(-100.0, 100.0);

        power = clamp(power, -MAX_POWER, MAX_POWER);
        if (Math.abs(power) < 0.02) power = 0.0;

        turntableMotor.setPower(power);
        myOpMode.telemetry.addData("pidPower", power);
    }


    public AprilTagDetection getTagById(int id) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == id) return d;
        }
        return null;
    }
    public void handleTagTracking(AprilTagDetection tag, double dt) {
        double rawBearing = tag.ftcPose.bearing;
        filteredBearing = lowPassFilter(filteredBearing, rawBearing);
        double error = filteredBearing;

        if (Math.abs(error) <= DEADBAND_DEG) {
            stopTurret();
//            myOpMode.telemetry.addLine("CENTERED ✔");
        } else {
            applyPID(error, dt);
        }

//        myOpMode.telemetry.addData("rawBearing", rawBearing);
//        myOpMode.telemetry.addData("filteredBearing", filteredBearing);
    }


    public void stopTurret() {
        turntableMotor.setPower(0.0);
        turnPID.reset();
    }


    public void setTurntableAngle(double targetDegrees, double maxPower) {

        // Convert degrees to encoder counts
        int targetCounts = (int) Math.round(targetDegrees * .42);

        turntableMotor.setTargetPosition(targetCounts);
        turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Ensure power is always positive for RUN_TO_POSITION
        turntableMotor.setPower(Math.abs(maxPower));
    }


//-------------------------------------------Daisy------------------------------------------------//
    public String colorName(int c) {
    if (c == COLOR_GREEN) return "GREEN";
    if (c == COLOR_PURPLE) return "PURPLE";
    else return "NONE";
}
    public void daisySpin(double daisy_power, int artifacts) {
        daisy.setPower(daisy_power);
        int time = 450 * artifacts;
        myOpMode.sleep(time);
        daisy.setPower(0);
    }

    public void startDaisySpin(double daisy_power, int artifacts) {
        daisySpinDuration = 450 * artifacts; // how long to run
        daisy.setPower(daisy_power);
        daisyTimer.reset();
        daisyIsSpinning = true;
    }

    public void updateDaisySpin() {
        if (daisyIsSpinning) {
            if (daisyTimer.milliseconds() >= daisySpinDuration) {
//                daisy.setPower(0);
                daisyIsSpinning = false;
            }
        }
    }

    public void turntable(double intake_power) {
        turntableMotor.setPower(intake_power);
    }

    public double powerControl() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition(); // Get position

        double yPos = pos.y;
        double power = .8;

        if (yPos>=0 && yPos <=10) {
            power = 1;
        }
        else if (yPos>=30 && yPos <=40) {
            power = .78;
        }
        else if (yPos>=30 && yPos <=50) {
            power = .5;
        }
        else {
            power = .8;
        }
        return power;
    }



    public void moveToVoltage(double target) {
        daisyTarget = target;
        daisyTimer.reset();
        daisyMoving = true;
    }

    public void updateDaisy() {
        if (!daisyMoving) return; // Not moving, do nothing

        double current = positionSensor.getVoltage();

        // Check if we reached target
        if (Math.abs(daisyTarget - current) < TOLERANCE || daisyTimer.seconds() > 3.0) {
            daisy.setPower(0);
            daisyMoving = false;
            return;
        }
    }

    public int getClosestPosition(double voltage) {
        for (int i = 0; i < POSITIONS.length; i++) {
            if (Math.abs(voltage - POSITIONS[i]) < TOLERANCE) return i;
        }
        return -1;
    }

    public void scanCurrentSpot(int index) {
        NormalizedRGBA c = colorSensorLeft.getNormalizedColors();
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

    public String returnCurrentSpot() {
        NormalizedRGBA cLeft = colorSensorLeft.getNormalizedColors();
        float[] hsvLeft = new float[3];
        Color.colorToHSV(cLeft.toColor(), hsvLeft);
        float hueLeft = hsvLeft[0];

        NormalizedRGBA cRight = colorSensorRight.getNormalizedColors();
        float[] hsvRight = new float[3];
        Color.colorToHSV(cRight.toColor(), hsvRight);
        float hueRight = hsvRight[0];

        String color = "";

        if ((hueLeft >= GREEN_HUE_MIN && hueLeft <= GREEN_HUE_MAX)||(hueRight >= GREEN_HUE_MIN && hueRight <= GREEN_HUE_MAX)) {
            color = "GREEN";

        } else if ((hueLeft >= PURPLE_HUE_MIN && hueLeft <= PURPLE_HUE_MAX)||(hueRight >= PURPLE_HUE_MIN && hueRight <= PURPLE_HUE_MAX)) {
            color = "PURPLE";
        }
        return color;
    }
    public void goToNext (){
        double currentVoltage = positionSensor.getVoltage();
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


        // Determine the next position in the sequence (0 -> 1 -> 2 -> 0)
        int nextIndex = (currentPosIndex + 1) % POSITIONS.length;
        int lastIndex = (currentPosIndex - 1) % POSITIONS.length;

        // Execute the movement using your one-way logic
        moveToVoltage(POSITIONS[nextIndex]);
        updateDaisy();

    }
    public void goToLast (){
        daisy.setPower(0);
        double currentVoltage = positionSensor.getVoltage();
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


        // Determine the next position in the sequence (0 -> 1 -> 2 -> 0)
        int lastIndex = (currentPosIndex - 1) % POSITIONS.length;

        // Execute the movement using your one-way logic
        moveToVoltage(POSITIONS[lastIndex]);
        updateDaisy();

    }

    public void waitAndSpin() {
        int nextIndex = (getClosestPosition(positionSensor.getVoltage()) + 1) % POSITIONS.length;
        moveToVoltage(POSITIONS[nextIndex]);
        updateDaisy();
    }

    public void resetSystem() {
        for (int i = 0; i < 3; i++) {
            spotColors[i] = COLOR_NONE;
            spotLocked[i] = false;
        }
        greenCount = 0; purpleCount = 0;
    }

    //--------------------------------------Launcher----------------------------------------------//

    public void shootBall() {
        launcherDoorOpen();
        startLauncherControlled(powerControl());
        myOpMode.sleep(500);
        double currentVoltage = positionSensor.getVoltage();
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
        myOpMode.sleep(500);

        stopLauncher();
        launcherDoorClosed();
        /// close launcher door
    }

    public void shootBallControlled(double power) {
        launcherDoorOpen();
        startLauncherControlled(power);
        myOpMode.sleep(500);
        double currentVoltage = positionSensor.getVoltage();
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
        myOpMode.sleep(500);

        stopLauncher();
        launcherDoorClosed();
        /// close launcher door
    }
    public void shootBalls() {
        launcherDoorOpen();
        startLauncherControlled(powerControl());
        myOpMode.sleep(500);
        double currentVoltage = positionSensor.getVoltage();
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

        double targetVoltage = currentVoltage + 3 * (POSITIONS[1] - POSITIONS[0]);
// assuming positions are evenly spaced, difference between spots

        moveToVoltage(targetVoltage);
        myOpMode.sleep(500);
//
//        // Determine the next position in the sequence (0 -> 1 -> 2 -> 0)
//        int nextIndex = (currentPosIndex + 3) % POSITIONS.length;
//
//        // Execute the movement using your one-way logic
//        moveToVoltage(POSITIONS[nextIndex]);
//        myOpMode.sleep(500);

        stopLauncher();
        launcherDoorClosed();
        /// close launcher door
    }
    public void startLauncher(){
        shooterRight.setPower(1);
        shooterLeft.setPower(1);
    }

    public void startLauncherControlled(double power){
        shooterRight.setPower(power);
        shooterLeft.setPower(power);
    }
    public void stopLauncher(){
        shooterRight.setPower(0);
        shooterLeft.setPower(0);
    }


    //---------------------------------------Launching--------------------------------------------//
    public void updateAprilTagOrder() {
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
        myOpMode.telemetry.update();
    }

    public void shootColor(String color) {
        if (returnCurrentSpot().equals(color)) {
            shootBall();
            return;
        } else {
            launcherDoorClosed();
            waitAndSpin();
            if (returnCurrentSpot().equals(color)) { // PROBLEM
                shootBall();
                return;
            } else {
                launcherDoorClosed();
                waitAndSpin();
                if (returnCurrentSpot().equals(color)) {
                    shootBall();
                } else {
                    launcherDoorClosed();
                    waitAndSpin();
                    if (returnCurrentSpot().equals(color)) {
                        shootBall();
                    }
                }
            }
        }
    }

    public void runAutoLaunch() {
        if (shootingOrder[0] == COLOR_NONE) return;

        for (int targetColor : shootingOrder) {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            daisy.setPower(SERVO_POWER); // always forward

            boolean colorFound = false;

            while (myOpMode.opModeIsActive() && timer.seconds() < 4.0 && !colorFound) { // safety timeout
                double voltage = positionSensor.getVoltage();
                int posIndex = getClosestPosition(voltage);

                // Only consider detection if we're at a valid position
                if (posIndex != -1 && spotLocked[posIndex] == false) {
                    NormalizedRGBA c = colorSensorLeft.getNormalizedColors();
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
                        daisy.setPower(0);

                        // Lock this spot
                        spotColors[posIndex] = targetColor;
                        spotLocked[posIndex] = true;

                        // Move to voltage for this position
                        moveToVoltage(POSITIONS[posIndex]);

                        // Shoot one ball
                        shootBall();

                        // Clear after shot
                        spotColors[posIndex] = COLOR_NONE;
                        spotLocked[posIndex] = false;

                        colorFound = true;
                        break;
                    }
                }

                // Telemetry for debugging
                myOpMode.telemetry.addData("Seeking", colorName(targetColor));
                myOpMode.telemetry.addData("Voltage", "%.2f", voltage);
                myOpMode.telemetry.update();
            }

            daisy.setPower(0); // safety stop if timed out
        }
    }

    public void shootColor(int targetColor) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        daisy.setPower(SERVO_POWER); // always forward

        while (myOpMode.opModeIsActive() && timer.seconds() < 4.0) { // safety timeout
            double voltage = positionSensor.getVoltage();
            int posIndex = getClosestPosition(voltage);

            // Only consider detection if we're at a valid position
            if (posIndex != -1) {
                NormalizedRGBA c = colorSensorLeft.getNormalizedColors();
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
                    daisy.setPower(0);

                    // Lock this spot
                    spotColors[posIndex] = targetColor;
                    spotLocked[posIndex] = true;

                    // Shoot one ball
                    shootBall();

                    // Clear after shot

                    return;
                }
            }

            myOpMode.telemetry.addData("Seeking", colorName(targetColor));
            myOpMode.telemetry.addData("Voltage", "%.2f", voltage);
            myOpMode.telemetry.update();
        }

        daisy.setPower(0); // timeout safety stop
    }

    public void launcherDoorOpen() {
        launcherDoor.setPosition(LAUNCHER_DOOR_OPEN);
    }

    public void launcherDoorClosed() {
        launcherDoor.setPosition(LAUNCHER_DOOR_CLOSED);
    }

    public String patternDecision() {
        String pattern = "";
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            // Look for a specific tag ID
            if (detection.id == 21) {
                pattern = "GPP"; //GPP
                return pattern;
            } else if (detection.id == 22) {
                pattern = "PGP"; //PGP
                return pattern;
            } else if (detection.id == 23) {
                pattern = "PPG"; //PPG
                return pattern;
            } else {
                return pattern;
            }
        }   // end method telemetryAprilTag()
        return pattern;
    }

    public void autoGPP(double power) {
        //just shoot all three
        shootBallControlled(power);
        shootBallControlled(power);
        shootBallControlled(power);
    }

    public void autoPGP(double power) {
        waitAndSpin();
        shootBallControlled(power);
        waitAndSpin();
        shootBallControlled(power);
        waitAndSpin();
        shootBallControlled(power);
        //wait and spin G, shoot P, wait and spin P, shoot G, empty spot spin, shoot P
    }

    public void autoPPG(double power) {
        waitAndSpin();
        shootBallControlled(power);
        shootBallControlled(power);
        shootBallControlled(power);
        //wait and spin G, shoot P and P and G
    }

    //-------------------------------------Telemetry----------------------------------------------//
    public void updateTelemetry(double volt, int pos) {
        myOpMode.telemetry.addData("Voltage", "%.2f", volt);
        myOpMode.telemetry.addData("Position", pos == -1 ? "MOVING" : (pos + 1));
        myOpMode.telemetry.addData("Voltage", "%.2f", volt);
        myOpMode.telemetry.addData("Position", pos == -1 ? "MOVING" : (pos + 1));

        myOpMode.telemetry.addLine("--- Spots ---");
        for (int i=0; i<3; i++) {
            myOpMode.telemetry.addData("Spot " + (i+1), spotLocked[i] ? colorName(spotColors[i]) : "EMPTY");
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 21) {
                myOpMode.telemetry.addData("Pattern", "GPP");
            } else if (detection.id == 22) {
                myOpMode.telemetry.addData("Pattern", "PGP");
            } else if (detection.id == 23) {
                myOpMode.telemetry.addData("Pattern", "PPG");
            }
        }
        driveWithOtos();
        myOpMode.telemetry.addData("Launcher Speed:", shooterLeft.getPower());
        myOpMode.telemetry.addData("Launcher Speed:", shooterRight.getPower());
        myOpMode.telemetry.addData("Launcher Speed:", powerControl());

        myOpMode.telemetry.update();

    }

}




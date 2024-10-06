package org.firstinspires.ftc.teamcode;

// import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class ChickalettaHardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public enum spike {LEFT, CENTER, RIGHT}

    public static final double SPIKE_CENTER_MIN = 100.;
    public static final double SPIKE_CENTER_MAX = 400.;
    public static final double SPIKE_RIGHT_MIN = 410;
    public static final double SPIKE_RIGHT_MAX = 600.;


    public enum PixelPickupState {IDLE_STATE, HAND_MIN_STATE, SHOULDER_PICKUP_STATE, HAND_PICKUP_STATE, SHOULDER_UP_STATE}

    PixelPickupState pixelPickupState = PixelPickupState.IDLE_STATE;
    ElapsedTime pixelPickupTimer;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor spinTake = null;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    private Servo plane = null;
    private Servo planeclamp = null;
    private Servo clamp = null;
    public DigitalChannel limitSwitch;

    private Servo hand = null;


    public DcMotor shoulder = null;

    private IMU imu = null;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;

    // Servo values for chopstick grabber

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: our Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 100.0 / 25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 1.0;
    private double turnSpeed = 0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 5.0;

    public static final int PLANE_LAUNCH = 0;
    public static final int PLANE_CLAMP_LAUNCH = 100;


    //need to play with these
    public static final double CLAMP_CLOSE = 0.25;
    public static final double CLAMP_OPEN = 0.0;
    public static final double HAND_PICKUP = 0.7;
    public static final double HAND_FIX = 0.5;
    public static final double HAND_PLACE = 0;
    public static final int SHOULDER_STORED = 0;
    public static final int SHOULDER_PICKUP1 = 23;
    public static final int SHOULDER_PICKUP2 = 30;
    public static final int SHOULDER_BACKDROP = -500;
    public static final int SHOULDER_UP = 57; //48

    private static final String CUSTOM_TFOD_MODEL_FILE = "/sdcard/model_20240127_162908.tflite";


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public ChickalettaHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init() {
        pixelPickupTimer = new ElapsedTime(); // Initialize pixelPickupTimer
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        shoulder = myOpMode.hardwareMap. get(DcMotor.class, "shoulder");
        spinTake = myOpMode.hardwareMap.get(DcMotor.class, "spin_take");
        clamp = myOpMode.hardwareMap.get(Servo.class, "clamp_servo");
        hand = myOpMode.hardwareMap.get(Servo.class, "hand_servo");
        plane = myOpMode.hardwareMap.get(Servo.class, "plane_servo");
        planeclamp = myOpMode.hardwareMap.get(Servo.class, "planeclamp_servo");
        limitSwitch = myOpMode.hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.FORWARD);

        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initTfod();

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

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

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
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH *.5);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH *.5);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH *.5);
            newRightBackTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH *.5);
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

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", " lf:%7d lb:%7d rf:%7d rb:%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at", " at lf:%7d lb:%7d rf:%7d rb:%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            stop();
            setMotorMode(oldMotorMode);
            myOpMode.sleep(500);
        }
    }

    public void driveTimed(double axial, double lateral, double yaw, double time) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRobot(axial, lateral, yaw);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            myOpMode.telemetry.update();
        }
        stop();
    }

    public void strafe(double strafe_power) {
        driveRobot(0.0, strafe_power, 0.0);
    }

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
        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    public void setShoulder(int shoulder_position) {
        shoulder.setTargetPosition(shoulder_position);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(0.5);
        myOpMode.telemetry.addData("shoulder", "%d", shoulder_position);
    }

    public void runShoulder(double shoulder_speed){
        shoulder.setPower(shoulder_speed);
    }
    public void shoulderWithoutEncoder(){
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shoulderWithEncoder(){
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void makeShoulderSlow(double shoulder_speed_slow) {
        shoulder.setPower(shoulder_speed_slow);
    }

    public void makeShoulderFastAgain(double shoulder_speed_faster) {
        shoulder.setPower(shoulder_speed_faster);
    }

    public int shoulderPosition() {
        return shoulder.getCurrentPosition();
    }

    public void spinTake(double intake_power) {
        spinTake.setPower(intake_power);
        myOpMode.telemetry.addData("spinTake", "%4.2f", spinTake);
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

    public void turnToHeading(double maxTurnSpeed, double heading) {
        resetHeading();
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
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

    public void stop() {
        driveRobot(0, 0, 0);
        //    slide(0);
    }

    public void straight(double power) {
        driveRobot(power, 0, 0);
    }

    public void straightTimed(double power, double time) {
        driveTimed(power, 0, 0, time);
    }

    public void launchPlane() {

        planeclamp.setPosition(PLANE_CLAMP_LAUNCH);
        runtime.reset();
        while (runtime.now(TimeUnit.MILLISECONDS) < 4000)  {

        }
        plane.setPosition(PLANE_LAUNCH);
        myOpMode.telemetry.addData("planeclamp,j6;,", "PLANE_CLAMP_LAUNCH");
        myOpMode.telemetry.addData("plane", "PLANE_LAUNCH");
        runtime.reset();
    }

    //modify servo position hand
    public void setHandPosition(double servo_position_hand) {
        hand.setPosition(servo_position_hand);
        myOpMode.telemetry.addData("hand", "%4.2f", servo_position_hand);
    }

    //modify CLAMP_OPEN
    public void setClampOpen() {
        clamp.setPosition(CLAMP_OPEN);
        myOpMode.telemetry.addData("chopstick", "right");
    }


    //modify CLAMP_CLOSE
    public void setClampClose() {
        clamp.setPosition(CLAMP_CLOSE);
        myOpMode.telemetry.addData("chopstick", "left");
    }

    //modify hand min state
    public void startPixelPickup() {
        pixelPickupTimer.reset();
        setHandPosition(HAND_PICKUP);
        pixelPickupState = PixelPickupState.SHOULDER_UP_STATE;}
    public void advancePixelPickup() {
        switch (pixelPickupState) {
            case IDLE_STATE:
            case SHOULDER_UP_STATE:
                runtime.reset();
                if (runtime.now(TimeUnit.MILLISECONDS) > 1000) {
                    setShoulder(SHOULDER_UP);
                    pixelPickupState = PixelPickupState.HAND_MIN_STATE;}
            case HAND_MIN_STATE:
                runtime.reset();
                if (runtime.now(TimeUnit.MILLISECONDS) > 1000) {
                    setShoulder(SHOULDER_PICKUP1);
                    pixelPickupState = PixelPickupState.SHOULDER_PICKUP_STATE;}
            case SHOULDER_PICKUP_STATE:
                pixelPickupState = PixelPickupState.HAND_PICKUP_STATE;
                setHandPosition(HAND_PICKUP);
            case HAND_PICKUP_STATE:
                if (runtime.now(TimeUnit.MILLISECONDS) > 500) {
                    pixelPickupState = PixelPickupState.IDLE_STATE;}

        }
    }

    public void cancelPixelPickup() {
        pixelPickupState = PixelPickupState.IDLE_STATE;
    }

    public void releasePixel(double time, double speed) {
        spinTake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            spinTake.setPower(speed);
        }
        spinTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    public void initTFOD(String file_name) {
        tfod = new TfodProcessor.Builder()
                .setModelFileName(file_name)
                .build();

    }
    public DemoBotHardware.spike spikeSenseAuto() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        do {
            currentRecognitions = tfod.getRecognitions();
            myOpMode.telemetry.addData("# Objects Detected %d", currentRecognitions.size());
            myOpMode.telemetry.update();

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                //telemetry
                if (SPIKE_CENTER_MIN < x && x < SPIKE_CENTER_MAX) {
                    return DemoBotHardware.spike.CENTER;
                } else if (SPIKE_RIGHT_MIN < x && x < SPIKE_RIGHT_MAX) {
                    return DemoBotHardware.spike.RIGHT;
                } else {
                    return DemoBotHardware.spike.LEFT;
                }
            }
        } while (runtime.seconds() <= 10);
        return DemoBotHardware.spike.LEFT;
    }
}


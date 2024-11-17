package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Hardware2025;

@TeleOp(name = "Beak2025", group = "LinearOpmode")
//@Disabled


public class Beak2025 extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);

    public Servo beakArm;
    public Servo beakServo;
    private static final double BEAK_ARM_DOWN = 0;
    private static final double BEAK_ARM_UP = 1;
    private static final double BEAK_OPEN = .7;
    private static final double BEAK_CLOSE = .5;

    //@Override
    public void runOpMode() {
        robot.init();
        // wait for the start button to be pressed.
        waitForStart();

        Hardware2025 robot = new Hardware2025(this);

//    private static final double BEAK_ARM_DOWN = 0;
//    private static final double BEAK_ARM_UP = 1;

        // while the OpMode is active, loop and read whether the sensor is being pressed.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
//
//            if (gamepad2.dpad_right){
//                arm.setPosition(BEAK_ARM_DOWN);
//
//            }
//            if (gamepad2.dpad_left){
//                arm.setPosition(BEAK_ARM_UP);
//            }
            if (gamepad2.left_bumper) {
                robot.closeBeak();
                telemetry.addData("GamepadLBumper", "Is Pressed");
            }
            else {
                robot.openBeak();
            }

        }
    }

}


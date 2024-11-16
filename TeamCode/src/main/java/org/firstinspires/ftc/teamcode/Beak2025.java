package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Beak2025", group = "LinearOpmode")
//@Disabled


public class Beak2025 extends LinearOpMode {

    public Servo beakArmServo;
    public Servo beakServo;
    private static final double BEAK_ARM_DOWN = 0;
    private static final double BEAK_ARM_UP = 1;
    private static final double BEAK_SPIN = 1;

    @Override
    public void runOpMode() {

        beakArmServo = hardwareMap.get(Servo.class, "servo_beak_arm");
        beakServo = hardwareMap.get(Servo.class, "servo_beak");

        // wait for the start button to be pressed.
        waitForStart();

        // while the OpMode is active, loop and read whether the sensor is being pressed.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            if (gamepad2.left_stick_button){
                beakArmServo.setPosition(BEAK_ARM_DOWN);

            }
            if (gamepad2.right_stick_button){
                beakArmServo.setPosition(BEAK_ARM_UP);
            }

            if (gamepad2.left_bumper)
                beakServo.setPosition(BEAK_SPIN);
        }

    }








    }

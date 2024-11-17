/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains Teleop
 */

@TeleOp(name = "teleop2025", group = "Linear Opmode")

public class teleop2025 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    Hardware2025 robot = new Hardware2025(this);
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init();

        //robot.startSlide();
//
//        robot.moveSlide(0.3);
//        if (robot.magneticSensorWall.isPressed()){
//            robot.moveSlide(0.0);
//        }else {
//            robot.moveSlide(0.3);
//        }
//

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            // This button choice was made so that it is hard to hit on accident
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                robot.resetYaw();
            }

            robot.getColor();
            robot.getSlideCurrent();
            telemetry.update();

            //SlowScales input
            double gp1LY = gamepad1.left_stick_y;
            double gp1LX = gamepad1.left_stick_x;
            double gp1RX = gamepad1.right_stick_x;

            //slowscale 1
            if (gamepad1.right_bumper) {
                double slowscale = .33;
                gp1LY *= slowscale;
                gp1LX *= slowscale;
                gp1RX *= slowscale;
            }

            //slowscale 2
            if (gamepad1.left_bumper) {
                double slowscale = .1;
                gp1LY *= slowscale;
                gp1LX *= slowscale;
                gp1RX *= slowscale;
            }
            robot.driveRobotFC(-gp1LY, gp1LX, gp1RX);

            //go to the pickup height
            if (gamepad2.a) {
                robot.slideByEncoder(.5, 5, 10);
            }

            //go to bar 1 height
            if (gamepad2.b) {
                robot.slideByEncoder(.5, 14, 10);
                robot.straightByEncoder(.3, 1, 10);
                robot.slideByEncoder(.5, 8.5, 10);
            }

            if (gamepad2.x) {
                robot.slideByEncoder(.5, 27, 15);
                robot.straightByEncoder(.3, 1, 5);
                robot.slideByEncoder(.5, 21.5, 10);
            }

            //open and close claw via touch sensor
            if (gamepad2.right_bumper) {
                robot.closeClaw();
                telemetry.addData("GamepadRBumper", "Is Pressed");

            }

            if (gamepad2.left_bumper) {
                robot.openClaw();
                telemetry.addData("GamepaLRBumper", "Is Pressed");
            }

/*            else {
                telemetry.addData("GamepadRBumper", "Is Not Pressed");
                if (gamepad2.left_bumper) {
                    robot.closeClaw();
                    telemetry.addData("Touch Sensor", "Is Pressed");
                } else {
                    robot.openClaw();
                    telemetry.addData("Touch Sensor", "Is Not Pressed");
                }
            }


            /** ?if (gamepad2.a){
                robot.setSlideTargetPosition(Hardware2025.SlidePosition.START);
            }
            if (gamepad2.b){
                robot.setSlideTargetPosition(Hardware2025.SlidePosition.WALL);
            }
            if (gamepad2.x){
                robot.setSlideTargetPosition(Hardware2025.SlidePosition.LOW);
            }
            if (gamepad2.y){
                robot.setSlideTargetPosition(Hardware2025.SlidePosition.HIGH);
            }

                        if (gamepad2.dpad_up) {
                robot.moveSlide(1);
            }

            if (gamepad2.dpad_down) {
                robot.moveSlide(-1);
            }

             private void driveRobotFC(double v, double gp1LX, double gp1RX)}
             private void resetYaw(){}

             if (gamepad2.dpad_up) {
             robot.moveSlide(1);
             }

             if (gamepad2.dpad_down) {
             robot.moveSlide(-1);
             }
        **/

        }
    }
}


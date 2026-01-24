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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * This file contains Teleop
 */

@TeleOp(name = "REDTeleop2026", group = "Linear Opmode")

public class REDTeleop2026 extends LinearOpMode {

    protected int allianceID = 24;

    // Declare OpMode members for each of the 4 motors.
    Hardware2026 robot = new Hardware2026(this);
    private final ElapsedTime runtime = new ElapsedTime();

    // Robot class
    public void runOpMode() {
        robot.init();
        robot.launcherDoorClosed();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        robot.updateAprilTagOrder();
        long lastNs = System.nanoTime();

        waitForStart();
        runtime.reset();
        robot.moveToVoltage(0);

        while (opModeIsActive()) {
            String pattern = robot.patternDecision();

            //robot.driveWithOtos();
//            telemetry.update();
//            telemetry.addData("Turret Encoder", robot.turntableMotor.getCurrentPosition());
            double ticksFromZero = robot.turntableMotor.getCurrentPosition() / .43;
//            telemetry.addData("Angle", ticksFromZero);

            //GAMEPAD 1: Field-centric driving, slowscale, etc.
            if (gamepad1.options) {
                robot.resetYaw();
            }

            double gp1LY = gamepad1.left_stick_y;
            double gp1LX = gamepad1.left_stick_x;
            double gp1RX = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                double slowScale = .25;
                gp1LY *= slowScale;
                gp1LX *= slowScale;
                gp1RX *= slowScale;
            }

            if (gamepad1.left_bumper) {
                double slowScale = .33;
                gp1LY *= slowScale;
                gp1LX *= slowScale;
                gp1RX *= slowScale;
            }

            robot.driveRobotFC(-gp1LY, gp1LX, gp1RX);

            //GAMEPAD 2

            double dt = robot.getDeltaTime(lastNs);
            lastNs = System.nanoTime();

            AprilTagDetection tag = robot.getTagById(allianceID);

            if (tag != null && tag.ftcPose != null) {
                robot.handleTagTracking(tag, dt);
            } else {
//                robot.stopTurret();
//                telemetry.addLine("Tag 24 NOT FOUND");
                double turntable = -gamepad2.left_stick_x;
                robot.turntableMotor.setPower(turntable);
            }

            // Shooter and sorter stuff
            if (gamepad2.start) robot.resetSystem();

            double currentVoltage = robot.positionSensor.getVoltage();
            int currentPosIndex = robot.getClosestPosition(currentVoltage);

            robot.updateAprilTagOrder();

            if (currentPosIndex != -1 && !robot.spotLocked[currentPosIndex]) {
                robot.scanCurrentSpot(currentPosIndex);
            }
            if (gamepad2.back) robot.shootBalls();
            if (gamepad2.a) {
                robot.launcherDoorOpen();
                robot.shootColor(robot.COLOR_GREEN);

                robot.launcherDoorClosed();
            }
            if (gamepad2.x) {
                robot.launcherDoorOpen();
                robot.shootColor(2);
                robot.launcherDoorClosed();
            }
            if (gamepad2.y) {
               robot.goToNext();
            }
            if (gamepad2.b) {
                robot.turntableMotor.setPower(0);
                robot.shootBall();
            }
            if (gamepad2.right_bumper) robot.runAutoLaunch();

            if (gamepad2.left_bumper) robot.launcherDoorOpen();
            else robot.launcherDoorClosed();

            if (gamepad2.dpad_up) {
                robot.startLauncherControlled(1);
            } else{
                robot.stopLauncher();
            }

            robot.updateTelemetry(currentVoltage, currentPosIndex);

        }
//        robot.stopCamera();
    }
}
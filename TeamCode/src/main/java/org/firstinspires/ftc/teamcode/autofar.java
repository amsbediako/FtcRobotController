package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autofar")
//@Disabled
public class autofar extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);

    @Override
    public void runOpMode() {
        robot.init();

        robot.clawServo.setPosition(.2);

        waitForStart();
        telemetry.update();
        robot.closeClaw();

        robot.straightByEncoder(.5, 10, 15);
        robot.strafeByEncoder(.5, 10, 10);

        //moving the slide
        robot.startSlideByEncoder(.5, robot.HIGH_POSITION, 15);
        while (!robot.isSlideDone()){
        }
        robot.strafeByEncoder(.5, 3.1, 15);
        robot.relativeSlideByEncoder(.5, -4, 5);
        while(!robot.isSlideDone()) {
        }
        robot.clawServo.setPosition(0.7);
        robot.startSlideByEncoder(.5, robot.WALL_POSITION, 15);
        while (!robot.isSlideDone()){
        }

        //push sample
        robot.strafeByEncoder(.5, -3, 10);
        robot.straightByEncoder(.5, -13, 15); //test
        robot.strafeByEncoder(.5, 12, 15); //test
        robot.straightByEncoder(-.5, -2.25, 15); //test
        robot.strafeByEncoder(.5, -22, 15); //test

        }
    }

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

        waitForStart();

        robot.driveDiagonalForTime(.4, .5, 1.2);
        robot.startSlideByEncoder(.5, robot.HIGH_POSITION, 10);
        //robot.wait(1);
        robot.relativeSlideByEncoder(.8, -6, 10);
        robot.openClaw();

        robot.straightByEncoder(-.5, 36, 15);
        robot.strafeTimed(1,1.5); //test
        robot.straightByEncoder(-.5, 12, 15);
        robot.strafeTimed(-1, 3.5); //test

        }
    }

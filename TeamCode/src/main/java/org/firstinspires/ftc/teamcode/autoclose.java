package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autoclose")
//@Disabled
public class autoclose extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);

    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();

        robot.closeBeak();
        robot.driveDiagonalForTime(-.4, .5, 1.2);
        robot.startSlideByEncoder(.5, robot.HIGH_POSITION, 10);
        robot.relativeSlideByEncoder(.8, -6, 10);
        robot.openClaw();
        robot.driveDiagonalForTime(.5, -.5, 1.2);

    }
}
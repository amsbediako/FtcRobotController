package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "autoclose")
//@Disabled
public class autoclose extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);
    private final ElapsedTime runtime = new ElapsedTime();

    //@Override
    public void runOpMode() {
        robot.init();

        robot.clawServo.setPosition(.2);

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        robot.closeClaw();


//        robot.driveDiagonalForTime(-.4, .5, 1);
//        robot.driveDiagonalByEncoder(0.4, 0.5, 6.4, 1);

        robot.straightByEncoder(.5, -10, 15);
        robot.strafeByEncoder(.5, 10, 10);

        robot.startSlideByEncoder(.5, robot.HIGH_POSITION, 15);
        while (!robot.isSlideDone()){
        }
        robot.strafeByEncoder(.5, 3.24, 15);
        runtime.reset();
        robot.relativeSlideByEncoder(1, -4, 5);
        while(!robot.isSlideDone()) {
        }
        robot.clawServo.setPosition(0.7);
        robot.startSlideByEncoder(.5, robot.WALL_POSITION, 15);
        while (!robot.isSlideDone()){
        }
//
//        robot.strafeByEncoder(.5, -3.2, 15);
//        robot.straightByEncoder(.5, 13, 15);
//        robot.strafeByEncoder(.5, 15, 15);
//        robot.startSlideByEncoder(.5, robot.LOW_POSITION, 10);
//        while (!robot.isSlideDone()){
//        }
//        robot.straightByEncoder(.5, -4, 15);
//
////        robot.driveDiagonalForTime(.5, -.6, 1.2);
//        robot.startSlideByEncoder(.5, robot.LOW_POSITION, 10);
//        while (!robot.isSlideDone()){
//        }

    }
}

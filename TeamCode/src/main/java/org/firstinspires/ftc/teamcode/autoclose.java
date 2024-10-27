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

        robot.strafeTimed(-1, 1.5);
        robot.strafeTimed(1, 1.5);

        robot.straightTimed(1, 4.1);
        robot.strafeTimed(-1, 1.1);
        robot.straightTimed(-1, 4.1);
        robot.strafeTimed(-1, 0.5);
        robot.strafeTimed(1, 8);


    }
}


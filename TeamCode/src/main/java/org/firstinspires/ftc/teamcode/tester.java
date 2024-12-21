package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "tester")
//@Disabled
public class tester extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);
    private final ElapsedTime runtime = new ElapsedTime();

    //@Override
    public void runOpMode() {
        robot.init();
        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        robot.closeClaw();

        robot.strafeByEncoder(.5, 1, 15);
        sleep(1000);

        robot.strafeByEncoder(.5, 2, 15);
        sleep(1000);

        robot.strafeByEncoder(.5, 4, 15);
        sleep(1000);

    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autofarv1")
public class autofarv1 extends LinearOpMode {
    Hardware2025 robot = new  Hardware2025(this);

        @Override
        public void runOpMode() {
            robot.init();

            robot.strafeTimed(-1, 7);
            robot.strafeTimed(1, 1.5);
            robot.straightTimed(1,4);
            robot.strafeTimed(-1,1.5);
            robot.straightTimed(1, 4);
            robot.strafeTimed(1,7.5);
        }
    }



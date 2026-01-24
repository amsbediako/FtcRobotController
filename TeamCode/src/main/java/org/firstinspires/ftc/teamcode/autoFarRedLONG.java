package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autoFarRedLONG")
//@Disabled
public class autoFarRedLONG extends LinearOpMode {

    Hardware2026 robot = new Hardware2026(this);

    public void runOpMode() {
        robot.init();
        robot.resetYaw();

        waitForStart();
        telemetry.update();
        robot.resetYaw();

        robot.driveVectorByEncoder(1, 31, 0, 48, 30);

        robot.setTurntableAngle(60, 1);
        robot.turntableMotor.setPower(0);
        if (robot.patternDecision().equals("GPP")) {
            robot.autoGPP(.3);
            telemetry.addLine("Pattern: GPP");
        } else if (robot.patternDecision().equals("PGP")) {
            robot.autoPGP(.4);
            telemetry.addLine("Pattern: PGP");
        } else if (robot.patternDecision().equals("PPG")) {
            robot.autoPPG(.4);
            telemetry.addLine("Pattern: PPG");
        } else if (robot.patternDecision().equals("")){
            robot.shootBalls();
            telemetry.addLine("Pattern: NONE");
        }


//        robot.shootArtifact(1);
//        sleep(500);
//        robot.daisySpin(1 ,5);
//        sleep(500);
//        robot.shootArtifact(0);

        robot.driveVectorByEncoder(1, -4, 0, 42, 30);

    }

}
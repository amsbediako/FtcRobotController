

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TwinSlideTest", group="Linear OpMode")

public class TwinSlideTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    private static final double SLIDE_POWER = 0.1;
    private static final double SLIDE_POWER_DOWN = -0.1
             ;


    @Override
    public void runOpMode() {

        leftSlide = hardwareMap.get(DcMotor.class, "Left_Slide_Motor");
        rightSlide = hardwareMap.get(DcMotor.class, "Right_Slide_Motor");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

           if (gamepad2.dpad_up){
               moveSlidesUp();
            }

            if (gamepad2.dpad_down){
                moveSlidesdown();
            }
            if (gamepad2.start){
                stopSlide();
            }
        }
    }
    public void moveSlidesUp(){
        leftSlide.setPower(SLIDE_POWER);
        rightSlide.setPower(SLIDE_POWER);
    }
    public void moveSlidesdown(){
        leftSlide.setPower(SLIDE_POWER_DOWN);
        rightSlide.setPower(SLIDE_POWER_DOWN);
    }

    public void stopSlide(){
        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);
    }
}

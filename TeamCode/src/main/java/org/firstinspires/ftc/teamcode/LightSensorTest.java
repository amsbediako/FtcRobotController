package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="REV Color Sensor Light Detection", group="Linear Opmode")

public class LightSensorTest extends LinearOpMode {

    private ColorSensor colorSensor; // Declare the color sensor
    private ElapsedTime runtime = new ElapsedTime(); // Timer to control runtime

    @Override
    public void runOpMode() {
        // Initialize the color sensor
        // "color_sensor" is the device name in the configuration
        colorSensor = hardwareMap.get(ColorSensor.class, "light_sensor");
        final double HAS_SPECIMAN = 5.0;
        // Wait for the start button to be pressed
        waitForStart();

        // Run until the end of the match
        while (opModeIsActive()) {

            // Get the ambient light level (this is the reflection of light that the sensor detects)
            int lightLevel = colorSensor.alpha(); // alpha() measures ambient light level

            // Display the light level on the driver station
            telemetry.addData("Ambient Light Level (Alpha):", lightLevel);
            telemetry.update();

            if (lightLevel < HAS_SPECIMAN ){
                //close claw

                telemetry.addData("Speciman", "is in claw");
            }

            // Optionally add more logic to react based on light level:
            // e.g., turn on a motor or control an actuator when a certain light threshold is met.

            // Sleep to prevent wasting resources


        }
    }
}
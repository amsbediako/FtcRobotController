/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/*
 * This OpMode demonstrates how to use a REV Robotics Touch Sensor, REV Robotics Magnetic Limit Switch, or other device
 * that implements the TouchSensor interface. Any touch sensor that connects its output to ground when pressed
 * (known as "active low") can be configured as a "REV Touch Sensor". This includes REV's Magnetic Limit Switch.
 *
 * The OpMode assumes that the touch sensor is configured with a name of "sensor_touch".
 *
 * A REV Robotics Touch Sensor must be configured on digital port number 1, 3, 5, or 7.
 * A Magnetic Limit Switch can be configured on any digital port.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Where and What", group = "Sensor")
public class WhereAndWhatIsArtifact extends LinearOpMode {

    TouchSensor positionOne;
     TouchSensor positionTwo;
     TouchSensor positionThree;

    NormalizedColorSensor colorSensor;

    public static final int COLOR_NONE = 0;
    public static final int COLOR_GREEN = 1;
    public static final int COLOR_PURPLE = 2;

    // HSV hue ranges (0 - 360)
    private static final float GREEN_HUE_MIN = 85;
    private static final float GREEN_HUE_MAX = 165;
    private static final float PURPLE_HUE_MIN = 225;
    private static final float PURPLE_HUE_MAX = 365;

    @Override
    public void runOpMode() {

        positionOne = hardwareMap.get(TouchSensor.class, "mag_sensor_one");
        positionTwo = hardwareMap.get(TouchSensor.class, "mag_sensor_two");
        positionThree = hardwareMap.get(TouchSensor.class, "mag_sensor_three");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        int spotOneColor = COLOR_NONE;
        int spotTwoColor = COLOR_NONE;
        int spotThreeColor = COLOR_NONE;
        boolean spotOneLocked = false;
        boolean spotTwoLocked = false;
        boolean spotThreeLocked = false;
        int[] spotColor = {COLOR_NONE, COLOR_NONE, COLOR_NONE};
        final int stableReadingsRequired = 1;

        final int stableNeeded = 5;
        int greenCount = 0;
        int purpleCount = 0;

        float[] hsv = new float[3];

        while (opModeIsActive()) {

            // RESET if X pressed
            if (gamepad1.x) {
                spotOneColor = COLOR_NONE;
                spotOneLocked = false;
                spotTwoColor = COLOR_NONE;
                spotTwoLocked = false;
                spotThreeColor = COLOR_NONE;
                spotThreeLocked = false;
                greenCount = 0;
                purpleCount = 0;
            }

            // If SPOT 1 is not locked, keep scanning colors
            if (!spotOneLocked) {

                NormalizedRGBA c = colorSensor.getNormalizedColors();
                Color.colorToHSV(c.toColor(), hsv);
                float hue = hsv[0];

                int detectedColor = COLOR_NONE;

                // GREEN detection
                if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
                    greenCount++;
                    purpleCount = 0;

                    if (greenCount >= stableNeeded) {
                        detectedColor = COLOR_GREEN;
                    }

                    // PURPLE detection
                } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
                    purpleCount++;
                    greenCount = 0;

                    if (purpleCount >= stableNeeded) {
                        detectedColor = COLOR_PURPLE;
                    }

                } else {
                    greenCount = purpleCount = 0;
                    detectedColor = COLOR_NONE;
                }

                // If touch sensor is pressed AND real color detected → LOCK IT
                if (positionOne.isPressed() &&
                        (detectedColor == COLOR_GREEN || detectedColor == COLOR_PURPLE)) {

                    spotOneColor = detectedColor;
                    spotOneLocked = true;
                }
            }

            if (!spotTwoLocked) {

                NormalizedRGBA c = colorSensor.getNormalizedColors();
                Color.colorToHSV(c.toColor(), hsv);
                float hue = hsv[0];

                int detectedColor = COLOR_NONE;

                // GREEN detection
                if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
                    greenCount++;
                    purpleCount = 0;

                    if (greenCount >= stableNeeded) {
                        detectedColor = COLOR_GREEN;
                    }

                    // PURPLE detection
                } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
                    purpleCount++;
                    greenCount = 0;

                    if (purpleCount >= stableNeeded) {
                        detectedColor = COLOR_PURPLE;
                    }

                } else {
                    greenCount = purpleCount = 0;
                    detectedColor = COLOR_NONE;
                }

                // If touch sensor is pressed AND real color detected → LOCK IT
                if (positionTwo.isPressed() &&
                        (detectedColor == COLOR_GREEN || detectedColor == COLOR_PURPLE)) {

                    spotTwoColor = detectedColor;
                    spotTwoLocked = true;
                }
            }

            if (!spotThreeLocked) {

                NormalizedRGBA c = colorSensor.getNormalizedColors();
                Color.colorToHSV(c.toColor(), hsv);
                float hue = hsv[0];

                int detectedColor = COLOR_NONE;

                // GREEN detection
                if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
                    greenCount++;
                    purpleCount = 0;

                    if (greenCount >= stableNeeded) {
                        detectedColor = COLOR_GREEN;
                    }

                    // PURPLE detection
                } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
                    purpleCount++;
                    greenCount = 0;

                    if (purpleCount >= stableNeeded) {
                        detectedColor = COLOR_PURPLE;
                    }

                } else {
                    greenCount = purpleCount = 0;
                    detectedColor = COLOR_NONE;
                }

                // If touch sensor is pressed AND real color detected → LOCK IT
                if (positionThree.isPressed() &&
                        (detectedColor == COLOR_GREEN || detectedColor == COLOR_PURPLE)) {

                    spotThreeColor = detectedColor;
                    spotThreeLocked = true;
                }
            }


            // TELEMETRY
            telemetry.addData("Spot 1 Locked", spotOneLocked);
            telemetry.addData("Spot 1 Color", colorName(spotOneColor));
            telemetry.addData("Spot 2 Locked", spotTwoLocked);
            telemetry.addData("Spot 2 Color", colorName(spotTwoColor));
            telemetry.addData("Spot 3 Locked", spotThreeLocked);
            telemetry.addData("Spot 3 Color", colorName(spotThreeColor));
            telemetry.addData("Press X to reset", "");
            telemetry.update();
        }
    }

    private String colorName(int c) {
        switch (c) {
            case COLOR_GREEN:
                return "GREEN";
            case COLOR_PURPLE:
                return "PURPLE";
            default:
                return "NONE";
        }
    }


}
/* Copyright (c) 2017-2020 FIRST. All rights reserved.
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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



public class SensorColorTestsEdited extends LinearOpMode {


  NormalizedColorSensor colorSensor;


  View relativeLayout;



  @Override public void runOpMode() {


    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    try {
      runSample();
    } finally {

      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.WHITE);
        }
      });
      }
  }

  protected void runSample() {

    float gain = 20;


    final float[] hsvValues = new float[3];


    boolean yButtonPreviouslyPressed = false;
    boolean yButtonCurrentlyPressed = false;

    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");


    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)colorSensor).enableLight(true);
    }


    waitForStart();


    while (opModeIsActive()) {

      telemetry.addData("Gain", gain);


      colorSensor.setGain(gain);


      yButtonCurrentlyPressed = gamepad1.y;


      if (yButtonCurrentlyPressed != yButtonPreviouslyPressed) {
        if (yButtonCurrentlyPressed) {
          if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)colorSensor;
            light.enableLight(!light.isLightOn());
          }
        }
      }
      yButtonPreviouslyPressed = yButtonCurrentlyPressed;


      NormalizedRGBA colors = colorSensor.getNormalizedColors();


      Color.colorToHSV(colors.toColor(), hsvValues);

      telemetry.addLine()
              .addData("Red", "%.3f", colors.red)
              .addData("Green", "%.3f", colors.green)
              .addData("Blue", "%.3f", colors.blue);
      telemetry.addLine()
              .addData("Hue", "%.3f", hsvValues[0])
              .addData("Saturation", "%.3f", hsvValues[1])
              .addData("Value", "%.3f", hsvValues[2]);
      telemetry.addData("Alpha", "%.3f", colors.alpha);




      if (colorSensor instanceof DistanceSensor) {
        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
      }


       if (hsvValues[0] > 21 && hsvValues[0] <28 ) {
       telemetry.addData("Red", "%.3f", hsvValues[0]);
       }
       else if (hsvValues[0] > 79 && hsvValues[0] < 86) {
       telemetry.addData("Yellow", "%.3f", hsvValues[0]);
       }
       else if (hsvValues[0] > 215 && hsvValues[0] < 225) {
       telemetry.addData("Blue", "%.3f", hsvValues[0]);
       }
       else {
         telemetry.addData("no color found", 0);
       }


      telemetry.update();



    }
  }
}

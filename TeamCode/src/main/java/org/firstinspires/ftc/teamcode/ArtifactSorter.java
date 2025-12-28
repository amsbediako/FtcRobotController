package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@Autonomous(name = "ArtifactSorterA - Continuous 3 Sensor Scan", group = "Sensor")
public class ArtifactSorter extends LinearOpMode {

  // ---- color constants ----
  public static final int COLOR_NONE   = 0;
  public static final int COLOR_GREEN  = 1;
  public static final int COLOR_PURPLE = 2;

  // HSV hue ranges (0 - 360)
  private static final float GREEN_HUE_MIN  = 75;
  private static final float GREEN_HUE_MAX  = 170;
  private static final float PURPLE_HUE_MIN = 225;
  private static final float PURPLE_HUE_MAX = 365;

  @Override
  public void runOpMode() {

    // three sensors
    NormalizedColorSensor spot1 = hardwareMap.get(NormalizedColorSensor.class, "spot1");
    NormalizedColorSensor spot2 = hardwareMap.get(NormalizedColorSensor.class, "spot2");
    NormalizedColorSensor spot3 = hardwareMap.get(NormalizedColorSensor.class, "spot3");

    // enable lights if available
    if (spot1 instanceof SwitchableLight) ((SwitchableLight) spot1).enableLight(true);
    if (spot2 instanceof SwitchableLight) ((SwitchableLight) spot2).enableLight(true);
    if (spot3 instanceof SwitchableLight) ((SwitchableLight) spot3).enableLight(true);

    telemetry.addLine("ArtifactSorter ready for 3 sensors");
    telemetry.addLine("Scanning continuously for GREEN or PURPLE");
    telemetry.addLine("Waiting for start...");
    telemetry.update();

    waitForStart();

    final int stableReadingsRequired = 5;
    int[] greenCounts  = new int[3];
    int[] purpleCounts = new int[3];
    int[] lastDetectedColors = new int[]{COLOR_NONE, COLOR_NONE, COLOR_NONE};

    final float[] hsvValues = new float[3];

    NormalizedColorSensor[] sensors = {spot1, spot2, spot3};

    while (opModeIsActive()) {

      for (int i = 0; i < sensors.length; i++) {

        NormalizedRGBA colors = sensors[i].getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];

        // detect green
        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
          greenCounts[i]++;
          purpleCounts[i] = 0;
          if (greenCounts[i] >= stableReadingsRequired) {
            lastDetectedColors[i] = COLOR_GREEN;
          }

          // detect purple
        } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
          purpleCounts[i]++;
          greenCounts[i] = 0;
          if (purpleCounts[i] >= stableReadingsRequired) {
            lastDetectedColors[i] = COLOR_PURPLE;
          }

        } else {
          greenCounts[i] = 0;
          purpleCounts[i] = 0;
          lastDetectedColors[i] = COLOR_NONE;
        }

        String colorName = lastDetectedColors[i] == COLOR_GREEN ? "GREEN" :
                lastDetectedColors[i] == COLOR_PURPLE ? "PURPLE" : "NONE";

        telemetry.addData("Position " + (i+1), "%s (Hue %.1f)", colorName, hue);
      }

      telemetry.update();
      sleep(50);
    }
  }
}

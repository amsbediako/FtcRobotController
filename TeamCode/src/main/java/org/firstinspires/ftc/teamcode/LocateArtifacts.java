
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@Autonomous(name = "Auto: Turn to Artifact (Mecanum Centered)", group = "Concept")
public class LocateArtifacts extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        // Initialize Mecanum motors
        frontLeft  = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight  = hardwareMap.get(DcMotor.class, "right_back_drive");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Purple artifact detector
        ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setCircleFitColor(Color.MAGENTA)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // Green artifact detector
        ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setCircleFitColor(Color.GREEN)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // Build VisionPortal with both processors
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);

        waitForStart();

        while (opModeIsActive()) {

            // Get blobs
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> greenBlobs  = greenLocator.getBlobs();

            // Combine blobs
            List<ColorBlobLocatorProcessor.Blob> allBlobs = purpleBlobs;
            allBlobs.addAll(greenBlobs);

            // Filter
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 20000, allBlobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.6, 1, allBlobs);

            if (!allBlobs.isEmpty()) {

                // Find the largest blob (closest artifact)
                ColorBlobLocatorProcessor.Blob target = allBlobs.get(0);
                double maxRadius = target.getCircle().getRadius();
                for (ColorBlobLocatorProcessor.Blob b : allBlobs) {
                    if (b.getCircle().getRadius() > maxRadius) {
                        target = b;
                        maxRadius = b.getCircle().getRadius();
                    }
                }

                Circle c = target.getCircle();
                double offsetX = c.getX() - 160; // camera width = 320, center = 160

                // Telemetry
                String color = purpleBlobs.contains(target) ? "PURPLE" : "GREEN (FREE)";
                telemetry.addLine("Closest Artifact: " + color);
                telemetry.addLine(String.format("Offset X: %.2f pixels, Radius: %.2f", offsetX, c.getRadius()));

                // Dead zone: stop turning if close to center
                double deadZone = 10.0; // pixels
                double turnPower = 0;

                if (Math.abs(offsetX) > deadZone) {
                    double k = 0.01; // increased from 0.005 to 0.01 for faster turning
                    turnPower = Range.clip(k * offsetX, -0.6, 0.6); // increased max power from 0.3 to 0.6
                }

                // Apply turning power
                frontLeft.setPower(turnPower);
                backLeft.setPower(turnPower);
                frontRight.setPower(turnPower);
                backRight.setPower(turnPower);

            } else {
                telemetry.addLine("No Artifact Detected");

                // Stop motors
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }

            telemetry.update();
            sleep(50);
        }
    }
}

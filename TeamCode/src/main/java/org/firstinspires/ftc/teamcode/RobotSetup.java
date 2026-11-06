package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "Robot Setup", group = "Setup")
@Disabled
public class RobotSetup extends LinearOpMode {

    private static final String WEBCAM_NAME = "Webcam 1";

    private static final int TARGET_TAG_ID = -1;

    private static final double TARGET_RANGE_IN = 0.0;
    private static final double TARGET_BEARING_DEG = 0.0;
    private static final double TARGET_YAW_DEG = 0.0;

    private static final double RANGE_TOLERANCE_IN = 1.0;
    private static final double BEARING_TOLERANCE_DEG = 1.5;
    private static final double YAW_TOLERANCE_DEG = 1.5;

    private static final RevBlinkinLedDriver.BlinkinPattern ALIGNED_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    private static final RevBlinkinLedDriver.BlinkinPattern CLOSE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    private static final RevBlinkinLedDriver.BlinkinPattern NO_TAG_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern = null;

    private boolean hasSavedTarget = false;
    private int savedTagId = TARGET_TAG_ID;
    private double savedRange = TARGET_RANGE_IN;
    private double savedBearing = TARGET_BEARING_DEG;
    private double savedYaw = TARGET_YAW_DEG;

    private boolean lastA = false;

    @Override
    public void runOpMode() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();

        hasSavedTarget = TARGET_TAG_ID != -1;

        setLedPattern(NO_TAG_PATTERN);

        try {
            while (!isStopRequested()) {
                AprilTagSingleDetection tag = findTag();

                boolean aPressed = gamepad1.a && !lastA;
                lastA = gamepad1.a;

                if (aPressed && tag != null) {
                    savedTagId = tag.id;
                    savedRange = tag.ftcPose.range;
                    savedBearing = tag.ftcPose.bearing;
                    savedYaw = tag.ftcPose.yaw;
                    hasSavedTarget = true;
                }

                telemetry.addLine("=== ROBOT SETUP ===");
                telemetry.addLine("Place robot, LEDs go GREEN when lined up");
                telemetry.addLine("G1 A: save current spot as the target");

                if (tag == null) {
                    setLedPattern(NO_TAG_PATTERN);
                    telemetry.addLine("");
                    telemetry.addLine(hasSavedTarget
                            ? String.format("Looking for tag %d...", savedTagId)
                            : "No tag seen");
                } else {
                    double range = tag.ftcPose.range;
                    double bearing = tag.ftcPose.bearing;
                    double yaw = tag.ftcPose.yaw;

                    telemetry.addLine("");
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Range (in)", "%.2f", range);
                    telemetry.addData("Bearing (deg)", "%.2f", bearing);
                    telemetry.addData("Yaw (deg)", "%.2f", yaw);

                    if (hasSavedTarget) {
                        double rangeError = range - savedRange;
                        double bearingError = bearing - savedBearing;
                        double yawError = yaw - savedYaw;

                        boolean rangeOk = Math.abs(rangeError) <= RANGE_TOLERANCE_IN;
                        boolean bearingOk = Math.abs(bearingError) <= BEARING_TOLERANCE_DEG;
                        boolean yawOk = Math.abs(yawError) <= YAW_TOLERANCE_DEG;
                        boolean aligned = rangeOk && bearingOk && yawOk;

                        setLedPattern(aligned ? ALIGNED_PATTERN : CLOSE_PATTERN);

                        telemetry.addLine("");
                        telemetry.addLine(aligned ? ">>> ALIGNED <<<" : "NOT ALIGNED");
                        telemetry.addLine(String.format("Range   %+.2f in  %s", rangeError, rangeOk ? "OK" : moveHint(rangeError, "back up", "move closer")));
                        telemetry.addLine(String.format("Bearing %+.2f deg %s", bearingError, bearingOk ? "OK" : moveHint(bearingError, "slide left", "slide right")));
                        telemetry.addLine(String.format("Yaw     %+.2f deg %s", yawError, yawOk ? "OK" : moveHint(yawError, "rotate one way", "rotate other way")));
                    } else {
                        setLedPattern(CLOSE_PATTERN);
                        telemetry.addLine("");
                        telemetry.addLine("No target saved yet, press A at the right spot");
                    }
                }

                if (hasSavedTarget) {
                    telemetry.addLine("");
                    telemetry.addLine("Copy these into the code to make them permanent:");
                    telemetry.addLine(String.format("TARGET_TAG_ID = %d", savedTagId));
                    telemetry.addLine(String.format("TARGET_RANGE_IN = %.2f", savedRange));
                    telemetry.addLine(String.format("TARGET_BEARING_DEG = %.2f", savedBearing));
                    telemetry.addLine(String.format("TARGET_YAW_DEG = %.2f", savedYaw));
                }

                telemetry.update();
                idle();
            }
        } finally {
            visionPortal.close();
            setLedPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

    private AprilTagSingleDetection findTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagSingleDetection best = null;

        for (AprilTagDetection raw : detections) {
            if (!(raw instanceof AprilTagSingleDetection)) {
                continue;
            }
            AprilTagSingleDetection detection = (AprilTagSingleDetection) raw;
            if (detection.metadata == null || detection.ftcPose == null) {
                continue;
            }
            if (hasSavedTarget && detection.id != savedTagId) {
                continue;
            }
            if (best == null || detection.ftcPose.range < best.ftcPose.range) {
                best = detection;
            }
        }

        return best;
    }

    private String moveHint(double error, String positiveHint, String negativeHint) {
        return error > 0 ? positiveHint : negativeHint;
    }

    private void setLedPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (pattern != currentPattern) {
            blinkin.setPattern(pattern);
            currentPattern = pattern;
        }
    }
}
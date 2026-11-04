package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.AngularVelocity;

@TeleOp(name = "TeleopFinal26266", group = "Test")
public class TeleopIndicatorLight extends LinearOpMode implements NextRobot {

    private DcMotor lf, rf, lr, rr;
    private DcMotor intake, transfer;

    private NextMotor launcher;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern = null;

    private enum ScoringElement {
        POLLEN,
        NECTAR
    }

    private ScoringElement selectedElement = ScoringElement.POLLEN;

    private static final double DRIVE_POWER = -1.0;
    private static final double INTAKE_POWER = -1.0;
    private static final double TRANSFER_POWER = -0.8;

    private static final AngularVelocity POLLEN_LAUNCH_VELOCITY = new AngularVelocity(2200, Units.RotationsPerMinute);
    private static final AngularVelocity NECTAR_LAUNCH_VELOCITY = new AngularVelocity(1000, Units.RotationsPerMinute);

    private static final double SLOW_MULTIPLIER = 0.5;
    private static final double DEADBAND = 0.05;

    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int[] TARGET_TAG_IDS = {};
    private static final double CAMERA_X_OFFSET_IN = 1.5;
    private static final double CAMERA_YAW_OFFSET_DEG = 0.0;
    private static final double AIM_BEARING_TOLERANCE_DEG = 3.0;
    private static final double AIM_RANGE_MIN_IN = 0.0;
    private static final double AIM_RANGE_MAX_IN = 200.0;
    private static final int AIM_STABLE_LOOPS = 3;

    private static final RevBlinkinLedDriver.BlinkinPattern AIMED_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    private static final RevBlinkinLedDriver.BlinkinPattern TARGET_SEEN_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    private static final RevBlinkinLedDriver.BlinkinPattern NO_TARGET_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;

    private int aimedLoopCount = 0;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rr = hardwareMap.get(DcMotor.class, "rr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();

        launcher = new NextMotor(RobotController.expansionHub(), 0);
        launcher.getVelocityConstants()
                .withP(0.00001)
                .withI(0)
                .withD(0)
                .withV(0.000425)
                .withS(0.12)
                .withA(0);

        DcMotor[] motors = {
                lf, rf, lr, rr, intake, transfer
        };

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(NextMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);

        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        setLedPattern(NO_TARGET_PATTERN);

        telemetry.addLine("Ready");
        telemetry.addLine("G1 left stick: drive and strafe");
        telemetry.addLine("G1 right stick X: turn");
        telemetry.addLine("G1 B: hold to stop all motors");
        telemetry.addLine("G2 left trigger: intake + transfer");
        telemetry.addLine("G2 right trigger: launch");
        telemetry.addLine("G2 Y: pollen | X: nectar");
        telemetry.addLine("G2 A: hold for half-speed intake + transfer");
        telemetry.addLine("G2 B: hold to reverse intake + transfer");
        telemetry.addLine("LEDs: green = aimed, gold = tag seen, red = no tag");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                if (gamepad2.y) {
                    selectedElement = ScoringElement.POLLEN;
                } else if (gamepad2.x) {
                    selectedElement = ScoringElement.NECTAR;
                }

                double forward = applyDeadband(gamepad1.left_stick_x);
                double strafe = applyDeadband(gamepad1.left_stick_y);
                double turn = applyDeadband(gamepad1.right_stick_x);

                double denominator = Math.max(
                        Math.abs(forward) + Math.abs(strafe) + Math.abs(turn),
                        1.0
                );

                double lfPower = (forward + strafe + turn) / denominator * DRIVE_POWER;
                double lrPower = (forward - strafe + turn) / denominator * DRIVE_POWER;
                double rfPower = (forward - strafe - turn) / denominator * DRIVE_POWER;
                double rrPower = (forward + strafe - turn) / denominator * DRIVE_POWER;

                boolean slowMode = gamepad2.a;
                boolean reverseRequested = gamepad2.b;
                boolean stopped = gamepad1.b;

                double speedMultiplier = slowMode ? SLOW_MULTIPLIER : 1.0;

                double intakeTrigger = applyDeadband(gamepad2.left_trigger);
                double launchTrigger = applyDeadband(gamepad2.right_trigger);

                double intakePower = intakeTrigger * INTAKE_POWER * speedMultiplier;
                double transferPower = intakeTrigger * TRANSFER_POWER * speedMultiplier;

                if (launchTrigger > 0.5) {
                    if (selectedElement == ScoringElement.POLLEN) {
                        launcher.setVelocitySetpoint(POLLEN_LAUNCH_VELOCITY);
                    } else {
                        launcher.setVelocitySetpoint(NECTAR_LAUNCH_VELOCITY);
                    }
                } else {
                    launcher.setVelocitySetpoint(new AngularVelocity(0, Units.RotationsPerMinute));
                }

                if (reverseRequested) {
                    intakePower = -INTAKE_POWER;
                    transferPower = -TRANSFER_POWER;
                }

                if (stopped) {
                    stopAll();
                } else {
                    lf.setPower(lfPower);
                    lr.setPower(lrPower);
                    rf.setPower(rfPower);
                    rr.setPower(rrPower);

                    intake.setPower(intakePower);
                    transfer.setPower(transferPower);
                    launcher.update();
                }

                AprilTagSingleDetection target = findTarget();
                boolean targetSeen = target != null;
                double bearing = targetSeen ? robotBearing(target) : 0.0;
                double range = targetSeen ? target.ftcPose.range : 0.0;

                boolean inAimWindow = targetSeen
                        && Math.abs(bearing) <= AIM_BEARING_TOLERANCE_DEG
                        && range >= AIM_RANGE_MIN_IN
                        && range <= AIM_RANGE_MAX_IN;

                if (inAimWindow) {
                    aimedLoopCount++;
                } else {
                    aimedLoopCount = 0;
                }

                boolean aimed = aimedLoopCount >= AIM_STABLE_LOOPS;

                if (aimed) {
                    setLedPattern(AIMED_PATTERN);
                } else if (targetSeen) {
                    setLedPattern(TARGET_SEEN_PATTERN);
                } else {
                    setLedPattern(NO_TARGET_PATTERN);
                }

                String intakeMode;
                if (stopped) {
                    intakeMode = "STOPPED";
                } else if (reverseRequested) {
                    intakeMode = "REVERSE";
                } else if (intakeTrigger == 0) {
                    intakeMode = "IDLE";
                } else if (slowMode) {
                    intakeMode = "HALF SPEED";
                } else {
                    intakeMode = "NORMAL";
                }

                telemetry.addLine("=== APRILTAG VISION ===");
                telemetry.addData("Robot bearing (deg)", "%.2f", bearing);
                addAprilTagTelemetry();

                telemetry.addLine("=== CONTROLLER 2 ===");
                telemetry.addData("G2 left trigger", "%.2f", gamepad2.left_trigger);
                telemetry.addData("G2 right trigger", "%.2f", gamepad2.right_trigger);
                telemetry.addData("G2 A / slow mode", slowMode);
                telemetry.addData("G2 B / reverse", reverseRequested);
                telemetry.addData("G2 Y / pollen", gamepad2.y);
                telemetry.addData("G2 X / nectar", gamepad2.x);

                telemetry.addData("Intake / transfer mode", intakeMode);
                telemetry.addData("Selected element", selectedElement);

                telemetry.addLine("=== MECHANISM POWER ===");
                telemetry.addData("Intake", "%.2f", intake.getPower());
                telemetry.addData("Transfer", "%.2f", transfer.getPower());
                telemetry.addData("Launcher", "%.2f", launcher.getEncoderVelocity().getMagnitude());

                telemetry.addLine("=== DRIVE ===");
                telemetry.addData("Left front", "%.2f", lf.getPower());
                telemetry.addData("Right front", "%.2f", rf.getPower());
                telemetry.addData("Left rear", "%.2f", lr.getPower());
                telemetry.addData("Right rear", "%.2f", rr.getPower());
                telemetry.addData("G1 B / all motors stopped", stopped);
                telemetry.update();

                idle();
            }
        } finally {
            stopAll();
            visionPortal.close();
            setLedPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

    private void addAprilTagTelemetry() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags detected", detections.size());

        for (AprilTagDetection raw : detections) {
            if (!(raw instanceof AprilTagSingleDetection)) {
                continue;
            }
            AprilTagSingleDetection detection = (AprilTagSingleDetection) raw;
            if (detection.metadata != null) {
                telemetry.addLine(String.format("ID %d (%s)", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("ID %d (Unknown)", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f  (pixels)",
                        detection.center.x, detection.center.y));
            }
        }
    }

    private double robotBearing(AprilTagDetection detection) {
        double x = detection.ftcPose.x + CAMERA_X_OFFSET_IN;
        double y = detection.ftcPose.y;
        return Math.toDegrees(-Math.atan2(x, y)) - CAMERA_YAW_OFFSET_DEG;
    }

    private AprilTagSingleDetection findTarget() {
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
            if (!isTargetId(detection.id)) {
                continue;
            }
            if (best == null || Math.abs(robotBearing(detection)) < Math.abs(robotBearing(best))) {
                best = detection;
            }
        }

        return best;
    }

    private boolean isTargetId(int id) {
        if (TARGET_TAG_IDS.length == 0) {
            return true;
        }
        for (int targetId : TARGET_TAG_IDS) {
            if (targetId == id) {
                return true;
            }
        }
        return false;
    }

    private void setLedPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (pattern != currentPattern) {
            blinkin.setPattern(pattern);
            currentPattern = pattern;
        }
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > DEADBAND ? input : 0.0;
    }

    private void stopAll() {
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
        intake.setPower(0);
        launcher.setThrottle(0);
        transfer.setPower(0);
    }
}
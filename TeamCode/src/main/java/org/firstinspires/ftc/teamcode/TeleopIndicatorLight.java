package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.AngularVelocity;

public abstract class TeleopIndicatorLight extends LinearOpMode implements NextRobot {

    private DcMotor lf, rf, lr, rr;
    private DcMotor intake, transfer;

    private NextMotor launcher;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private RevBlinkinLedDriver blinkin;

    private RevBlinkinLedDriver.BlinkinPattern currentPattern = null;

    private enum ScoringElement {
        pollen,
        nectar
    }

    private ScoringElement currentballtoscore = ScoringElement.pollen;

    private static final double drivemotorpower = -1.0;
    private static final double intakemotorpower = -1.0;
    private static final double transfermotorpower = -0.8;

    private static final AngularVelocity pollenvelocity = new AngularVelocity(2200, Units.RotationsPerMinute);
    private static final AngularVelocity nectarvelocity = new AngularVelocity(1000, Units.RotationsPerMinute);

    private static final double slowspeed = 0.5;
    private static final double deadband = 0.05;

    private static final String webcamonhub = "Webcam 1";
    private static final boolean automaticexposure = true;
    private static final long exposurelevel = 6;
    private static final int gain = 250;
    private boolean exposureSet = false;
    protected abstract String[] targetClusterName();
    private static final int minimumcluster = 25;
    private static final double maximumscoringangle = 90.0;
    private static final double cameraoffset = 1.5;
    private static final double camerayawoffset = 0.0;
    private static final double aimtolerance = 20.0;
    private static final double aimrange = 0.0;
    private static final double maximumaimingrange = 1000.0;
    private static final long heldaim = 400;
    private static final long targetholding = 400;

    private static final RevBlinkinLedDriver.BlinkinPattern aimedcolor = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    private static final RevBlinkinLedDriver.BlinkinPattern hiveseencolor = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    private static final RevBlinkinLedDriver.BlinkinPattern nohivecolor = RevBlinkinLedDriver.BlinkinPattern.RED;

    private long lastaimedms = -100000;
    private long lastseenms = -100000;

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
                .setCamera(hardwareMap.get(WebcamName.class, webcamonhub))
                .setCameraResolution(new Size(1280,720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
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

        setLedPattern(nohivecolor);

        while (opModeInInit()) {
            telemetry.addLine("Ready");
            telemetry.addData("Aiming at", targetClusterName());
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
            if (!exposureSet && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                applyExposure();
            }
            idle();
        }

        try {
            while (opModeIsActive()) {
                if (gamepad2.y) {
                    currentballtoscore = ScoringElement.pollen;
                } else if (gamepad2.x) {
                    currentballtoscore = ScoringElement.nectar;
                }

                double forward = applyDeadband(gamepad1.left_stick_x);
                double strafe = applyDeadband(gamepad1.left_stick_y);
                double turn = applyDeadband(gamepad1.right_stick_x);


                double denominator = Math.max(
                        Math.abs(forward) + Math.abs(strafe) + Math.abs(turn),
                        1.0
                );

                double lfPower = (forward + strafe + turn) / denominator * drivemotorpower;
                double lrPower = (forward - strafe + turn) / denominator * drivemotorpower;
                double rfPower = (forward - strafe - turn) / denominator * drivemotorpower;
                double rrPower = (forward + strafe - turn) / denominator * drivemotorpower;

                boolean slowMode = gamepad2.a;
                boolean reverseRequested = gamepad2.b;
                boolean stopped = gamepad1.b;

                double speedMultiplier = slowMode ? slowspeed : 1.0;

                double intakeTrigger = applyDeadband(gamepad2.left_trigger);
                double launchTrigger = applyDeadband(gamepad2.right_trigger);

                double intakePower = intakeTrigger * intakemotorpower * speedMultiplier;
                double transferPower = intakeTrigger * transfermotorpower * speedMultiplier;

                if (launchTrigger > 0.5) {
                    if (currentballtoscore == ScoringElement.pollen) {
                        launcher.setVelocitySetpoint(pollenvelocity);
                    } else {
                        launcher.setVelocitySetpoint(nectarvelocity);
                    }
                } else {
                    launcher.setVelocitySetpoint(new AngularVelocity(0, Units.RotationsPerMinute));
                }

                if (reverseRequested) {
                    intakePower = -intakemotorpower;
                    transferPower = -transfermotorpower;
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

                AprilTagClusterDetection target = findTarget();
                boolean targetSeen = target != null;
                double bearing = targetSeen ? robotBearing(target) : 0.0;
                double range = targetSeen ? target.ftcPose.range : 0.0;

                boolean inAimWindow = targetSeen
                        && Math.abs(bearing) <= aimtolerance
                        && range >= aimrange
                        && range <= maximumaimingrange;

                long nowMs = System.currentTimeMillis();
                if (inAimWindow) {
                    lastaimedms = nowMs;
                }
                if (targetSeen) {
                    lastseenms = nowMs;
                }

                boolean aimed = nowMs - lastaimedms <= heldaim;
                boolean recentlySeen = nowMs - lastseenms <= targetholding;

                if (aimed) {
                    setLedPattern(aimedcolor);
                } else if (recentlySeen) {
                    setLedPattern(hiveseencolor);
                } else {
                    setLedPattern(nohivecolor);
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
                telemetry.addData("Selected element", currentballtoscore);

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
            setLedPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        }
    }

    private void addAprilTagTelemetry() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags detected", detections.size());

        for (AprilTagDetection raw : detections) {
            if (raw instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) raw;
                telemetry.addLine(String.format("Cluster %s (%d%% of tags seen)",
                        cluster.metadata.name, cluster.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        cluster.ftcPose.pitch, cluster.ftcPose.roll, cluster.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        cluster.ftcPose.range, cluster.ftcPose.bearing, cluster.ftcPose.elevation));
                continue;
            }
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
        double x = detection.ftcPose.x + cameraoffset;
        double y = detection.ftcPose.y;
        return Math.toDegrees(-Math.atan2(x, y)) - camerayawoffset;
    }

    private AprilTagClusterDetection findTarget() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagClusterDetection best = null;

        for (AprilTagDetection raw : detections) {
            if (!(raw instanceof AprilTagClusterDetection)) {
                continue;
            }
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) raw;
            if (cluster.metadata == null || cluster.ftcPose == null) {
                continue;
            }
            if (cluster.percentClusterFound < minimumcluster) {
                continue;
            }
            if (!isTargetCluster(cluster.metadata.name)) {
                continue;
            }
            if (Math.abs(cluster.ftcPose.roll) >= maximumscoringangle) {
                continue;
            }
            if (best == null
                    || cluster.percentClusterFound > best.percentClusterFound
                    || (cluster.percentClusterFound == best.percentClusterFound
                        && Math.abs(robotBearing(cluster)) < Math.abs(robotBearing(best)))) {
                best = cluster;
            }
        }

        return best;
    }

    private boolean isTargetCluster(String name) {
        return Arrays.asList(targetClusterName()).contains(name);
    }

    private void applyExposure() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (automaticexposure) {
            exposureControl.setMode(ExposureControl.Mode.Auto);
        } else {
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposurelevel, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
        }
        exposureSet = true;
    }

    private void setLedPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (pattern != currentPattern) {
            blinkin.setPattern(pattern);
            currentPattern = pattern;
        }
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > deadband ? input : 0.0;
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
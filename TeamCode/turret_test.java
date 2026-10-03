package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
class turret_test extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;

    private static final double PROPORTIONAL_GAIN = 0.02;
    private static final double MAX_MOTOR_POWER = 1.0;
    private static final double DEADZONE = 1.0;

    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Define your AprilTag clusters here
    // For example, if tags 1 and 2 form a cluster:
    private static final int[] CLUSTER_1_TAGS = {1, 2};
    private static final int[] CLUSTER_2_TAGS = {3, 4};

    @Override
    public void runOpMode() {
        initAprilTag();

        DcMotor turretmotor = hardwareMap.get(DcMotor.class, "turret_motor");
        turretmotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetryAprilTag();

            telemetry.update();
            sleep(20);
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private double calculateAimbotPower() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.isEmpty()) {
            telemetry.addData("Aimbot", "No tags");
            return 0;
        }

        // Try to find a cluster first
        AprilTagCluster bestCluster = findBestCluster(detections);
        if (bestCluster != null) {
            double offset = bestCluster.centerX;
            telemetry.addData("Target Type", "Cluster %d", bestCluster.id);
            telemetry.addData("Offset", "%.1f", offset);

            if (Math.abs(offset) < DEADZONE) {
                telemetry.addData("Lock", "true");
                return 0;
            }

            double power = offset * PROPORTIONAL_GAIN;
            power = Math.max(-MAX_MOTOR_POWER, Math.min(MAX_MOTOR_POWER, power));
            telemetry.addData("Motor Power", "%.2f", power);
            return power;
        }

        // Fall back to individual tags if no cluster found
        AprilTagDetection bestTag = null;
        double bestDistance = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                double distance = detection.robotPose.getPosition().z;
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestTag = detection;
                }
            }
        }

        if (bestTag == null) {
            telemetry.addData("Aimbot", "No valid target");
            return 0;
        }

        double offset = bestTag.robotPose.getPosition().x;

        telemetry.addData("Offset", "%.1f", offset);

        if (Math.abs(offset) < DEADZONE) {
            telemetry.addData("Lock", "true");
            return 0;
        }

        double power = offset * PROPORTIONAL_GAIN;
        power = Math.max(-MAX_MOTOR_POWER, Math.min(MAX_MOTOR_POWER, power));

        telemetry.addData("Motor Power", "%.2f", power);
        return power;
    }

    private AprilTagCluster findBestCluster(List<AprilTagDetection> detections) {
        AprilTagCluster bestCluster = null;
        double bestDistance = Double.MAX_VALUE;

        // Check each cluster definition
        int[][] clusters = {CLUSTER_1_TAGS, CLUSTER_2_TAGS};
        
        for (int clusterIndex = 0; clusterIndex < clusters.length; clusterIndex++) {
            int[] clusterTags = clusters[clusterIndex];
            int matchCount = 0;
            double sumX = 0;
            double sumZ = 0;

            // Count how many tags in this cluster are detected
            for (AprilTagDetection detection : detections) {
                for (int tagId : clusterTags) {
                    if (detection.id == tagId) {
                        matchCount++;
                        sumX += detection.robotPose.getPosition().x;
                        sumZ += detection.robotPose.getPosition().z;
                        break;
                    }
                }
            }

            // If we found at least 2 tags from the cluster, consider it valid
            if (matchCount >= 2) {
                double avgDistance = sumZ / matchCount;
                if (avgDistance < bestDistance) {
                    bestDistance = avgDistance;
                    double centerX = sumX / matchCount;
                    bestCluster = new AprilTagCluster(clusterIndex, centerX, avgDistance);
                }
            }
        }

        return bestCluster;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("ID %d %s", detection.id, detection.metadata.name));
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                }
            }
        }
    }

    // Simple class to represent an AprilTag cluster
    private static class AprilTagCluster {
        int id;
        double centerX;
        double distance;

        AprilTagCluster(int id, double centerX, double distance) {
            this.id = id;
            this.centerX = centerX;
            this.distance = distance;
        }
    }
}

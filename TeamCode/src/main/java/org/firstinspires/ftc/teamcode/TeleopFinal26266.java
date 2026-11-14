package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.AngularVelocity;

@TeleOp(name = "TeleopFinal26266", group = "Test")
public class TeleopFinal26266 extends LinearOpMode implements NextRobot {

    private DcMotor lf, rf, lr, rr;
    private DcMotor intake, transfer;


    private NextMotor launcher;

    private enum ScoringElement {
        pollenballs,
        nectarballs
    }

    private ScoringElement selectedElement = ScoringElement.pollenballs;

    private static final double drivemotorpower = -1.0;
    private static final double intakemotorpower = -1.0;
    private static final double transfermotorpower = -0.8;
    private static final double pollenlaunch = -0.85;
    private static final double nectarlaunch = -0.9;

    private static final AngularVelocity pollenvelocity = new AngularVelocity(2200, Units.RotationsPerMinute);
    private static final AngularVelocity nectarvelocity = new AngularVelocity(1000, Units.RotationsPerMinute);

    private static final double slowspeed = 0.5;
    private static final double deadband = 0.05;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rr = hardwareMap.get(DcMotor.class, "rr");
        intake = hardwareMap.get(DcMotor.class, "intake");


        launcher = new NextMotor(RobotController.expansionHub(),0);
        launcher.getVelocityConstants()
                .withP(0.00001)
                .withI(0)
                .withD(0)
                .withV(0.000425)

                .withS(0.12)
                .withA(0);


        transfer = hardwareMap.get(DcMotor.class, "transfer");

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

        telemetry.addLine("Ready");
        telemetry.addLine("G1 left stick: drive and strafe");
        telemetry.addLine("G1 right stick X: turn");
        telemetry.addLine("G1 B: hold to stop all motors");
        telemetry.addLine("G2 left trigger: intake + transfer");
        telemetry.addLine("G2 right trigger: launch");
        telemetry.addLine("G2 Y: pollen | X: nectar");
        telemetry.addLine("G2 A: hold for half-speed intake + transfer");
        telemetry.addLine("G2 B: hold to reverse intake + transfer");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                if (gamepad2.y) {
                    selectedElement = ScoringElement.pollenballs;
                } else if (gamepad2.x) {
                    selectedElement = ScoringElement.nectarballs;
                }

                // Original drive mapping.
                double forward = applyDeadband(gamepad1.left_stick_x);
                double strafe = applyDeadband(gamepad1.left_stick_y);
                double turn = applyDeadband(gamepad1.right_stick_x);

                double denominator = Math.max(
                        Math.abs(forward)
                                + Math.abs(strafe)
                                + Math.abs(turn),
                        1.0
                );

                double lfPower =
                        (forward + strafe + turn)
                                / denominator * drivemotorpower;

                double lrPower =
                        (forward - strafe + turn)
                                / denominator * drivemotorpower;

                double rfPower =
                        (forward - strafe - turn)
                                / denominator * drivemotorpower;

                double rrPower =
                        (forward + strafe - turn)
                                / denominator * drivemotorpower;

                boolean slowMode = gamepad2.a;
                boolean reverseRequested = gamepad2.b;
                boolean stopped = gamepad1.b;

                double speedMultiplier =
                        slowMode ? slowspeed : 1.0;

                double intakeTrigger =
                        applyDeadband(gamepad2.left_trigger);

                double launchTrigger =
                        applyDeadband(gamepad2.right_trigger);

                double intakePower =
                        intakeTrigger * intakemotorpower * speedMultiplier;

                double transferPower =
                        intakeTrigger * transfermotorpower * speedMultiplier;

              /*  double launchPreset =
                        selectedElement == ScoringElement.POLLEN
                                ? POLLEN_LAUNCH_POWER
                                : NECTAR_LAUNCH_POWER; */
                if (launchTrigger > 0.5) {
                    if (selectedElement == ScoringElement.pollenballs) {
                        launcher.setVelocitySetpoint(pollenvelocity);
                    } else {
                        launcher.setVelocitySetpoint(nectarvelocity);
                    }
                } else {
                    launcher.setVelocitySetpoint(new AngularVelocity(0, Units.RotationsPerMinute));
                }


              /*  double launcherPower =
                        launchTrigger * launchPreset;*/

                // Reverse overrides slow mode and both triggers.
                if (reverseRequested) {
                    intakePower = -intakemotorpower;
                    transferPower = -transfermotorpower;

                    //launcherPower = 0;
                }

                // Controller 1 B overrides all motor commands.
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

                telemetry.addLine("=== CONTROLLER 2 ===");
                telemetry.addData(
                        "G2 left trigger", "%.2f", gamepad2.left_trigger);
                telemetry.addData(
                        "G2 right trigger", "%.2f", gamepad2.right_trigger);
                telemetry.addData("G2 A / slow mode", slowMode);
                telemetry.addData("G2 B / reverse", reverseRequested);
                telemetry.addData("G2 Y / pollen", gamepad2.y);
                telemetry.addData("G2 X / nectar", gamepad2.x);

                telemetry.addData("Intake / transfer mode", intakeMode);
                telemetry.addData("Selected element", selectedElement);
                //telemetry.addData("Launch velocity target", "%.2f", launcher.getVelocityPID().);

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
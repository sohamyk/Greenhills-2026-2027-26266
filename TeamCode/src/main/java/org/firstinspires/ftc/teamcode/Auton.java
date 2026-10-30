package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Autonomous")
public class Auton extends LinearOpMode {

    static final double DRIVE_POWER = 0.6;
    static final double TURN_POWER = 0.4;
    static final double LAUNCHER_POWER = 0.85;
    static final double TRANSFER_POWER = 0.7;
    static final double INTAKE_POWER = 1.0;

    DcMotor lf, rf, lr, rr, intake, launcher, transfer;
    boolean red = true;

    @Override
    public void runOpMode() {
        setup();

        while (opModeInInit()) {
            if (gamepad1.b) red = true;
            if (gamepad1.x) red = false;
            telemetry.addData("Alliance", red ? "RED (B)" : "BLUE (X)");
            telemetry.addLine("Press B for red, X for blue");
            telemetry.update();
        }
        if (isStopRequested()) return;

        if (red) {
            runRed();
        } else {
            runBlue();
        }

        stopAll();
    }

    void runRed() {
        left(400);
        pause(200);
        forward(3000);
    }

    void runBlue() {
        right(400);
        pause(200);
        forward(1100);
    }

    void forward(long ms) { move(DRIVE_POWER, 0, 0, ms); }
    void back(long ms) { move(-DRIVE_POWER, 0, 0, ms); }
    void left(long ms) { move(0, -DRIVE_POWER, 0, ms); }
    void right(long ms) { move(0, DRIVE_POWER, 0, ms); }
    void turnLeft(long ms) { move(0, 0, -TURN_POWER, ms); }
    void turnRight(long ms) { move(0, 0, TURN_POWER, ms); }

    void pause(long ms) {
        sleep(ms);
    }

    void shoot(long ms) {
        launcher.setPower(LAUNCHER_POWER);
        sleep(3000);
        transfer.setPower(TRANSFER_POWER);
        intake.setPower(INTAKE_POWER);
        sleep(ms);
        launcher.setPower(0);
        transfer.setPower(0);
        intake.setPower(0);
    }

    void runIntake(long ms) {
        intake.setPower(INTAKE_POWER);
        transfer.setPower(TRANSFER_POWER);
        sleep(ms);
        intake.setPower(0);
        transfer.setPower(0);
    }

    void driveWhileIntaking(long ms) {
        intake.setPower(INTAKE_POWER);
        transfer.setPower(TRANSFER_POWER);
        move(DRIVE_POWER, 0, 0, ms);
        intake.setPower(0);
        transfer.setPower(0);
    }

    void move(double forward, double strafeRight, double turnRight, long ms) {
        setDrive(forward, strafeRight, turnRight);
        sleep(ms);
        setDrive(0, 0, 0);
    }

    void setDrive(double forward, double strafeRight, double turnRight) {
        double x = strafeRight;
        double y = -forward;
        double t = turnRight;
        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(t), 1.0);
        lf.setPower((x + y + t) / d);
        lr.setPower((x - y + t) / d);
        rf.setPower((x - y - t) / d);
        rr.setPower((x + y - t) / d);
    }

    void stopAll() {
        setDrive(0, 0, 0);
        intake.setPower(0);
        launcher.setPower(0);
        transfer.setPower(0);
    }

    void setup() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rr = hardwareMap.get(DcMotor.class, "rr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        for (DcMotor m : new DcMotor[]{lf, rf, lr, rr, intake, launcher, transfer}) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
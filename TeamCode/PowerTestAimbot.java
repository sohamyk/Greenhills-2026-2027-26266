/*
Copyright 2026 FIRST Tech Challenge Team 5555

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class PowerTestAimbot extends LinearOpMode {

    private DcMotor turretmotor = null;
    private DcMotor flywheelmotor = null;

    boolean turretoverride = false;
    @Override
    public void runOpMode() {


        turretmotor = hardwareMap.get(DcMotor.class, "turret_motor");


        turretmotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        double turretMotorPower = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Push telemetry to the Driver Station.

            // Save CPU resources; can resume streaming when needed.


            // Share the CPU.
            //sleep(20);

            // Send power to motor
            turretmotor.setPower(turretMotorPower);

            // Show opmode is active
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Motor Power:",turretMotorPower);
            telemetry.update();

            //Check dpad for inputs
            if(gamepad1.dpadUpWasReleased()){
                telemetry.addLine("Up");
                turretMotorPower = turretMotorPower+0.1;
                turretmotor.setPower(turretMotorPower);

            } else if(gamepad1.dpadDownWasReleased()){
                telemetry.addLine("Down");
                turretMotorPower = turretMotorPower-0.1;
                turretmotor.setPower(turretMotorPower);
            } else{
                turretmotor.setPower(turretMotorPower);
            }
            telemetry.update();
        }

    }

}

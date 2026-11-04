/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.hardware.actuators.NextMotor;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Launch + Gather + Park", group="Robot")
public class Auton extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lr = null;
    private DcMotor rr = null;
    private DcMotor intake = null;
    private DcMotor launcher = null;
    private DcMotor transfer = null;


    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.5;
    static final double STOP = 0.0;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        lf = hardwareMap.get(DcMotor.class, "fl");
        rf = hardwareMap.get(DcMotor.class, "fr");
        rr = hardwareMap.get(DcMotor.class, "br");
        lr = hardwareMap.get(DcMotor.class, "bl");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        transfer = hardwareMap.get(DcMotor.class, "transfer");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();


        //Launch + Stop
        launcher.setPower(0.85);
        sleep(2000);
        transfer.setPower(1);
        intake.setPower(1);
        sleep(7500);
        launcher.setPower(STOP);
        sleep(100);
        //Turn
        rf.setPower(0.5);
        rr.setPower(0.5);
        sleep(1500);
        rf.setPower(STOP);
        rr.setPower(STOP);
        sleep(100);
        //Gather pollen
        lf.setPower(0.5);
        rf.setPower(0.5);
        rr.setPower(0.5);
        lr.setPower(0.5);
        transfer.setPower(1);
        intake.setPower(1);
        sleep(2700);
        //Reverse to starting point while maintaining pollen in robot
        transfer.setPower(0.5);
        intake.setPower(0.3);
        lf.setPower(-0.5);
        rf.setPower(-0.5);
        rr.setPower(-0.5);
        lr.setPower(-0.5);
        sleep(2700);
        //Rest
        lf.setPower(STOP);
        rf.setPower(STOP);
        rr.setPower(STOP);
        lr.setPower(STOP);
        sleep(1000);
        //Strafe Right
        lf.setPower(0.5);
        rr.setPower(0.5);
        lr.setPower(-0.5);
        rf.setPower(-0.5);
        sleep(4000);
        lf.setPower(STOP);
        rf.setPower(STOP);
        rr.setPower(STOP);
        lr.setPower(STOP);
        sleep(100);
        //Drive into Park Box
        lf.setPower(0.5);
        rf.setPower(0.5);
        rr.setPower(0.5);
        lr.setPower(0.5);
        sleep(2500);
        lf.setPower(STOP);
        rf.setPower(STOP);
        rr.setPower(STOP);
        lr.setPower(STOP);
        transfer.setPower(STOP);
        intake.setPower(STOP);
        sleep(1);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
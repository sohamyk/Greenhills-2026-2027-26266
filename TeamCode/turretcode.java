package org.firstinspires.ftc.teamcode.Mechanisms;

import static com.qualcomm.robotcore.util.Range.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="turretcode", group="Linear OpMode")
public class turretcode extends LinearOpMode {


    //VARIABLE DECLARATIONS

    private DcMotor turretMotor = null;

//HARDWARE MAP DECLARATIONS

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        turretmotor = hardwareMap.get(DcMotor.class, "turret_motor");
      
        turretmotor.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double turretMotorPower = 0;
            // Send power to motor
            turretMotor.setPower(turretMotorPower);

            // Show opmode is active
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            //Check dpad for inputs
            if(gamepadpad1.dpad-left){
              turretmotor.setPower(1);
           } else if(gamepad1.dpad-right){
              turretmotor.setPower(-1);
           } else{
              turretmotor.setPower(0);
        }
      }
    }

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Red", group = "Test")
public class TeleopIndicatorLightRed extends TeleopIndicatorLight {

    @Override
    protected String[] targetClusterName() {
        return new String[]{"RED SCORING", "RED AUDIENCE"};
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Blue", group = "Test")
@Disabled
public class TeleopIndicatorLightBlue extends TeleopIndicatorLight {

    @Override
    protected String[] targetClusterName() {
        return new String[]{"BLUE AUDIENCE", "BLUE SCORING"};
    }

}

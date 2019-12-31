package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMA", group = "")
public class TeleOpMA extends CommonOpMode {

    @Override
    public void runOpMode() {

        initHardware();
        armReset();

        waitForStart();

        while (opModeIsActive()) {
            drive();
            arms();
            suction();
            lift();
            lineUpBlock();
            strafeAuto();
            Grabber.setPosition(1);
            telemetry.update();
        }
    }
}

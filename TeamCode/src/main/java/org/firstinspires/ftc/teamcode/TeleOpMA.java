package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMA", group = "")
public class TeleOpMA extends CommonOpMode {

    @Override
    public void runOpMode() {

        initHardware();
        armsResetAndRun();

        waitForStart();

        while (opModeIsActive()) {
            drive();
            suction();
            //lift();
            setSpeed();
            arms();
            //capstoneControl();
            //capstoneGrabber();
            foundationGrabberControl();
            lineUpBlock();
            incrementDown();
            //incrementUp();
            getGeneralTelemetry();
            //pushOutBackwards();
            // leftactuator.setPosition(0.65);
            //rightactuator.setPosition(-0.65);
        }
    }
}

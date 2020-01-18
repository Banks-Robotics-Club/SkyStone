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
            incrementArms();
            suction();
            lift();
            setSpeed();
            capstoneGrabber();
            grabberControl();    
            lineUpBlock();
           // Grabber.setPosition(1);
            incrementDown();
            incrementUp();
            getArmTelemetry();
        }
    }
}

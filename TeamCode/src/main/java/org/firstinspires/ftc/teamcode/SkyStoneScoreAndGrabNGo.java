package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@Autonomous(name = "SkyStoneScoreAndGrabNGo", group = "")

public class SkyStoneScoreAndGrabNGo extends CommonOpMode {
    static final int SERVO_WAIT_TIME = 300;

    @Override
    public void runOpMode() {
        initHardware();

        allianceChooser();

        initPID();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (alliance == BLUE) {
                if (position == LEFT) {

                    rightactuator.setPosition(0.5);
                    //strafeLeft(100);
                    autoDriveForwardAndStoneCheck();
                    //autoScoreSkystone();
                    //strafeRight(30);
                    //driveStraightForward(40);

                    break;

                }
                if (position == RIGHT) {

                    leftactuator.setPosition(0.3);
                    rightactuator.setPosition(0.4);
                    /*strafeLeft(220);
                    sleep(100);*/
                    autoDriveForwardAndStoneCheck();
                    /*stopDriveMotors();
                    pidPower = .6;
                    sleep(100);
                    stoneDisposal();
                    sleep(700);
                    stopStoneDisposal();
                    strafeRight(110);
                    sleep(100);
                    driveStraightBackward(300);
                    sleep(100);
                    driveBackwardsSlowlyToFoundation();
                    stopDriveMotors();
                    autoSuckIn();
                    sleep(200);
                    stopStoneDisposal();
                    driveStraightForward(140);*/

                    break;
                }

            }
            if (alliance == RED) {
                if (position == RIGHT) {

                    break;
                }
                if (position == LEFT) {

                    break;
                }
            }


        }
    }
}




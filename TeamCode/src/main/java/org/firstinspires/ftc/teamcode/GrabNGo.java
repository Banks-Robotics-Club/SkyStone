package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@Autonomous(name = "GrabNGo", group = "")
public class GrabNGo extends CommonOpMode {
    static final int SERVO_WAIT_TIME = 300;

    @Override
    public void runOpMode() {
        initHardware();

        allianceChooser();

        waitForStart();

        while (opModeIsActive()) {

            // distance (in cm) to ticks = ((d/31.4)=r*1120)
            if (alliance == BLUE) {
                if (position == LEFT) {
                    driveAuto(0,1,0,1,60);

                    driveAuto(1, 0, 0, 1, 75);
                    leftFoundationGrabber.setPosition(1);
                    rightFoundationGrabbet.setPosition(0);
                    sleep(SERVO_WAIT_TIME);

                    driveAuto(-1, 0, 0, 1, -80);
                    leftFoundationGrabber.setPosition(0);
                    rightFoundationGrabbet.setPosition(1);
                    sleep(SERVO_WAIT_TIME);

                    driveAuto(0, -1, 0, 1, -70);

                    driveAuto(1, 0, 0, 1, 45);

                    driveAuto(0, 1, 0, 1, 42);

                    driveAuto(0, -1, 0, 1, -55);
                    break;
                }
                if (position == RIGHT) {

                    driveAuto(0, 1, 0, 1, 60);

                    driveAuto(1, 0, 0, .25, 75);
                    leftFoundationGrabber.setPosition(1);
                    rightFoundationGrabbet.setPosition(0);
                    sleep(SERVO_WAIT_TIME);

                    driveAuto(-1, 0, 0, .25, -80);
                    leftFoundationGrabber.setPosition(0);
                    rightFoundationGrabbet.setPosition(1);
                    sleep(SERVO_WAIT_TIME);

                    driveAuto(0, -1, 0, 1, -70);

                    driveAuto(1, 0, 0, 1, 45);

                    driveAuto(0, 1, 0, 1, 42);

                    driveAuto(-1, 0, 0, 1, -50);

                    driveAuto(0, -1, 0, 1, -55);
                    break;
                }

            }
            if (alliance == RED) {
                if (position == RIGHT) {

                    driveAuto(0, 1, 0, .1, -60);


                    driveAuto(1, 0, 0, .1, 80);
                    leftFoundationGrabber.setPosition(1);
                    rightFoundationGrabbet.setPosition(0);
                    sleep(SERVO_WAIT_TIME);

                    driveAuto(-1, 0, 0, .1, -82);
                    leftFoundationGrabber.setPosition(0);
                    rightFoundationGrabbet.setPosition(1);
                    sleep(CYCLE_MS);

                    driveAuto(0, -1, 0, .1, 70);

                    driveAuto(- 1, 0, 0, .1, 42);

                    driveAuto(0, 1, 0, .1, -38);

                    driveAuto(0, -1, 0, .1, 58);
                    break;
                }
                if (position == LEFT) {
                    driveAuto(0, -1, 0, 1, -60);

                    driveAuto(1, 0, 0, 1, 75);
                    leftFoundationGrabber.setPosition(1);
                    rightFoundationGrabbet.setPosition(0);
                    sleep(SERVO_WAIT_TIME);



                    driveAuto(-1, 0, 0, 1, -80);
                    leftFoundationGrabber.setPosition(0);
                    rightFoundationGrabbet.setPosition(1);
                    sleep(SERVO_WAIT_TIME);

                    driveAuto(0, 1, 0, 1, 70);

                    driveAuto(1, 0, 0, 1, 45);

                    driveAuto(0, -1, 0, 1, -42);

                    driveAuto(-1, 0, 0, 1, -50);

                    driveAuto(0, 1, 0, 1, 55);
                    break;
                }
            }
        }
    }
}

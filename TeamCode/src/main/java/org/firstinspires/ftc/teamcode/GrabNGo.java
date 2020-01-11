package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "GrabNGo", group = "")
public class GrabNGo extends CommonOpMode {
    static final int CYCLE_MS = 1000;

    @Override //this is a test
    public void runOpMode() {
        initHardware();

        allianceChooser();

        waitForStart();

        while (opModeIsActive()) {

            // distance (in cm) to ticks = ((d/31.4)=r*1120)
            if (alliance == BLUE) {
                if (position == LEFT) {
                    //DriveX(-160, 1);
                    driveAuto(0, -1, 0, 1, -160);

                    //DriveY(155, 1);
                    driveAuto(1, 0, 0, 1, 155);
                    Grabber.setPosition(0);
                    sleep(CYCLE_MS);
                    idle();

                    //DriveY(-169, 1);
                    driveAuto(-1, 0, 0, 1, -169);
                    Grabber.setPosition(1);
                    sleep(CYCLE_MS);
                    idle();

                    //DriveX(160, 1);
                    driveAuto(0, 1, 0, 1, 160);
                    //DriveY(105, 1);
                    driveAuto(1, 0, 0, 1, 105);
                    //DriveX(-57, 1);
                    driveAuto(0, -1, 0, 1, -50);
                    //DriveX(140, 1);
                    driveAuto(0, 1, 0, 1, 140);
                    break;
                }
                if (position == RIGHT) {
                    //DriveX(-160, 1);
                    driveAuto(0, -1, 0, 1, -160);

                    //DriveY(155, 1);
                    driveAuto(1, 0, 0, 1, 155);
                    Grabber.setPosition(0);
                    sleep(CYCLE_MS);
                    idle();

                    //DriveY(-169, 1);
                    driveAuto(-1, 0, 0, 1, -169);
                    Grabber.setPosition(1);
                    sleep(CYCLE_MS);
                    idle();

                    //DriveX(160, 1);
                    driveAuto(0, 1, 0, 1, 160);

                    //DriveY(105, 1);
                    driveAuto(1, 0, 0, 1, 105);

                    //DriveX(-57, 1);
                    driveAuto(0, -1, 0, 1, -57);

                    //DriveY(-160, 1);
                    driveAuto(-1, 0, 0, 1, -170);

                    //DriveX(140, 1);
                    driveAuto(0, 1, 0, 1, 140);
                    break;
                }

            }
            if (alliance == RED) {
                if (position == LEFT) {
                    // DriveX(160, 1);
                    driveAuto(0, 1, 0, 1, 195);

                    //DriveY(155, 1);
                    driveAuto(1, 0, 0, 1, 155);
                    Grabber.setPosition(0);
                    sleep(CYCLE_MS);
                    idle();

                    //DriveY(-170, 1);
                    driveAuto(-1, 0, 0, 1, -170);
                    Grabber.setPosition(1);
                    sleep(CYCLE_MS);
                    idle();

                    // DriveX(-220, 1);
                    driveAuto(0, -1, 0, 1, -220);

                    //DriveY(120, 1);
                    driveAuto(1, 0, 0, 1, 120);

                    //DriveX(40, 1);
                    driveAuto(0, 1, 0, 1, 40);

                    //DriveX(-140, 1);
                    driveAuto(0, -1, 0, 1, -140);
                    break;
                }
                if (position == RIGHT) {
                    //DriveX(160, 1);
                    driveAuto(0, 1, 0, 1, 160);

                    //DriveY(155, 1);
                    driveAuto(1, 0, 0, 1, 155);
                    Grabber.setPosition(0);
                    sleep(CYCLE_MS);
                    idle();

                    // DriveY(-170, 1);
                    driveAuto(-1, 0, 0, 1, -170);
                    Grabber.setPosition(1);
                    sleep(CYCLE_MS);
                    idle();

                    //DriveX(-180, 1);
                    driveAuto(0, -1, 0, 1, -180);

                    // DriveY(110, 1);
                    driveAuto(1, 0, 0, 1, 110);

                    //DriveX(70, 1);
                    driveAuto(0, 1, 0, 1, 65);

                    //DriveY(-120, 1);
                    driveAuto(-1, 0, 0, 1, -95);

                    //DriveX(-140, 1);
                    driveAuto(0, -1, 0, 1, -140);
                    break;
                }
            }
        }
    }
}

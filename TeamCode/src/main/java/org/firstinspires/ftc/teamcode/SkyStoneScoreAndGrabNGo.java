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

        waitForStart();
        while (opModeIsActive()) {
            if (alliance == BLUE) {
                if (position == LEFT) {


                    break;
                }
                if (position == RIGHT) {


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




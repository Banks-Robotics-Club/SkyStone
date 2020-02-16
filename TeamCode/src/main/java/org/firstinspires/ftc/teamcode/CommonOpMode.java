 package org.firstinspires.ftc.teamcode;


 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.ColorSensor;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.hardware.DigitalChannel;
 import com.qualcomm.robotcore.hardware.DistanceSensor;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.TouchSensor;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

 import static java.lang.Math.abs;
 // Gobilda 5202 yellow jaket ticks 383.6 = 384
 public abstract class CommonOpMode extends LinearOpMode {

     private ElapsedTime     runtime = new ElapsedTime();
     double speedAdjust = 10;
     boolean speedUp = false;
     boolean slowDown = false;
     int increment = 1;
     boolean incrementUp = false;
     boolean incrementDown = false;
     boolean grabberbuttonpushed = false;
     boolean grabberbuttonnotPushed = false;
     int currentHeight;
     //double AndyMarkToGobuilda = 383.6/537.5;

     static boolean RED = true;
     static boolean BLUE = false;
     static boolean LEFT = true;
     static boolean RIGHT = false;
     boolean alliance = BLUE;
     boolean position = RIGHT;

     boolean armPress = false;

     public DcMotor flm;
     public DcMotor blm;
     public DcMotor frm;
     public DcMotor brm;
     public DcMotor LEFTSUCTIONMOTOR;
     public DcMotor RIGHTSUCTIONMOTOR;
     public DcMotor leftarm;
     public DcMotor rightarm;
     public DcMotor lift;
     public CRServo LF;
     public CRServo RF;
     public CRServo RB;
     public CRServo LB;
     public CRServo LI;
     public CRServo RI;
     //public Servo Grabber;
     public Servo CapGrab;
     public Servo leftactuator;
     public Servo rightactuator;
     //private DistanceSensor sensorRange;
     double frPower = 0;
     double flPower = 0;
     double blPower = 0;
     double brPower = 0;
     double clockwiseRotation = 0;
     double counterclockwiseRotation = 0;
     static final int    CYCLE_MS    =   300;

     public TouchSensor leftTouchSensor;
     public TouchSensor rightTouchSensor;
     public ColorSensor leftColorSensor;
     public ColorSensor rightColorSensor;
     public DistanceSensor distanceSensor;

     public void allianceChooser() {
         if (gamepad1.b) {
             telemetry.addData("red", "alliance");
             alliance = RED;
         } else {
             telemetry.addData("blue", "alliance");
             alliance = BLUE;
         }

        if (gamepad1.a) {
             telemetry.addData("left", "");
             position = LEFT;
         } else {
             telemetry.addData("right", "");
             position = RIGHT;
         }

        telemetry.update();

     }

     public void initHardware() {
     flm = hardwareMap.dcMotor.get("frm");
     blm = hardwareMap.dcMotor.get("blm");
     frm = hardwareMap.dcMotor.get("flm");
     brm = hardwareMap.dcMotor.get("brm");
     LEFTSUCTIONMOTOR = hardwareMap.dcMotor.get("lsm");
     RIGHTSUCTIONMOTOR = hardwareMap.dcMotor.get("rsm");
     leftactuator  = hardwareMap.servo.get("LA");
     rightactuator = hardwareMap.servo.get("RA");
     //Grabber = hardwareMap.servo.get("Grabber");
     CapGrab = hardwareMap.servo.get("CapGrab");
     leftarm = hardwareMap.dcMotor.get("leftarm");
     rightarm = hardwareMap.dcMotor.get("rightarm");
    // lift = hardwareMap.dcMotor.get("lift");
     LF = hardwareMap.crservo.get("LF");
     RF = hardwareMap.crservo.get("RF");
     LB = hardwareMap.crservo.get("LB");
     RB = hardwareMap.crservo.get("RB");
     LI = hardwareMap.crservo.get("LI");
     RI = hardwareMap.crservo.get("RI");
    // sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
     leftTouchSensor = hardwareMap.get(TouchSensor.class, "LT");
     rightTouchSensor = hardwareMap.get(TouchSensor.class, "RT");
     leftColorSensor = hardwareMap.get(ColorSensor.class, "LeftColorSensor");
     rightColorSensor = hardwareMap.get(ColorSensor.class, "RightColorSensor");
     flm.setDirection(DcMotorSimple.Direction.FORWARD);
     blm.setDirection(DcMotorSimple.Direction.REVERSE);
     frm.setDirection(DcMotorSimple.Direction.REVERSE);
     brm.setDirection(DcMotorSimple.Direction.FORWARD);
     leftarm.setDirection(DcMotorSimple.Direction.FORWARD);
     rightarm.setDirection(DcMotorSimple.Direction.REVERSE);
     //lift.setDirection(DcMotorSimple.Direction.FORWARD);
     //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     LF.setDirection(DcMotorSimple.Direction.REVERSE);
     LB.setDirection(DcMotorSimple.Direction.REVERSE);
     RF.setDirection(DcMotorSimple.Direction.FORWARD);
     RB.setDirection(DcMotorSimple.Direction.FORWARD);
         CapGrab.setPosition(0.95);
     }

     public void motorForward(DcMotor motor, Integer distance, double power) {
         setupMotorToRunToPosition(motor, distance);
         motor.setPower(power);
         while (motor.isBusy()) {
             idle();
         }
         motor.setPower(0);
     }

     public void motorBackward(DcMotor motor, Integer distance, double power) {
         setupMotorToRunToPosition(motor, -distance);
         motor.setPower(-power);
         while (motor.isBusy()) {
             idle();
         }
         motor.setPower(0);
     }

     public void setupMotorToRunToPosition(DcMotor motoR, Integer distance) {
         motoR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motoR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         motoR.setTargetPosition(distance);
         motoR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     }

     public void setupMotorToRunWithoutEncoder(DcMotor motor) {
         motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     }

     public void backToNormal() {
         if (gamepad1.x) {
             setupMotorToRunWithoutEncoder(frm);
         }
     }

     public void setSpeed() {
         slowDown();
         speedUp();
         // return speedAdjust;
     }

     private void slowDown() {
         if (gamepad1.dpad_left) {
             if (!slowDown) {
                 speedAdjust = 4;
                 slowDown = true;
             }
         } else {
             slowDown = false;
         }
     }

     private void speedUp() {
         if (gamepad1.dpad_right) {
             if (!speedUp) {
                 speedAdjust = 10;
                 speedUp = true;
             }
         } else {
             speedUp = false;
         }
     }

     public void getGeneralTelemetry() {
         telemetry.addData("IncrementLevel", increment);
         telemetry.addData("speed adjust",  "%.2f", speedAdjust);
         telemetry.addData("Arm target position:", "%d", currentHeight);
         //telemetry.addData("Skystone", checkSkystone());
        // telemetry.addData("Left red value", leftColorSensor.red());
         //telemetry.addData("Right red value", rightColorSensor.red());
         telemetry.update();
     }

     public void incrementDown() {
         if (gamepad2.left_bumper && !gamepad2.dpad_up) {
             if (!incrementDown) {
                 increment -= 1;
                 incrementDown = true;
                 if (increment < 1) {
                     increment = 1;
                 }
             }
         } else {
             incrementDown = false;
         }
     }

     public void incrementUp() {
         if (gamepad2.right_bumper && !gamepad2.dpad_up) {
             if (!incrementUp) {
                 increment += 1;
                 incrementUp = true;
                 if (increment >= 5) {
                     increment = 5;
                 }
             }
         } else {
             incrementUp = false;
         }

     }

     public void drive() {
         double yAxis;
         double xAxis;
         double turn;

         yAxis = gamepad1.left_stick_y;
         xAxis  = gamepad1.left_stick_x;
         turn = gamepad1.right_stick_x;
         //DON'T CHANGE!!!!!
         blm.setPower((yAxis + xAxis - turn) * (-speedAdjust/10));
         flm.setPower((yAxis - xAxis - turn) * (-speedAdjust/10));
         brm.setPower((yAxis - xAxis + turn) * (-speedAdjust/10));
         frm.setPower((yAxis + xAxis + turn) * (-speedAdjust/10));
     }

   /*  public void grabberControl(){
         if (gamepad1.left_trigger == 1 && grabberbuttonpushed == false){
             Grabber.setPosition(0);
             grabberbuttonpushed = true;
         } else {
             grabberbuttonpushed = false;
         }

         if (gamepad1.right_trigger == 1 && grabberbuttonnotPushed == false){
             Grabber.setPosition(1);
             grabberbuttonnotPushed = true;
         }else {
             grabberbuttonnotPushed = false;
         }


     }
*/

     public void driveAuto(double straight, double strafe, double turn, double speed, int distance_cm) {
         double distance_encoder = (int)((distance_cm * 383.6) / 31.4);

         resetDrive();

         blm.setPower((+straight - strafe + turn) * (-speed));
         flm.setPower((-straight - strafe - turn) * (-speed));
         brm.setPower((+straight + strafe - turn) * (-speed));
         frm.setPower((-straight + strafe + turn) * (-speed));
         lessThanEqualTelemetry(distance_encoder);
         motorSpeedRelay();
         stopDriveMotors();
     }

     public void lessThanEqualTelemetry (double target){
         while (opModeIsActive() && distanceDone(target)) {
                sleep(CYCLE_MS);
                idle();
         }

     }

     public void resetDrive() {
        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     }

     public void resetDriveWithoutEncoder(){
         flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         flm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         frm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     }

     public void runWithoutEncoders() {
         flm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         frm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     }

     public void stopDriveMotors(){
      flm.setPower(0);
      blm.setPower(0);
      frm.setPower(0);
      brm.setPower(0);
     }

     public void setDrivePower(double DrivePower) {
       flm.setPower(DrivePower);
       blm.setPower(DrivePower);
       frm.setPower(DrivePower);
       brm.setPower(DrivePower);
     }

     public void runToPosition() {
       flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       blm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       brm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     }

     public void waitForMotorsAndRelayTelm() {
       while (opModeIsActive() && flm.isBusy() && blm.isBusy() && frm.isBusy() && brm.isBusy()) {
         telemetry.addData("FL", flm.getCurrentPosition());
         telemetry.addData("BL", blm.getCurrentPosition());
         telemetry.addData("FR", frm.getCurrentPosition());
         telemetry.addData("BR", brm.getCurrentPosition());
         telemetry.update();
         //idle();
       }
     }

     public void motorSpeedRelay() {
         telemetry.addData("FL speed", flm.getPower());
         telemetry.addData("BL speed", blm.getPower());
         telemetry.addData("FR speed", frm.getPower());
         telemetry.addData("BR speed", brm.getPower());
         telemetry.update();
     }

   public void setDriveY(int y){
       y = (int)((y*1120)/31.4);
       flm.setTargetPosition(y);
       blm.setTargetPosition(y);
       frm.setTargetPosition(y);
       brm.setTargetPosition(y);
     }
   public void setDriveX(int x){
       x = (int)((x*1120)/31.4);
       flm.setTargetPosition(x);
       blm.setTargetPosition(-x);
       frm.setTargetPosition(-x);
       brm.setTargetPosition(x);
     }
   public void setDriveRotate(int r) {
       flm.setTargetPosition(r);
       blm.setTargetPosition(r);
       frm.setTargetPosition(-r);
       brm.setTargetPosition(-r);
     }

     public void DriveX(int distance,double speed){
       setDriveX(distance);
       runToPosition();
       setDrivePower(speed);
       waitForMotorsAndRelayTelm();
       setDrivePower(0);
       resetDrive();
     }

     public void DriveY(int distance,double speed){
       setDriveY(distance);
       runToPosition();
       setDrivePower(speed);
       waitForMotorsAndRelayTelm();
       setDrivePower(0);
       resetDrive();
     }

      public void armsResetAndRun() {
         leftarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         rightarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         leftarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     }

     public void armsReset() {
         boolean armszero = leftarm.getCurrentPosition() == 0 && rightarm.getCurrentPosition() == 0;

         if (armszero) {
             leftarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             rightarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         }
     }

     public void armsRunToPosition() {
         leftarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         rightarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     }



     public void armsPower(double power) {
         leftarm.setPower(power);
         rightarm.setPower(power);
     }

     /* public void arms() {
         boolean armtoplimit = leftarm.getCurrentPosition() <= 7000 && rightarm.getCurrentPosition() <= 7000;
         boolean armlowlimit = leftarm.getCurrentPosition() >= 0 && rightarm.getCurrentPosition() >= 0;

         if (gamepad2.a && armtoplimit)  {
             leftarm.setPower(1);
             rightarm.setPower(1);
         } else if (gamepad2.b && armlowlimit) {
             leftarm.setPower(-1);
             rightarm.setPower(-1);
         } else {
             leftarm.setPower(0);
             rightarm.setPower(0);
         }
         telemetry.addData("LeftArmTicks", leftarm.getCurrentPosition());
         telemetry.addData("RightArmTicks", rightarm.getCurrentPosition());
         telemetry.update();
     } */


     /* public void incrementArms() {
         boolean armtoplimit = leftarm.getCurrentPosition() <= 7000 && rightarm.getCurrentPosition() <= 7000;
         boolean armlowlimit = leftarm.getCurrentPosition() >= 0 && rightarm.getCurrentPosition() >= 0;

         if (gamepad2.a && armtoplimit) {
             leftarm.setTargetPosition((blockHeight * (increment - 1) + 775));
             rightarm.setTargetPosition((blockHeight * (increment - 1) + 775));
             armsPower(1);
             armsRunToPosition();
         } else if (gamepad2.b && armlowlimit) {
             leftarm.setTargetPosition(0);
             rightarm.setTargetPosition(0);
             armsPower(-1);
             armsRunToPosition();
             armsReset();
         } else {
             // armsPower(0);
         }

     } */

     /* public void arms() {
         boolean armtoplimit = leftarm.getCurrentPosition() <= 9992 && rightarm.getCurrentPosition() <= 9992 ;
         boolean armlowlimit = leftarm.getCurrentPosition() >= 0 && rightarm.getCurrentPosition() >= 0;

         if (gamepad2.a && armtoplimit && (increment >= 2))  {
             leftarm.setTargetPosition(( blockHeight*(increment-1)+200 ));//267
             rightarm.setTargetPosition((blockHeight*(increment-1)+200 ));
             armsPower(.5);
             armsRunToPosition();
         } else if (gamepad2.b && armlowlimit) {
             leftarm.setTargetPosition(0);
             rightarm.setTargetPosition(0);
             armsPower(-.5);
             armsRunToPosition();
             armsReset();
         } else if (gamepad2.a && armtoplimit && (increment == 1)) {
             leftarm.setTargetPosition(571);
             rightarm.setTargetPosition(571);
             armsPower(.5);
             armsRunToPosition();
         }
     } */

     public void arms() {

         int maxBlock = 5;
         int blockHeight = 520;
         int fudgeFactor = 60 * (increment - 1);
         currentHeight = (blockHeight*increment) + fudgeFactor;
         int armTopLimit = (blockHeight*(maxBlock + 1)) + (60 * maxBlock);

         if (currentHeight >= armTopLimit) {
             telemetry.addData("Danger", "will robinson");
         } else if (gamepad2.a && !armPress) {
             leftarm.setTargetPosition((int) currentHeight);
             rightarm.setTargetPosition((int) currentHeight);
             armsPower(.6);
             armsRunToPosition();
             increment++;
             armPress = true;
         } else if (gamepad2.b) {
             leftarm.setTargetPosition(0);
             rightarm.setTargetPosition(0);
             armsPower(-.5);
             armsRunToPosition();
             armsReset();
             armPress = false;
         }
     }

     public void suction() {
         if (gamepad2.left_trigger == 1) {
             LF.setPower(1);
             LB.setPower(-1);
             RF.setPower(1);
             RB.setPower(-1);
             //LI.setPower(1);
            // RI.setPower(-1);
             LEFTSUCTIONMOTOR.setPower(-1);
             RIGHTSUCTIONMOTOR.setPower(1);
             //driveAuto(1,0,0,1,30);
         } else if (gamepad2.right_trigger == 1) {
             LF.setPower(-1);
             LB.setPower(1);
             RF.setPower(-1);
             RB.setPower(1);
             //LI.setPower(-1);
             //RI.setPower(1);
             LEFTSUCTIONMOTOR.setPower(1);
             RIGHTSUCTIONMOTOR.setPower(-1);
             //driveAuto(1,0,0,
         } else {
             LF.setPower(0);
             LB.setPower(0);
             RF.setPower(0);
             RB.setPower(0);
             LEFTSUCTIONMOTOR.setPower(0);
             RIGHTSUCTIONMOTOR.setPower(0);

         }
     }

     public void pushOut() {
         if (gamepad1.left_trigger == 1) {
             LI.setPower(-0.75);
             RI.setPower(0.75);
         }
         else {
             LI.setPower(0);
             RI.setPower(0);
         }
     }

     public void pushOutBackwards(){
         LI.setPower(0.75);
         RI.setPower(-0.75);
         sleep(500);
         driveAuto(-1,0,0,.3,-15);
         runWithoutEncoders();
     }

     /*public void lift() {
          if (gamepad2.x) {
           lift.setPower(0.5);
           telemetry.addData("Ticks", lift.getCurrentPosition());
           telemetry.update();
         } else if (gamepad2.y) {
            lift.setPower(-0.5);
            telemetry.addData("Ticks", lift.getCurrentPosition());
            telemetry.update();
         }  else {
            lift.setPower(0);
         }
     }*/

     public void blockStrafeLeft() {
       flm.setPower(.2);
       blm.setPower(-.4);
       frm.setPower(-.4);
       brm.setPower(.2);
     }

     public void blockStrafeRight() {
       flm.setPower(-.6);
       blm.setPower(.3);
       frm.setPower(.3);
       brm.setPower(-.6);
     }

     public void lineUpBlock() {
         if (gamepad2.dpad_up && (leftTouchSensor.isPressed() || rightTouchSensor.isPressed())){
             setDrivePower(-.4);
             while (gamepad2.left_bumper && opModeIsActive()) {
                 if (leftTouchSensor.isPressed() && rightTouchSensor.isPressed()) {
                     blockStrafeRight();
                     sleep(100);
                 } else {
                     blm.setPower(0);
                     flm.setPower(0);
                     brm.setPower(0);
                     frm.setPower(0);
                     //strafeRightAuto();
                     sleep(CYCLE_MS);
                     pushOutBackwards();
                     break;
                 }
             }

             while (gamepad2.right_bumper && opModeIsActive()) {
                 if (leftTouchSensor.isPressed() && rightTouchSensor.isPressed()) {
                     blockStrafeLeft();
                     sleep(100);
                 } else {
                     blm.setPower(0);
                     flm.setPower(0);
                     brm.setPower(0);
                     frm.setPower(0);
                     //strafeLeftAuto();
                     sleep(CYCLE_MS);
                     //driveAuto(1,0,0,1,2);
                     pushOutBackwards();
                     break;
                 }
             }
         }
         stopDriveMotors();
     }

     public void strafeLeftAuto() {
         driveAuto(0,1,0,.5,7);
     }

     public void strafeRightAuto() {
         driveAuto(0,-1,0,.5,-13);
     }


     public void capstoneGrabber() {
         if (gamepad1.a) {
             CapGrab.setPosition(0);
         } else if (gamepad1.b) {
             CapGrab.setPosition(-0.4);
         } else {
             CapGrab.setPosition(0.95);
         }
     }

     public boolean distanceDone(double target){
         if ((abs(flm.getCurrentPosition()) <= abs(target)) && (abs(blm.getCurrentPosition()) <= abs(target))
                                                            && (abs(frm.getCurrentPosition()) <= abs(target))
                                                            && (abs(brm.getCurrentPosition()) <= abs(target))) {
              return true;
         } else {
              return false;
         }

     }

     public void distanceDrive(double distance_cm) {
         double target = (int)((distance_cm * 1120) / 31.4);
         resetDrive();
         while (opModeIsActive() && (/*sensorRange.getDistance(DistanceUnit.CM) > 4 ||*/ distanceDone(target))) {
             //telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
             telemetry.addData("distance met", "%s", distanceDone(target));
             telemetry.update();
             flm.setPower(-1);
             blm.setPower(-1);
             brm.setPower(-1);
             frm.setPower(-1);
         }
         stopDriveMotors();
     }

     public void leftActuatorTest() {
         if (gamepad1.a) {
             leftactuator.setPosition(0.65);
             telemetry.addData("Active?", "Yes");
         } else {
             telemetry.addData("Active?", "No");
         }
         telemetry.update();
     }

     public void driveMotorTest() {
         if (gamepad1.dpad_up) {
             flm.setPower(1);
         } else if (gamepad1.dpad_down) {
             frm.setPower(1);
         } else if (gamepad1.dpad_left) {
             blm.setPower(1);
         } else if (gamepad1.dpad_right) {
             brm.setPower(1);
         } else {
             flm.setPower(0);
             blm.setPower(0);
             frm.setPower(0);
             brm.setPower(0);
         }
     }



     public boolean checkSkystone() {
         //100 is the deciding constant that determines either stone or Skystone
         if (abs(leftColorSensor.red()-rightColorSensor.red())>100){
             return true;
         } else {
             return false;
         }
     }

     //public void

     /*public void touch() {
         if (leftTouchSensor.isPressed() && gamepad2.left_bumper) {
           driveAuto(0,-1,0,1,-100);
         } else if(rightTouchSensor.isPressed() && gamepad2.right_bumper) {
           driveAuto(0,1,0,1,100);
         }
     }*/

     /*public void testServo() {
         if (gamepad1.a) {
             robot.myServo.setPosition(1);
             sleep(500);
             motorForward(robot.rightFrontDrive, 1000, 0.5);
         } else if (gamepad1.b) {
             robot.myServo.setPosition(0);
             sleep(500);
             motorBackward(robot.rightFrontDrive, 1000, 0.5);
         }
     }*/
 }

   

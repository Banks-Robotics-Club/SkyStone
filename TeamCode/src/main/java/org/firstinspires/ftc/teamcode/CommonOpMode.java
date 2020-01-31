 package org.firstinspires.ftc.teamcode;


 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.hardware.DigitalChannel;
 import com.qualcomm.robotcore.hardware.DistanceSensor;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

 import static java.lang.Math.abs;

 public abstract class CommonOpMode extends LinearOpMode {

     private ElapsedTime     runtime = new ElapsedTime();
     int blockHeight = 925;
     double speedAdjust = 10;
     boolean speedUp = false;
     boolean slowDown = false;
     int increment = 1;
     boolean incrementUp = false;
     boolean incrementDown = false;
     boolean grabberbuttonpushed = false;
     boolean grabberbuttonnotPushed = false;

     static boolean RED = true;
     static boolean BLUE = false;
     static boolean LEFT = true;
     static boolean RIGHT = false;
     boolean alliance = BLUE;
     boolean position = RIGHT;

     boolean leftPress = false;

     DcMotor flm;
     DcMotor blm;
     DcMotor frm;
     DcMotor brm;
     public DcMotor leftarm;
     public DcMotor rightarm;
     public DcMotor lift;
     public CRServo LI;
     public CRServo RI;
     public Servo Grabber;
     public Servo CapGrab;
     private DistanceSensor sensorRange;
     double frPower = 0;
     double flPower = 0;
     double blPower = 0;
     double brPower = 0;
     double clockwiseRotation = 0;
     double counterclockwiseRotation = 0;
     static final int    CYCLE_MS    =   500;

     public DigitalChannel leftTouchSensor;
     public DigitalChannel rightTouchSensor;
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

        telemetry.addData("Initialized Correctly!", "");
        telemetry.update();

     }

     public void initHardware() {
     flm = hardwareMap.dcMotor.get("frm");
     blm = hardwareMap.dcMotor.get("blm");
     frm = hardwareMap.dcMotor.get("flm");
     brm = hardwareMap.dcMotor.get("brm");
     Grabber = hardwareMap.servo.get("Grabber");
     CapGrab = hardwareMap.servo.get("CapGrab");
     leftarm = hardwareMap.dcMotor.get("leftarm");
     rightarm = hardwareMap.dcMotor.get("rightarm");
     lift = hardwareMap.dcMotor.get("lift");
     LI = hardwareMap.crservo.get("LI");
     RI = hardwareMap.crservo.get("RI");
     sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
     leftTouchSensor = hardwareMap.get(DigitalChannel.class, "LT");
     rightTouchSensor = hardwareMap.get(DigitalChannel.class, "RT");
     distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");
     flm.setDirection(DcMotorSimple.Direction.REVERSE);
     blm.setDirection(DcMotorSimple.Direction.REVERSE);
     frm.setDirection(DcMotorSimple.Direction.FORWARD);
     brm.setDirection(DcMotorSimple.Direction.FORWARD);
     leftarm.setDirection(DcMotorSimple.Direction.FORWARD);
     rightarm.setDirection(DcMotorSimple.Direction.REVERSE);
     lift.setDirection(DcMotorSimple.Direction.FORWARD);
     lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     LI.setDirection(DcMotorSimple.Direction.REVERSE);
     RI.setDirection(DcMotorSimple.Direction.FORWARD);
     leftTouchSensor.setMode(DigitalChannel.Mode.INPUT);
     rightTouchSensor.setMode(DigitalChannel.Mode.INPUT);
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
                 speedAdjust -= 5;
                 slowDown = true;
                 if (speedAdjust < 5) {
                     speedAdjust = 5;
                 }
             }
         } else {
             slowDown = false;
         }
     }

     private void speedUp() {
         if (gamepad1.dpad_right) {
             if (!speedUp) {
                 speedAdjust += 5;
                 speedUp = true;
                 if (speedAdjust > 10) {
                     speedAdjust = 10;
                 }
             }
         } else {
             speedUp = false;
         }
     }

     public void getArmTelemetry() {
         telemetry.addData("IncrementLevel", increment);
         telemetry.addData("speed adjust",  "%.2f", speedAdjust);
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
         turn = gamepad1.left_stick_x;
         xAxis = gamepad1.right_stick_x;

         blm.setPower((yAxis + xAxis - turn) * (-speedAdjust/10));
         flm.setPower((yAxis - xAxis - turn) * (-speedAdjust/10));
         brm.setPower((yAxis - xAxis + turn) * (-speedAdjust/10));
         frm.setPower((yAxis + xAxis + turn) * (-speedAdjust/10));
     }

     public void grabberControl(){
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


     public void driveAuto(double straight, double strafe, double turn, double speed, int distance_cm) {
         double distance_encoder = (int)((distance_cm * 1120) / 31.4);

         resetDrive();

         blm.setPower((straight + strafe - turn) * (-speed));
         flm.setPower((straight - strafe - turn) * (-speed));
         brm.setPower((straight - strafe + turn) * (-speed));
         frm.setPower((straight + strafe + turn) * (-speed));
         lessThanEqualTelemetry(distance_encoder);
         stopDriveMotors();
     }

     public void lessThanEqualTelemetry (double target){
         while (opModeIsActive() && distanceDone(target)) {
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

     public void armsPower(float power) {
         leftarm.setPower(-power);
         rightarm.setPower(power);
     }

     public void arms() {
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
         /*telemetry.addData("LeftArmTicks", leftarm.getCurrentPosition());
         telemetry.addData("RightArmTicks", rightarm.getCurrentPosition());
         telemetry.update();*/
     }

     public void incrementArms() {
         boolean armtoplimit = leftarm.getCurrentPosition() <= 7000 && rightarm.getCurrentPosition() <= 7000;
         boolean armlowlimit = leftarm.getCurrentPosition() >= 0 && rightarm.getCurrentPosition() >= 0;

         if (gamepad2.a && armtoplimit)  {
             leftarm.setTargetPosition((blockHeight*(increment-1)+775));
             rightarm.setTargetPosition((blockHeight*(increment-1)+775));
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

     }

     public void suction() {
         if (gamepad2.left_trigger == 1) {
             LI.setPower(1);
             RI.setPower(1);
             //driveAuto(1,0,0,1,30);
         } else if (gamepad2.right_trigger == 1) {
             LI.setPower(-1);
             RI.setPower(-1);
         } else {
             LI.setPower(0);
             RI.setPower(0);
         }
     }

     public void suctionBackwards(){
         LI.setPower(0.75);
         RI.setPower(0.75);
         driveAuto(-1,0,0,1,-20);
         runWithoutEncoders();
     }

     public void lift() {
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
     }

     public void blockStrafeLeft() {
       flm.setPower(.2);
       blm.setPower(-.5);
       frm.setPower(-.5);
       brm.setPower(.2);
     }

     public void blockStrafeRight() {
       flm.setPower(-.5);
       blm.setPower(.2);
       frm.setPower(.2);
       brm.setPower(-.5);
     }

     public void lineUpBlock() {
         if (gamepad2.dpad_up && (!leftTouchSensor.getState() || !rightTouchSensor.getState())){
             setDrivePower(-.3);
             while (gamepad2.left_bumper && opModeIsActive()) {
                 if (!leftTouchSensor.getState() && !rightTouchSensor.getState()) {
                     blockStrafeRight();


                 } else {
                     blm.setPower(0);
                     flm.setPower(0);
                     brm.setPower(0);
                     frm.setPower(0);
                     strafeRightAuto();
                     sleep(CYCLE_MS);
                     //idle();
                     suctionBackwards();
                     break;
                 }
             }

             while (gamepad2.right_bumper && opModeIsActive()) {
                 if (!leftTouchSensor.getState() && !rightTouchSensor.getState()) {
                     blockStrafeLeft();

                 } else {
                     blm.setPower(0);
                     flm.setPower(0);
                     brm.setPower(0);
                     frm.setPower(0);
                     strafeLeftAuto();
                     //sleep(CYCLE_MS);
                     //idle();
                     //driveAuto(1,0,0,1,2);
                     suctionBackwards();
                     break;
                 }
             }
         }
         stopDriveMotors();
     }

     public void strafeLeftAuto(){
         driveAuto(0,1,0,.5,7);
     }

     public void strafeRightAuto(){
         driveAuto(0,-1,0,.5,-8);
     }

     public void capstoneGrabber() {
         if (gamepad1.a) {
             CapGrab.setPosition(0.95);
         } else if (gamepad1.b) {
             CapGrab.setPosition(0.4);
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
             telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
             telemetry.addData("distance met", "%s", distanceDone(target));
             telemetry.update();
             flm.setPower(-1);
             blm.setPower(-1);
             brm.setPower(-1);
             frm.setPower(-1);
         }
         stopDriveMotors();
     }

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

   

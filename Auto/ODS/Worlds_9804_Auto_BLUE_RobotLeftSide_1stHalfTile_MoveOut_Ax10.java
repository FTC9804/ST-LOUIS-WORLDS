package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


/*
 * FTC Team 9804 Bomb Squad -- Autonomous
 * Made by the programmers of FTC Team 9804 Bomb Squad

 * Ax00 4-25-16 at 6:45 pm Etienne -- made Ax00 for start with left side of robot anywhere within the first 1/2 tile
 * Ax


 * ~~~~SETUP_(TEETH 1-5)~~~~
 * (1) Left side of the robot can be placed between first one half tile from the mountain perpendicular to the wall


 * ~~~~~~~~~MOVEMENT Ax41 (BLUE!!!)~~~~~~~~~
 * -All the steps correspond to actual steps in the op mode
 * -Continue using setup version Tx21
 * (0) Disable all servo controllers so that they are not draining energy while the robot is powered
 * (1) Drive straight backwards from wall at a 0 degree angle. (When initializing the gyro, 0 degrees is set)
 * (2) spin move clockwise 45º
 * (3) drive straight backwards high speed for 72 inches
 * (4) drive until white line is seen at medium-low speed
 * (5) overshoot by 6.8 inches
 * (6) spin move clockwise 45º (global position is now -90º)
 * (7) enter loop for checking if we are next to white line; exit when time runs out or we are no longer next to line
 * (7.1) drive forwards 4 inches (away from the beacon)
 * (7.2) spin counter clockwise until line is seen or we reach a maximum angle
 * (7.3) spin clockwise back to global heading of -90º
 * ***NOW THE ROBOT IS SLIGHTLY BEYOND THE TIP OF THE WHITE LINE*** (facing the beacon)
 * (8) drive straight backwards 4 inches
 * (9) spin move counter clockwise until white line is detected, back to earlier position
 * (10) ACORN proportional line follow for 15 inches
 * (10.5) enable the servo controller that hosts the shelterDrop servo
 * (11) score shelter drop
 * (12) code complete!
 *
 * NOTES:
 *      -> This auto program uses a variable called "CHECK_FOR_WHITE_LINE_FORWARDS_DISTANCE."
 *      -> This auto program also uses the idle wheel for accurate encoder measurements to prevent backlash
 *      -> This means that this auto program uses CHECK_FOR_WHITE_LINE_FORWARDS_DISTANCE for the forwards value when checking
 *          and the value for going backwards once we exit the loop. In this auto, THE SECOND DISTANCE IS NOT GREATER!!!!
 *
 * GENERAL RULE:
 * FWD: leftPower = midPower - driveSteering;
 * BWD: leftPower = midPower + drive Steering
 * CCW: positive
 * CW: negative
 * Heading = ABSOLUTE heading of the robot on the field
 * Distance = INCREMENTAL distance of the robot on the field
 *
 * referencing floorODS in this program call values from ods1, which is the ODS on the left side of the robot, used when blue
 *
 * ACORN = adjusted center of rotation navigation; we use this for our proportional line follow because our ODS sensors are off centered
 *
 *
 * +++++++++++++++++++++++++++++++Configuration Info+++++++++++++++++++++++++++++++
 *
 * SN           COLOR           NAME                    PORT            CONFIG. NAME
 * UVQF         Purple          spin                    1               m8
 * UVQF         Purple          arms                    2               m7
 * XTJI         Orange          left winch              1               m4
 * XTJI         Orange          right winch             2               m3
 * VUTK         Red             driveRightFront         1               m2
 * VUTK         Red             driveRightBack          2               m1
 * VUTK         Red             right encoder front     1
 * VUTK         Red             right encoder back      2
 * VF7F         Green           driveLeftFront          1               m6
 * VF7F         Green           driveLeftBack           2               m5
 * VF7F         Green           left encoder front      1
 * VF7F         Green           right encoder back      2
 * VSI1         White           hookPoles               3               s3
 * VSI1         White           grabRight               4               s2
 * VSI1         White           box                     5               s4
 * VSI1         White           grabLeft                6               s1
 * VCT7         Pink            ziplineBar              5               s8
 * VCT7         Pink            shelterDrop             6               s6
 * VCT7         Pink            windowWiperL            1               s5
 * VCT7         Pink            windowWiperR            2               s7
 * VCT7         Pink            cliffCamL               3               s9
 * VCT7         Pink            cliffCamR               4               s10
 * VCFP         Yellow          gyro                    I2C5            gyro
 * VCFP         Yellow          led extend              DO7             led1
 * VCFP         Yellow          led retract             DO1             led2
 * VCFP         Yellow          magnet extend           DO0             mag1
 * VCFP         Yellow          magnet retract          DO2             mag2
 * VCFP         Yellow           ODS Right            ***               ods2
 * VCFP         Yellow           ODS Left             ***               ods1
 * Blue            Power Distro <-> Phones
 *
 * NOTE ABOUT THE CODE:
 * To prevent catastrophic errors when the code is running,
 * we include a stop motors and a wait one full hardware cycle after EVERY hardware command
 *
 */


public class Worlds_9804_Auto_BLUE_RobotLeftSide_1stHalfTile_MoveOut_Ax10 extends LinearOpMode {


    /**
     * SERVO CONTROLLERS
     */
    ServoController servoControllerPink, servoControllerWhite;


    /**
     * DRIVE MOTORS
     **/
    DcMotor driveLeftBack;               // there are 2 motors functioning in each tread and the tread
    DcMotor driveLeftFront;              //requires that the leading or front motor is powered at 95% of what
    DcMotor driveRightBack;              // the Back or trailing motor is powered. Further, on the right motors
    DcMotor driveRightFront;             // are reversed because the right side goes forward with positive values.

    /**
     * SPINNER
     **/
    //The spinner is out method for picking blocks up and works in accordance with the hopper so that the hopper
    //collects blocks in the right direction depending on which team is selected when the spinner is running.
    DcMotor spin;


    /**
     * CHURRO GRABBERS
     **/
    //servos to lock in place on ramp
    //These servos deploy downward and lock themselves on the metal churros so that the robot is stable and
    //secure when scoring blocks.
    Servo grabLeft;                       // Standard servos
    Servo grabRight;

    /**
     * HOPPER SERVO
     **/
    //servo to score blocks in the goal
    //This hopper is finally the final product of many iterations. It uses now black elastic belts to move the
    //blocks that are in out hopper either right or left. There are TWO INDEPENDENT hoppers and they are NEVER
    //both on the robot. One is for the RED TEAM and the other for the BLUE team
    Servo box;

    /**
     * SHELTER DROP SERVO
     */
    //servo for dropping the climbers in the shelter drop
    //This mechanism is used in the AUTONOMOUS and TELEOP segments and is attached to the top of our robot.
    //It carries two climbers and only has the movement of deploy and store.
    Servo shelterDrop;                    //CR servo

    /**
     * ZIPLINE BAR
     */
    //The zipline bar is a different kind of mechanism where two different micro servos get the same value when pressed.
    //The servos are positioned one on each side and when "ziplineRelease" is set, the bar deploys.
    Servo ziplineBar;                     //Micro Servo

    /**
     * HOOK POLES
     */
    //the hook poles are the poles which are connected to the arms which that carry the hooks. The hook poles are
    //on one servo and move the hooks closer to the top bar.
    Servo hookPoles;                      //CR Servo

    /**
     * WINDOW WIPER SERVOS
     */
    //servo for debris sweeping away
    //The window wiper servo is used in both AUTO and TELE-OP for when the robot is moving in reverse.
    //Our robot cannot move over blocks very efficiently and, therefore, we try to keep the blocks away
    //from us. With the spinner that collects blocks in the front and the shields in the front, there is no
    //worry for the front. On the back, we rely on the window wiper which wipes all the blocks and balls away
    //from our path using a standard servo
    Servo windowWiperL;                 //Standard Servo
    Servo windowWiperR;

    /**
     * CLIFF CAM SERVOS
     */
    Servo cliffCamL;                    //initialize to 1
    Servo cliffCamR;                    //initialize to 0

    /**
     * GYRO AND ODS DECLARATION
     */

    ModernRoboticsI2cGyro gyro;

    OpticalDistanceSensor floorODS;


    //%%&&~~~~~~~~~VARIABLE DECLARATIONS~~~~~~~~~&&%%//

    //variables for auto

    boolean stillAtWhiteLine = true;
    static double CHECK_TO_WHITE_LINE_DELTA = 20;
    static double ACORN_LINE_FOLLOW_DISTANCE = 14;
    static double CHECK_FOR_WHITE_LINE_FORWARDS_DISTANCE = 2.5;

    boolean runMe = true;                   //boolean for exiting code while loop

    int targetHeading;                      //target heading the gyro will go to
    double driveGainODS = 0.005;            //the gain we use for the ODS line follow
    double leftPower;                       //power for drive motors on the left side
    double rightPower;                      //power for drive motors on the right side
    int currentHeading = 0;                 //This is a signed value, CCW is positive
    int headingError;                       //the error in the heading between the target and current values
    double driveSteering;                   //the variable for proportional control
    double currentDistance;                 //the calculated distance we have travelled
    int currentEncDeltaCountLeft;           //the change (delta) for the left encoder
    int currentEncDeltaCountRight;          //the change (delta) for the right encoder


    double targetDistance;                      //magnitude in distance
    final int encoderCountsPerRotation = 1440;  //AndyMark encoders use 1120 counts per revolution
    final double diameter = 2.583;              //effective diameter of drive pulley and tread b/c of sinking into foam mat (measures 3" dia)
    double circumference = diameter * 3.14159;  //Circumference = PI * Diameter
    double rotations;                           //desired rotations
    int targetEncoderCounts;                    //calculated target encoder counts
    int EncErrorLeft;                           //the error for the left encoder
    int telemetryVariable;                      // a NULL telemetry variable designed to diplay text only
    int initialEncCountLeft;                    //initial encoder counts of the left encoder
    int initialEncCountRight;                   //initial encoder counts of the right encoder


    //ODS VARIABLES
    int rawDetectedLight;                       //ODS Light detection value
    static int ODS_THRESHOLD = 570;             //ODS Light threshold ****TESTED*****
    boolean lineDetected = false;               //boolean to leave do-while loop in methods
    static int DESIREDLINEFOLLOWNUMBER = 350;   //the number for perfect line follow along the edge of the line, NEEDS TO BE TESTED
    double proportionalODSError;

    //servo variables
    double grabLeftUp = 0;                      //0 is max CCW (UP on left side)
    double grabRightUp = 1.0;                   //1 is max CW (UP on right side)
    double sweepOpenedL = 0.75;
    double sweepClosedL = 0.1;
    double sweepPositionL = sweepClosedL;
    double sweepOpenedR = 0.25;
    double sweepClosedR = 0.91;
    double sweepPositionR = sweepClosedR;
    double cliffCamLClosed = 1;
    double cliffCamRClosed = 0;
    double boxPosition = 0.5;
    double hookPolesInitialize = 0.505;
    double hookPolesPosition = hookPolesInitialize;
    double shelterDropInitialize = 0.5;
    double shelterDropRelease = 0.85; //when red, use 0.15
    double shelterDropRetract = 0.2; //when red use 0.8
    double shelterDropPosition = shelterDropInitialize;
    double ziplineBarInitialize = 0;
    double ziplineBarPosition = ziplineBarInitialize;
    double shelterDropTime = 3;

    //USE CONFIGURATION FILE 'JABBED' ON BOTH MAIN AND B PHONES

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    @Override
    public void runOpMode() throws InterruptedException {

        //gives name of drive motors
        driveLeftBack = hardwareMap.dcMotor.get("m5");
        driveLeftFront = hardwareMap.dcMotor.get("m6");
        driveRightBack = hardwareMap.dcMotor.get("m1");
        driveRightFront = hardwareMap.dcMotor.get("m2");
        driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);

        //The encoders on the robot are present on an idler wheel within the tread to get the actual distance
        //and not simply the amount of rotations on the motor. The motor clicks can not be used to calculate
        //a true distance.
        //Other encoders are found on the actual motors on two of the motors within the tread.
        //Encoders are not currently used during TeleOp operation


        spin = hardwareMap.dcMotor.get("m8");


        //give the configuration names for the servos
        grabLeft = hardwareMap.servo.get("s1");
        grabRight = hardwareMap.servo.get("s2");

        windowWiperL = hardwareMap.servo.get("s5");
        windowWiperR = hardwareMap.servo.get("s7");

        cliffCamL = hardwareMap.servo.get("s9");
        cliffCamR = hardwareMap.servo.get("s10");


        box = hardwareMap.servo.get("s4");

        hookPoles = hardwareMap.servo.get("s3");

        ziplineBar = hardwareMap.servo.get("s8");

        shelterDrop = hardwareMap.servo.get("s6");

        // hardware assignment of two sensors

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        floorODS = hardwareMap.opticalDistanceSensor.get("ods1");

        // servo controllers; can disable and re-enable entire unit
        servoControllerWhite = hardwareMap.servoController.get("servoControllerWhite"); // VSI1
        servoControllerPink = hardwareMap.servoController.get("servoControllerPink");   // VCT7

        //sets initial positions for the servos to activate to
        grabLeft.setPosition(grabLeftUp);
        grabRight.setPosition(grabRightUp);
        windowWiperL.setPosition(sweepPositionL);
        windowWiperR.setPosition(sweepPositionR);
        box.setPosition(boxPosition);
        hookPoles.setPosition(hookPolesPosition);
        ziplineBar.setPosition(ziplineBarPosition);
        shelterDrop.setPosition(shelterDropPosition);
        cliffCamL.setPosition(cliffCamLClosed);
        cliffCamR.setPosition(cliffCamRClosed);

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");    //initialize gyro to be used in code

        OpticalDistanceSensor floorODS = hardwareMap.opticalDistanceSensor.get("ods1");

        hardwareMap.logDevices();

        //calibrate the gyro and set the current heading to zero
        gyro.calibrate();


        //begin match with driver station button
        waitForStart();

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        while (this.opModeIsActive() && runMe) {     //the op mode is active conditional forces the code to stop once the driver station specifies

            /**
             * Each step refers to the steps in the movement from Tx40
             */

            //step 0
            // de-energize the servos when stop button is pressed
            servoControllerWhite.pwmDisable();
            servoControllerPink.pwmDisable();

            //step 1
            driveStraightBackwards(0, 36, 0.8); //36 inches because it is the ROBOT_LEFT_SIDE_1ST_HALF_TILE

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 2
            spinMoveClockwise(-45);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 3
            driveStraightBackwards(-45, 25, 0.9);

            waitOneFullHardwareCycle();

            //step 4
            driveStraightBackwardsUntilWhiteLineIsDetected(-45, 0.6);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 5
            driveStraightBackwards(-45, 6.8, 0.5); //6.8 is the overshoot distance that ***TESTED***

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 6
            spinMoveClockwise(-90);         //now robot is facing beacon with white line underneath

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 7

            this.resetStartTime();

            while (this.opModeIsActive() && this.getRuntime() < 20 && stillAtWhiteLine) {

                windowWiperActivate();

                //sub-step 1
                driveStraightForwards(-90, CHECK_FOR_WHITE_LINE_FORWARDS_DISTANCE, 0.5);

                waitOneFullHardwareCycle();

                stopMotors();

                waitOneFullHardwareCycle();

                //sub-step 2
                //this function will keep turning counter clockwise until it does not see white.
                //Then it will set stillAtWhiteLine = false;
                spinMoveCounterClockwiseToCheckForWhiteLine(CHECK_TO_WHITE_LINE_DELTA);

                waitOneFullHardwareCycle();

                stopMotors();

                waitOneFullHardwareCycle();

                //sub-step 3
                spinMoveClockwise(-90);

                waitOneFullHardwareCycle();

                stopMotors();

                waitOneFullHardwareCycle();

                telemetry.addData("At the end of the line : ", !stillAtWhiteLine); //this will display to true

            }

            //step 8

            driveStraightBackwards(-90, CHECK_FOR_WHITE_LINE_FORWARDS_DISTANCE, 0.5); //THE DISTANCE IS 1.5 GREATER BECAUSE OF BACKLASH

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 9
            spinMoveCounterClockwiseUntilWhiteLineIsDetected();

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 10
            proportionalLineFollowForDistanceWithACORNRightSideOfLineLeftSideSensor(ACORN_LINE_FOLLOW_DISTANCE, 0.5); //distance of 20 needs to be checked

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //10.5

            //enable the servo controller that hosts the shelterDrop servo
            servoControllerPink.pwmEnable();

            //step 11
            scoreShelterDrop(shelterDropTime);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 12

            // de-energize the servos
            servoControllerWhite.pwmDisable();
            servoControllerPink.pwmDisable();

            //step 13

            driveStraightForwards(-90, 60, 0.8);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            telemetry.addData("CODE COMPLETE", telemetryVariable);

            runMe = false;

        }

    } //finish the opmode

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    void stopMotors() {
        /*
         * How to use this method:
         *  Use this method at any point right after motors are running
         *  This will stop the motors to get them ready for the next step in the code
         */
        //set all motor powers to 0 after portions of the code finish running
        driveLeftBack.setPower(0.0);
        driveLeftFront.setPower(0.0);
        driveRightBack.setPower(0.0);
        driveRightFront.setPower(0.0);
        spin.setPower(0);

        telemetry.addData("STOP ALL MOTORS", telemetryVariable);
    }

    void scoreShelterDrop(double shelterScoreTime) {
        /*
         * How to use this method:
         *  Run this method when you wish to use the servo to score in the shelter
         *  Input the time you want the servo to move forwards.
         *  This number will be used for release and retract times
         */
        //resets the start time to be used in the loop
        this.resetStartTime();
        //while loop to run servo while to loop is active
        while (this.getRuntime() < shelterScoreTime && this.opModeIsActive()) {
            shelterDrop.setPosition(shelterDropRelease);
        }
        //sets the shelter drop back to no power
        shelterDrop.setPosition(shelterDropInitialize);
        this.resetStartTime();
        while (this.getRuntime() < shelterScoreTime && this.opModeIsActive()) {
            shelterDrop.setPosition(shelterDropRetract);
        }
        shelterDrop.setPosition(shelterDropInitialize);
        //telemetry for phones about task finished
        telemetry.addData("SCORE CLIMBERS", telemetryVariable);

    }

    void windowWiperActivate() {
        /*
         * How to use this method:
         *  Call on this method when you wish to run the window wiper servo to clear debris from the front of the robot
         */
        //CLEAR DEBRIS WITH WINDOW WIPER SERVO
        windowWiperL.setPosition(sweepOpenedL);
        windowWiperR.setPosition(sweepOpenedR);
        this.resetStartTime();
        while (this.getRuntime() < 1 && this.opModeIsActive()) { //give a short period of time for the hardware to execute the command
            stopMotors();
        }
        windowWiperL.setPosition(sweepClosedL);
        windowWiperR.setPosition(sweepClosedR);
        this.resetStartTime();
        while (this.getRuntime() < 0.5 && this.opModeIsActive()) { //give a short period of time for the hardware to execute the command
            stopMotors();
        }


    }

    //NON ODS DRIVING METHODS

    void spinMoveClockwise(int heading) {
       /*
         * How to use this method:
         *  In the op mode, input the ABSOLUTE heading of the desired position for the robot on the field.
         *  After the code is finished, run the stopMotors method to fully stop all drive and spin motors
         */
        //SPIN MOVE

        driveGainODS = 0.005;       //OK for spin
        targetHeading = heading;    //90º CW (using signed heading) (positive value CCW)

        this.resetStartTime();

        do {
            spin.setPower(1);  // Eject debris while driving, to clear path

            //takes the current heading of the gyro
            currentHeading = gyro.getIntegratedZValue();

            //telemetry to display the current signed heading
            telemetry.addData("current signed heading: ", currentHeading);

            //takes the heading error for the value of our gyro
            headingError = targetHeading - currentHeading;//for CW spin from 0 to -º, error always negative

            //drive steering for proportional control
            driveSteering = headingError * driveGainODS;         //negative value

            //for CW spin, left tread runs forwards
            leftPower = -driveSteering;
            if (leftPower > 1) {
                leftPower = 1;
            }
            if (leftPower < 0.6) {           //avoid zero closing power at low error
                leftPower = 0.6;            //0.1 stalled near target heading
            }


            //for CW spin, right tread runs backwards
            rightPower = driveSteering;
            if (rightPower < -1) {
                rightPower = -1;
            }
            if (rightPower > -0.6) {
                rightPower = -0.6;
            }

            //when spinning CW, left front is leading, left back is trailing
            //right front is trailing, right back is leading
            //trailing gets calculated full power, leading gets 95% of calculated full power
            driveLeftFront.setPower(0.95 * leftPower);
            driveLeftBack.setPower(leftPower);
            driveRightFront.setPower(rightPower);
            driveRightBack.setPower(0.95 * rightPower);

        } while (currentHeading > targetHeading
                && this.getRuntime() < 6 && this.opModeIsActive());
        //spin from 0 towards a more - heading, so loop while 'greater than' the target heading

        telemetry.addData("SPIN CW DONE", telemetryVariable);

    }

    void driveStraightForwards(int heading, double distance, double midPower) {
        /*
         * How to use this method:
         *  Programmer inputs heading, distance, and midpower
         *      Heading = ABSOLUTE heading of the robot on the field
         *      Distance = INCREMENTAL distance that you wish to have the robot run this time
         *      MidPower = the desired midpower that you wish to have the robot run with
         */
        //clear the previous telemetry output on the phones
        telemetry.clearData();

        driveGainODS = 0.005;           //gain used for proportional steering
        targetHeading = heading;        //drive straight ahead, same heading

        targetDistance = distance;      //distance given in the parameters
        rotations = targetDistance / circumference;   //calculate rotations necessary
        targetEncoderCounts = (int) (encoderCountsPerRotation * rotations);    //cast the target encoder counts as an integer

        //grab the current position of the encoders
        initialEncCountLeft = driveLeftBack.getCurrentPosition();
        initialEncCountRight = driveRightBack.getCurrentPosition();

        this.resetStartTime();      //for safety timeout

        //loop until driving distance reached (or safety timeout)
        do {
            spin.setPower(1);  // Eject debris while driving, to clear path

            //gets the encoder delta for the left and right encoders
            currentEncDeltaCountLeft = driveLeftBack.getCurrentPosition() - initialEncCountLeft;
            currentEncDeltaCountRight = driveRightBack.getCurrentPosition() - initialEncCountRight;

            //calculates the error of the left encoder
            EncErrorLeft = targetEncoderCounts - Math.abs(currentEncDeltaCountLeft);

            //prints the left encoder delta to the phones
            telemetry.addData("Left Encoder Delta:", currentEncDeltaCountLeft);

            //calculates the current distance travelled
            currentDistance = (currentEncDeltaCountLeft / encoderCountsPerRotation) * circumference;

            //prints the calculated current distance to the phones
            telemetry.addData("Calculated current distance: ", currentDistance);

            //gets the current heading from the gyro
            currentHeading = gyro.getIntegratedZValue();

            //prints the current signed heading to the phones
            telemetry.addData("current signed heading: ", currentHeading);

            //calculates the heading error
            headingError = targetHeading - currentHeading;      //positive if pointing too far CW

            //calculates the drive steering for proportional control
            driveSteering = headingError * driveGainODS;           //positive if pointing too far CW

            leftPower = midPower - driveSteering;
            if (leftPower > 1) {
                leftPower = 1;
            }
            if (leftPower < 0.2) {
                leftPower = 0.2;
            }


            rightPower = midPower + driveSteering;
            if (rightPower > 1) {
                rightPower = 1;
            }
            if (rightPower < 0.2) {
                rightPower = 0.2;
            }

            //when driving forward, left front is leading, left back is now trailing, same for right
            //trailing gets full power
            driveLeftFront.setPower(0.95 * leftPower);
            driveLeftBack.setPower(leftPower);
            driveRightFront.setPower(0.95 * rightPower);
            driveRightBack.setPower(rightPower);

        } while (EncErrorLeft > 0 && this.getRuntime() < 12 && this.opModeIsActive());

        telemetry.addData("STRAIGHT FORWARDS DONE", telemetryVariable);


    }

    void driveStraightBackwards(int heading, double distance, double midPower) {
        /*
         * How to use this method:
         *  Programmer inputs heading, distance, and midpower
         *      Heading = ABSOLUTE heading of the robot on the field
         *      Distance = INCREMENTAL distance that you wish to have the robot run this time
         *      MidPower = the desired midpower that you wish to have the robot run with
         */
        //DRIVE BACKWARDS DESIRED INCHES

        telemetry.clearData();      //clear all telemetry data before starting

        driveGainODS = 0.005;           //gain for proportional control

        targetHeading = heading;                //drive straight ahead at the initial/default heading

        targetDistance = distance;              //drive straight given inches from parameter

        //math for target encoder counts to travel
        rotations = targetDistance / circumference;
        targetEncoderCounts = (int) (encoderCountsPerRotation * rotations); //casts the target encoder counts as an integer

        //resets start time before starting the drive
        this.resetStartTime();

        //takes the initial position of the encoders to establish a starting point for the distance
        initialEncCountLeft = driveLeftBack.getCurrentPosition();
        initialEncCountRight = driveRightBack.getCurrentPosition();


        do {
            spin.setPower(1);  // Eject debris while driving, to clear path

            currentEncDeltaCountLeft = driveLeftBack.getCurrentPosition() - initialEncCountLeft;         //the current - initial will give the
            currentEncDeltaCountRight = driveRightBack.getCurrentPosition() - initialEncCountRight;      //current distance of the encoders


            EncErrorLeft = targetEncoderCounts - Math.abs(currentEncDeltaCountLeft);                     //the error is the delta between the target counts and current counts


            //telemetry for encoder information
            telemetry.addData("EncErrorLeft = ", EncErrorLeft);
            telemetry.addData("Left Encoder: ", currentEncDeltaCountLeft);

            //telemetry for the distance travelled (IN INCHES)
            currentDistance = (currentEncDeltaCountLeft * circumference) / encoderCountsPerRotation;
            telemetry.addData("Calculated current distance: ", currentDistance);

            // get the Z-axis heading info.
            //this is a signed heading not a basic heading
            currentHeading = gyro.getIntegratedZValue();

            headingError = targetHeading - currentHeading;  //find the error between the headings

            driveSteering = headingError * driveGainODS;       //create the proportion for the steering

            leftPower = midPower + driveSteering;           //adds the drive steering to midpower because we are driving backwards
            if (leftPower > 1.0) {                            //cuts ourselves off at 1, the maximum motor power
                leftPower = 1.0;
            }
            if (leftPower < 0.2) {
                leftPower = 0.2;
            }
            rightPower = midPower - driveSteering;          //subtraction because we are driving backwards
            if (rightPower > 1.0) {
                rightPower = 1.0;
            }
            if (rightPower < 0.2) {
                rightPower = 0.2;
            }
            //when driving backwards, reverse leading and trailing
            //left front is now trailing, left back is now leading
            //trailing gets full power
            driveLeftFront.setPower(-leftPower);
            driveLeftBack.setPower(-.95 * leftPower);       //creates belt tension between the drive pulleys
            driveRightFront.setPower(-rightPower);
            driveRightBack.setPower(-.95 * rightPower);


        }
        while (EncErrorLeft > 0                   //the error is slowly decreasing, so run while greater than 0
                && this.getRuntime() < 12 && this.opModeIsActive());         //safety timeout of 12 seconds


        telemetry.addData("DRIVE STRAIGHT BACKWARDS DONE", telemetryVariable);

    }

    //ODS DRIVING METHODS

    void driveStraightBackwardsUntilWhiteLineIsDetected(int heading, double midPower) {

        /*
         * Purpose of this method:
         *  drive backwards with spinners running until the robot senses a white line
         *
         * How to use this method:
         *  Programmer inputs heading and the midpower for driving
         *       Heading = ABSOLUTE heading of the robot on the field
         *       MidPower = the desired midpower that you wish to have the robot run with
         *
         */

        telemetry.clearData();      //clear all telemetry data before starting

        driveGainODS = 0.005;           //gain for proportional control

        targetHeading = heading;                //drive straight ahead at the initial/default heading


        //resets start time before starting the drive
        this.resetStartTime();


        do {
            spin.setPower(1);  // Eject debris while driving, to clear path

            //get and assign the raw infrared value the sensor detects
            rawDetectedLight = floorODS.getLightDetectedRaw();

            if (rawDetectedLight > ODS_THRESHOLD) {
                lineDetected = true;
            }

            telemetry.addData("ods1 Blue rear left ",  floorODS.getLightDetectedRaw());

            spin.setPower(1);  // Eject debris while driving, to clear path

            // get the Z-axis heading info.
            //this is a signed heading not a basic heading
            currentHeading = gyro.getIntegratedZValue();

            headingError = targetHeading - currentHeading;  //find the error between the headings

            driveSteering = headingError * driveGainODS;       //create the proportion for the steering

            leftPower = midPower + driveSteering;           //adds the drive steering to midpower because we are driving backwards
            if (leftPower > 1.0) {                            //cuts ourselves off at 1, the maximum motor power
                leftPower = 1.0;
            }
            if (leftPower < 0.2) {
                leftPower = 0.2;
            }
            rightPower = midPower - driveSteering;          //subtraction because we are driving backwards
            if (rightPower > 1.0) {
                rightPower = 1.0;
            }
            if (rightPower < 0.2) {
                rightPower = 0.2;
            }
            //when driving backwards, reverse leading and trailing
            //left front is now trailing, left back is now leading
            //trailing gets full power
            driveLeftFront.setPower(-leftPower);
            driveLeftBack.setPower(-.95 * leftPower);       //creates belt tension between the drive pulleys
            driveRightFront.setPower(-rightPower);
            driveRightBack.setPower(-.95 * rightPower);


        }
        while (!lineDetected       //run while the line is not detected
                && this.getRuntime() < 12 && this.opModeIsActive());         //safety timeout of 12 seconds

        lineDetected = false; //reset the variable to be used again

        telemetry.addData("DRIVE STRAIGHT BACKWARDS TO LINE DONE", telemetryVariable);

    }

    void spinMoveCounterClockwiseUntilWhiteLineIsDetected() {

        /*
         * How to use this method:
         *  Run the code and watch the robot spin to the line
         */

        //SPIN MOVE

        driveGainODS = 0.005;       //OK for spin move

        this.resetStartTime();

        do {
            spin.setPower(1);  // Eject debris while driving, to clear path

            //get and assign the raw infrared value the sensor detects
            rawDetectedLight = floorODS.getLightDetectedRaw();

            if (rawDetectedLight > ODS_THRESHOLD) {
                lineDetected = true;
            }

            //takes the current heading of the gyro
            currentHeading = gyro.getIntegratedZValue();

            //telemetry to print the current signed heading
            telemetry.addData("current signed heading: ", currentHeading);

            //for CCW spin, left tread runs backwards
            leftPower = -.7;

            //for CCW spin, right tread runs forwards
            rightPower = .7;


            //when spinning CCW, left front is trailing, left back is leading
            //right front is leading, right back is trailing
            //trailing gets calculated full power, leading gets 95% of calculated full  power

            driveLeftFront.setPower(leftPower);
            driveLeftBack.setPower(0.95 * leftPower);
            driveRightFront.setPower(0.95 * rightPower);
            driveRightBack.setPower(rightPower);

        } while (!lineDetected       //run while the line is not detected
                && this.getRuntime() < 6 && this.opModeIsActive());

        lineDetected = false;

        telemetry.addData("SPIN CCW TO LINE DONE", telemetryVariable);


    }

    void spinMoveCounterClockwiseToCheckForWhiteLine(double checkRotationDegreeDelta) {

        /*
         * How to use this method:
         *  Run the code and watch the robot spin counter-clockwise for a predetermined number of degrees unless it sees white and then rotate to its initial position
         *  If the robot sees white, the function will make "stillAtWhiteLine" true.
         *  Programmer inputs the degree difference the programmer wants the robot to rotate
         */

        //SPIN MOVE

        driveGainODS = 0.005;       //OK for spin move

        this.resetStartTime();

        lineDetected = false;

        do {

            spin.setPower(1);  // Eject debris while driving, to clear path

            //get and assign the raw infrared value the sensor detects
            rawDetectedLight = floorODS.getLightDetectedRaw();

            if (rawDetectedLight > ODS_THRESHOLD) {
                lineDetected = true;
            }

            //takes the current heading of the gyro
            currentHeading = gyro.getIntegratedZValue();

            //telemetry to print the current signed heading
            telemetry.addData("current signed heading: ", currentHeading);

            //telemetry to print the current raw light detection values
            telemetry.addData("current ODS raw light value: ", rawDetectedLight);

            //for CCW spin, left tread runs backwards
            leftPower = -.7;

            //for CCW spin, right tread runs forwards
            rightPower = .7;


            //when spinning CCW, left front is trailing, left back is leading
            //right front is leading, right back is trailing
            //trailing gets calculated full power, leading gets 95% of calculated full  power

            driveLeftFront.setPower(leftPower);
            driveLeftBack.setPower(0.95 * leftPower);
            driveRightFront.setPower(0.95 * rightPower);
            driveRightBack.setPower(rightPower);

        } while (
                this.getRuntime() < 6 && this.opModeIsActive() && currentHeading < (-90 + checkRotationDegreeDelta) && !lineDetected);

        stillAtWhiteLine = lineDetected;

        telemetry.addData("FUNCTION DONE. Line Detected: ", lineDetected);

        lineDetected = false;

    }

    void proportionalLineFollowForDistanceWithACORNRightSideOfLineLeftSideSensor(double distance, double midPower) {
        /*
         * How to use this method:
         *  Programmer inputs distance and midpower
         *      Distance = INCREMENTAL distance that you wish to have the robot run this time
         *      MidPower = the desired midpower that you wish to have the robot run with
         * This follows the line with the sensor on the right side of the right side of the robot
         *
         * Drive backwards a desired distance while following the right side of the white line
         */

        telemetry.clearData();      //clear all telemetry data before starting

        driveGainODS = 0.005;           //gain for proportional control

        targetDistance = distance;              //drive straight given inches from parameter

        //math for target encoder counts to travel
        rotations = targetDistance / circumference;
        targetEncoderCounts = (int) (encoderCountsPerRotation * rotations); //casts the target encoder counts as an integer

        //resets start time before starting the drive
        this.resetStartTime();

        //takes the initial position of the encoders to establish a starting point for the distance
        initialEncCountLeft = driveLeftFront.getCurrentPosition();      //here we are using the LEFT FRONT ENCODER because it is on an idle wheel and
                                                                        //it also is closer to the ACORN side
                                                                        //During testing we found that the treads had backlash and
                                                                        //using an idle wheel was thought to fix it. Has not been tested.


        do {
            spin.setPower(1);  // Eject debris while driving, to clear path

            currentEncDeltaCountLeft = driveLeftFront.getCurrentPosition() - initialEncCountLeft;         //the current - initial will give the
                                                                                                          // current distance of the encoders

            EncErrorLeft = targetEncoderCounts - Math.abs(currentEncDeltaCountLeft);                     //the error is the delta between the target counts and current counts


            //telemetry for encoder information
            telemetry.addData("EncErrorLeft = ", EncErrorLeft);
            telemetry.addData("Left Encoder: ", currentEncDeltaCountLeft);

            //telemetry for the distance travelled (IN INCHES)
            currentDistance = (currentEncDeltaCountLeft * circumference) / encoderCountsPerRotation;
            telemetry.addData("Calculated current distance: ", currentDistance);


            rawDetectedLight = floorODS.getLightDetectedRaw();

            proportionalODSError = DESIREDLINEFOLLOWNUMBER - rawDetectedLight; //lets say more white, this would be -

            driveSteering = proportionalODSError * driveGainODS; //this would be -

            leftPower = midPower;                           //This allows the right side of the robot to move at a constant speed
            rightPower = midPower - driveSteering;                                          //left side is the one that gets altered with the driving
            if (rightPower > 1.0) {                                         //Addition because need to decrease power when more white is seen
                rightPower = 1.0;
            }
            if (rightPower < 0.2) {
                rightPower = 0.2;
            }
            //when driving backwards, reverse leading and trailing
            //left front is now trailing, left back is now leading
            //trailing gets full power
            driveLeftFront.setPower(-leftPower);
            driveLeftBack.setPower(-.95 * leftPower);       //creates belt tension between the drive pulleys
            driveRightFront.setPower(-rightPower);
            driveRightBack.setPower(-.95 * rightPower);


        }
        while (EncErrorLeft > 0                   //the error is slowly decreasing, so run while greater than 0
                && this.getRuntime() < 10 && this.opModeIsActive());         //safety timeout of 17 seconds


        telemetry.addData("DRIVE STRAIGHT PROPORTIONAL BACKWARDS DONE", telemetryVariable);

    }

}//finish the code

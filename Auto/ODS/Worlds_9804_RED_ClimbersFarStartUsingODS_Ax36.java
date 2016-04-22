package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//  TO TEST A NON PROPORTIONAL LINE FOLLOW

/**
 * FTC Team 9804 Bomb Squad -- Autonomous
 * Made by the programmers of FTC Team 9804 Bomb Squad
 * <p>
 * Drives a predetermined set distance
 * <p>
 * <p>
 * Ax10 4-8-16 at 8:32 pm Steve -- introduce new autonomous blue program with optical distance sensor methods
 * Ax11 4-8-16 at 9:13 pm Steve -- new code with layout of movement patterns
 * Ax21 4-9-16 at 6:35 pm Steve -- code with primitive line follow
 * Ax22 4-9-16 at 6:59 pm Steve -- code with v1 of proportional line follow
 * Ax23 4-9-16 at 7:57 pm Steve -- code with revised layout of movement patterns
 * Ax24 ****** at *** pm Bridget -- ~code~
 * Ax33 4-20-16 at 6:08 pm Etienne and Steve --updated code with new autonomous robot movement, new method for calculating distance on white line
 * Ax34 4-20-16 at 7:53 pm Steve & Etienne -- updated cde with logic and proportional control revisions
 * Ax35 4-21-16 at 3:49 pm Steve & Etienne -- updated with new robot movement and adjusted ods values; added telemetry to ods sensor
 * Ax36 4-21-16 at 5:27 pm Steve & Etienne -- updated code with new motors/servos/servo positions; updated comments
 * <p>
 * <p>
 * <p>
 * <p>
 * ~~~~MOVEMENT_(VERSION Tx01)~~~~
 * (1) The Robot is aimed at the white line at about a 45 degree angle from the wall towards the white line.
 * (2) When it reaches 90 (or a determined encoder distance), drive backward at 0.3 power
 * (3) As soon as the robot sees white, STOP (with safety time limit)
 * (4) Overshoot line with the distance between the sensor and the center of the robot (7 inch ish ??)
 * (5) Spin CW until white line
 * (6) Move backward a tested distance
 * (7) Line follow until right and left sensors are pressed
 * (8) Move backward another set distance
 * (9) Rotate 45 degrees CW
 * (10) Move forward a tested distance
 * (11) Rotate 45 degrees CCW
 * (12) Move forward another set distance
 * (13) deploy climbers
 * <p>
 * <p>
 * <p>
 * ~~~~MOVEMENT_(VERSION Tx11)~~~~
 * (1) Drive forward 2 full tile lengths (48 inches)
 * (2) Pivot 45º clockwise
 * (3) Drive until white line is sensed
 * (4) Pivot 45º counter clockwise
 * (5) Drive forwards 24 inches
 * (6) Pivot 180º
 * (7) Drive forwards until white line is sensed
 * (8) Pivot 90º counter clockwise
 * (9) Score climbers
 * (10) Code complete, exit loop -- further autonomous codes have different finish options
 * <p>
 * <p>
 * <p>
 * ~~~~MOVEMENT_(VERSION Tx12){Setup: right side edge of first box from mountain}~~~~
 * (1) Drive forward 1 full tile lengths (24 inches)
 * (2) Pivot 45º counter clockwise
 * (3) Drive until white line is sensed w/ right ods sensor
 * (4) High-gain left side line follow w/ right ods sensor for 12 inches
 * (5) Backup to red line with right ods
 * (6) Drive until white line is sensed w/ right ods sensor
 * (7) Medium-gain left side line follow w/ right ods sensor for 12 inches
 * (8) Backup to red line with right ods
 * (9) Drive until white line is sensed w/ right ods sensor
 * (10) Low-gain left side line follow w/ right ods sensor for 12 inches
 * (11) Backup to red line with right ods
 * (12) Medium gain left side line follow w/ right ods sensor for 24 inches
 * (13) Score climbers
 * (14) Code complete, exit while loop
 * <p>
 * ~~~~SETUP_(VERSION Tx21)~~~~
 * (1) Right side of the robot is in the middle of first full tile from mountain
 * <p>
 * ~~~~MOVEMENT_(VERSION Tx22)~~~~
 * (1) Record angle
 * (2) Drive backwards 1 full tile (24 inches)
 * (3) Rotate 45º counter-clockwise
 * (4) Drive backwards until white line is seen w/ right ods sensor
 * (5) Record angle
 * (6) Drive forwards away from white line until blue line is seen WHILE RECORDING DISTANCE
 * (7) leg of triangle along white line is sin(theta)*hypotenuse
 * (8) Loop until conditions are met
 * Substeps for loop
 * -1- Spin move clockwise to new desired heading (delta variable **TEST THIS**)
 * -2- drive backwards until white line is sensed
 * -3- drive forwards to the blue line while counting recorder counts and converting to inches
 * -4- check to see if conditions are met, if so, EXIT loop
 * $if not, repeat
 * (9) drive to the white line backwards
 * (10) proportional line follow with adjusted center of rotation navigation (ACORN) with calculated distance
 * (11) once distance is reached, stop and score
 * (12) code complete!
 * <p>
 * <p>
 * <p>
 * <p>
 * <p>
 * * ~~~~SETUP_(VERSION Tx21)~~~~
 * (1) left side of the robot is in the middle of second full tile from mountain
 * <p>
 * ~~~~MOVEMENT_(VERSION Tx23)~~~~
 * (1) Record angle
 * (2) Drive backwards 1 full tile (24 inches)
 * (3) Rotate 45º counter-clockwise
 * (4) Drive backwards until white line is seen w/ right ods sensor
 * (5) Record angle
 * (6) Drive forwards away from white line until blue line is seen WHILE RECORDING DISTANCE
 * (7) leg of triangle along white line is sin(theta)*hypotenuse
 * (8) Loop until conditions are met
 * Substeps for loop
 * -1- Spin move clockwise to new desired heading (delta variable **TEST THIS**)
 * -2- drive backwards until white line is sensed
 * -3- drive forwards to the blue line while counting recorder counts and converting to inches
 * -4- check to see if conditions are met, if so, EXIT loop
 * $if not, repeat
 * (9) drive to the white line backwards
 * (10) overshoot to red line
 * (11) spin move to white line counter clockwise
 * (12) proportional line follow with adjusted center of rotation navigation (ACORN) for 9.5 inches
 * (13) stop when robot is five inches from the wall
 * (14) once distance is reached, stop and score
 * (15) code complete!
 * All the steps correspond to actual steps in the op mode
 * <p>
 * <p>
 * ~~~~MOVEMENT_(VERSION Tx24)~~~~
 * (1)
 * <p>
 * <p>
 * <p>
 * <p>
 * GENERAL RULE:
 * FWD: leftPower = midPower - driveSteering;
 * BWD: leftPower = midPower + drive Steering
 * CCW: positive
 * CW: negative
 * Heading = ABSOLUTE heading of the robot on the field
 * Distance = INCREMENTAL distance of the robot on the field
 * <p>
 * referencing floorODS in this program call values from irr, which is the ods on the right side of the robot, used when red
 * <p>
 * <p>
 * ACORN = adjusted center of rotation navigation; we use this for our proportional line follow because our ods sensors are off centered
 * <p>
 * <p>
 * <p>
 * +++++++++++++++++++++++++++++++Configuration Info+++++++++++++++++++++++++++++++
 * <p>
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
 * <p>
 * <p>
 * <p>
 * FOR THE WHITE LINE DETECTION, THE FUNCTION IS WRITTEN. THESE THINGS NEED TO BE ADDED
 * <p>
 * //This assigns the raw infared value to RawDetectedODS
 * RawDetectedODS = floorODS.getLightDetectedRaw();
 * <p>
 * //sends the value to the function where true/false wil be assigned
 * foundWhiteLine(RawDetectedODS);
 * <p>
 * //declare the ODS Sensor aimed at the floor.
 * OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.gyroSensor.get("ods");
 * <p>
 * <p>
 * NOTE ABOUT THE CODE:
 * To prevent catastrophic errors when the code is running,
 * we include a stop motors and a wait one full hardware cycle after EVERY hardware command
 * <p>
 * <p>
 * Center of Rotation adjustment notes:
 * If the sensor is on the right side of the robot:
 * Right motor = mid power
 * Left motor = mid power + proportional steering value (error * gain)
 * <p>
 * <p>
 * <p>
 * <p>
 * CHECKLIST TO DO:
 * CALCULATE THE DISTANCE LEFT TO GO REGARDING THE ARC LENGTH
 * SPEED UP ON STRAIGHTAWAYS AND SLOW DOWN AS WE GET CLOSER TO TARGET
 * ADD THE PROPORTIONAL LINE FOLLOW IN THE CODE FOR 5 INCHES LEFT TO SHELTER
 */


public class Worlds_9804_RED_ClimbersFarStartUsingODS_Ax36 extends LinearOpMode {

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
    //This mechanism is used in the AUTONOMOUS and TELEOP segments and is attched to the top of our robot.
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
    Servo windowWiperL;                    //Standard Servo
    Servo windowWiperR;

    /**
     * CLIFF CAM SERVOS
     */
    Servo cliffCamL;                    //initialize to 1
    Servo cliffCamR;                    //initialize to 0


    //%%&&~~~~~~~~~VARIABLE DECLARATIONS~~~~~~~~~&&%%//

    boolean runMe = true;                   //boolean for exiting code while loop

    double midPower;                        //the middle power for driving that we add and subtract calculates values from
    int targetHeading;                      //target heading the gyro will go to
    double driveGain;                       //the gain for proportional control
    double leftPower;                       //power for drive motors on the left side
    double rightPower;                      //power for drive motors on the right side
    int currentHeading = 0;                 //This is a signed value, CCW is positive
    int headingError;                       //the error in the heading between the target and current values
    double driveSteering;                   //the variable for proportional control
    double currentDistance;                 //the calculated distance we have travelled
    int currentEncDeltaCountLeft;           //the change (delta) for the left encoder
    int currentEncDeltaCountRight;          //the change (delta) for the right encoder


    double targetDistance;                      //magnitude in distance
    final int encoderCountsPerRotation = 1120;  //AndyMark encoders use 1120 counts per revolution
    final double diameter = 2.583;              //effective diameter of drive pulley and tread b/c of sinking into foam mat (measures 3" dia)
    double circumference = diameter * 3.14159;  //Circumference = PI * Diameter
    double rotations;                           //desired rotations
    int targetEncoderCounts;                    //calculated target encoder counts
    int EncErrorLeft;                           //the error for the left encoder
    int telemetryVariable;                      // a NULL telemetry variable designed to diplay text only
    int initialEncCountLeft;                    //initial encoder counts of the left encoder
    int initialEncCountRight;                   //initial encoder counts of the right encoder


    //servo variables
    double grabLeftUp = 0;                  //0 is max CCW (UP on left side)
    double grabRightUp = 1.0;               //1 is max CW (UP on right side)
    double sweepOpenedL = 0.75;
    double sweepClosedL = 0;
    double sweepPositionL = sweepClosedL;
    double sweepOpenedR = 0.25;
    double sweepClosedR = 1;
    double sweepPositionR = sweepClosedR;
    double cliffCamLClosed = 1;
    double cliffCamRClosed = 0;
    double boxPosition = 0.5;
    double hookPolesInitialize = 0.51;
    double hookPolesPosition = hookPolesInitialize;
    double shelterDropInitialize = 0.5;
    double shelterDropRelease = 0.87;
    double shelterDropRetract = 0.2;
    double shelterScoreTime;
    double shelterDropPosition = shelterDropInitialize;
    double ziplineBarInitialize = 0;
    double ziplineBarPosition = ziplineBarInitialize;

    //variables for auto
    int initialGyroHeading;
    int angleOfTriangleCalculationsDegrees;
    double angleOfTriangleCalculationsRadians;
    int initialEncoderValue;
    int concludingEncoderValue;
    int totalEncoderMovement;
    double calculatedDistanceInches;
    double distanceTravelledOnWhiteLine;
    double distanceFromShelterOnLine;

    static double DESIREDDISTANCEFROMSHELTER = 20;
    int currentGlobalHeadingForSpinMoves = 0;
    static int DELTADEGREESPINMOVE = 10;


    //ODS VARIABLES
    int rawDetectedODS;                  //ODS Light detection value
    static int ODS_THRESHOLD = 700;      //ODS Light threshold ****NEEDS TO BE TESTED*****
    boolean lineDetected = false;       //boolean to leave do-while loop in methods
    static int DESIREDLINEFOLLOWNUMBER = 410; //the number for perfect line follow along the edge of the line, NEEDS TO BE TESTED
    double proportionalODSError;
    static int BLUE_ODS_THRESHOLD = 135;
    static int RED_ODS_THRESHOLD = 576;  // BASED ON BRIDGET'S TESTING please confirm

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

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        hardwareMap.logDevices();

        //calibrate the gyro and get the current heading
        gyro.calibrate();


        //begin match with driver station button
        waitForStart();

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        while (this.opModeIsActive() && runMe) {     //the op mode is active conditional forces the code to stop once the driver station specifies
            /**
             * Each step refers to the steps in the movement from Tx22
             */

            //step 1

            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            initialGyroHeading = gyro.getIntegratedZValue();

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            //step 2
            driveStraightBackwards(initialGyroHeading, 24, 0.7);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 3
            spinMoveCounterClockwise((initialGyroHeading + 45));

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 4
            driveStraightBackwardsUntilWhiteLineIsDetected((initialGyroHeading + 45), 0.5);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 5
            angleOfTriangleCalculationsDegrees = gyro.getIntegratedZValue();

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 6
            initialEncoderValue = driveLeftBack.getCurrentPosition();

            driveStraightForwardsUntilRedLineIsDetected(angleOfTriangleCalculationsDegrees, 0.5);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            concludingEncoderValue = driveLeftBack.getCurrentPosition();

            //step 7

            calculateDistanceTravelledByEncodersOnWhiteLine(initialEncoderValue, concludingEncoderValue, angleOfTriangleCalculationsDegrees);

            //step 8

            currentGlobalHeadingForSpinMoves = gyro.getIntegratedZValue();

            do {

                spinMoveClockwise(currentGlobalHeadingForSpinMoves - DELTADEGREESPINMOVE);

                waitOneFullHardwareCycle();

                stopMotors();

                waitOneFullHardwareCycle();

                angleOfTriangleCalculationsDegrees = gyro.getIntegratedZValue();

                driveStraightBackwardsUntilWhiteLineIsDetected(angleOfTriangleCalculationsDegrees, 0.5);

                waitOneFullHardwareCycle();

                stopMotors();

                waitOneFullHardwareCycle();

                initialEncoderValue = driveLeftBack.getCurrentPosition();

                driveStraightForwardsUntilRedLineIsDetected(angleOfTriangleCalculationsDegrees, 0.5);

                waitOneFullHardwareCycle();

                stopMotors();

                waitOneFullHardwareCycle();

                concludingEncoderValue = driveLeftBack.getCurrentPosition();

                angleOfTriangleCalculationsDegrees = gyro.getIntegratedZValue();

                distanceTravelledOnWhiteLine = calculateDistanceTravelledByEncodersOnWhiteLine(initialEncoderValue, concludingEncoderValue, angleOfTriangleCalculationsDegrees);

                distanceFromShelterOnLine = 22 - distanceTravelledOnWhiteLine;


            }
            while (this.opModeIsActive() && this.getRuntime() < 15 && distanceFromShelterOnLine < DESIREDDISTANCEFROMSHELTER);

            angleOfTriangleCalculationsDegrees = gyro.getIntegratedZValue();

            //step 9
            driveStraightBackwardsUntilWhiteLineIsDetected(angleOfTriangleCalculationsDegrees, 0.5);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 10
            driveStraightBackwardsUntilRedLineIsDetected(angleOfTriangleCalculationsDegrees, 0.5);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 11
            spinMoveCounterClockwiseUntilWhiteLineIsDetected();

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 12
            proportionalLineFollowForDistanceWithACORNRightSideOfLineRightSideSensor(9.5, 0.6);

            waitOneFullHardwareCycle();

            stopMotors();

            waitOneFullHardwareCycle();

            //step 13
            scoreShelterDrop(2);

            waitOneFullHardwareCycle();

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            //step 14
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

    void spinMoveCounterClockwise(int heading) {
        /*
         * How to use this method:
         *  In the op mode, input the ABSOLUTE heading of the desired position for the robot on the field.
         *  After the code is finished, run the stopMotors method to fully stop all drive and spin motors
         */

        //SPIN MOVE
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");


        driveGain = 0.05;       //OK for spin move
        targetHeading = heading;    //CCW (using signed heading) (positive value CCW)

        this.resetStartTime();

        do {
            //takes the current heading of the gyro
            currentHeading = gyro.getIntegratedZValue();

            //telemetry to print the current signed heading
            telemetry.addData("current signed heading: ", currentHeading);

            //takes the heading error for the value of our gyro
            headingError = targetHeading - currentHeading;//for CCW spin from 0 to +º, error always positive

            //drive steering for proportional control
            driveSteering = headingError * driveGain;     //positive value for CCW

            //for CCW spin, left tread runs backwards
            leftPower = -driveSteering;
            if (leftPower < -1) {
                leftPower = -1;
            }
            if (leftPower > -0.6) {           //avoid zero closing power at low error
                leftPower = -0.6;            //anything less than 0.6 stalled near target heading
            }


            //for CCW spin, right tread runs forwards
            rightPower = driveSteering;
            if (rightPower > 1) {
                rightPower = 1;
            }
            if (rightPower < 0.6) {
                rightPower = 0.6;
            }

            //when spinning CCW, left front is trailing, left back is leading
            //right front is leading, right back is trailing
            //trailing gets calculated full power, leading gets 95% of calculated full  power

            driveLeftFront.setPower(leftPower);
            driveLeftBack.setPower(0.95 * leftPower);
            driveRightFront.setPower(0.95 * rightPower);
            driveRightBack.setPower(rightPower);

        } while (currentHeading < targetHeading
                && this.getRuntime() < 6 && this.opModeIsActive());
        //spin from 0 to + number, so loop while 'less than' the target heading

        telemetry.addData("SPIN CCW DONE", telemetryVariable);


    }

    void spinMoveClockwise(int heading) {
       /*
         * How to use this method:
         *  In the op mode, input the ABSOLUTE heading of the desired position for the robot on the field.
         *  After the code is finished, run the stopMotors method to fully stop all drive and spin motors
         */
        //SPIN MOVE
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        driveGain = 0.05;       //OK for spin
        targetHeading = heading;    //90º CW (using signed heading) (positive value CCW)

        this.resetStartTime();

        do {
            //takes the current heading of the gyro
            currentHeading = gyro.getIntegratedZValue();

            //telemetry to display the current signed heading
            telemetry.addData("current signed heading: ", currentHeading);

            //takes the heading error for the value of our gyro
            headingError = targetHeading - currentHeading;//for CW spin from 0 to -º, error always negative

            //drive steering for proportional control
            driveSteering = headingError * driveGain;         //negative value

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

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        driveGain = 0.05;           //gain used for proportional steering
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
            driveSteering = headingError * driveGain;           //positive if pointing too far CW

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

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

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

            driveSteering = headingError * driveGain;       //create the proportion for the steering

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //ODS DRIVING METHODS

    void driveStraightForwardsUntilBlueLineIsDetected(int heading, double midPower) {

        /*
         * Purpose of this method:
         *  drive backwards with spinners running until the robot senses a blue line
         *
         * How to use this method:
         *  Programmer inputs heading and the midpower for driving
         *       Heading = ABSOLUTE heading of the robot on the field
         *       MidPower = the desired midpower that you wish to have the robot run with
         *
         */

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

        targetHeading = heading;                //drive straight ahead at the initial/default heading


        //resets start time before starting the drive
        this.resetStartTime();


        do {

            //get and assign the raw infrared value the sensor detects
            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);


            if (rawDetectedODS > BLUE_ODS_THRESHOLD) {
                lineDetected = true;
            }

            spin.setPower(1);  // Eject debris while driving, to clear path

            // get the Z-axis heading info.
            //this is a signed heading not a basic heading
            currentHeading = gyro.getIntegratedZValue();

            headingError = targetHeading - currentHeading;  //find the error between the headings

            driveSteering = headingError * driveGain;       //create the proportion for the steering

            leftPower = midPower + driveSteering;           //adds the drive steering to midpower
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
            driveLeftFront.setPower(leftPower);
            driveLeftBack.setPower(.95 * leftPower);       //creates belt tension between the drive pulleys
            driveRightFront.setPower(rightPower);
            driveRightBack.setPower(.95 * rightPower);


        }
        while (!lineDetected       //run while the line is not detected
                && this.getRuntime() < 12 && this.opModeIsActive());         //safety timeout of 12 seconds

        lineDetected = false; //reset the variable to be used again

        telemetry.addData("DRIVE STRAIGHT BACKWARDS TO LINE DONE", telemetryVariable);

    }

    void driveStraightForwardsUntilRedLineIsDetected(int heading, double midPower) {

        /*
         * Purpose of this method:
         *  drive forwards with spinners running until the robot senses a red line
         *
         * How to use this method:
         *  Programmer inputs heading and the midpower for driving
         *       Heading = ABSOLUTE heading of the robot on the field
         *       MidPower = the desired midpower that you wish to have the robot run with
         *
         */

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

        targetHeading = heading;                //drive straight ahead at the initial/default heading


        //resets start time before starting the drive
        this.resetStartTime();


        do {

            //get and assign the raw infrared value the sensor detects
            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);


            if (rawDetectedODS > RED_ODS_THRESHOLD) {
                lineDetected = true;
            }

            spin.setPower(1);  // Eject debris while driving, to clear path

            // get the Z-axis heading info.
            //this is a signed heading not a basic heading
            currentHeading = gyro.getIntegratedZValue();

            headingError = targetHeading - currentHeading;  //find the error between the headings

            driveSteering = headingError * driveGain;       //create the proportion for the steering

            leftPower = midPower + driveSteering;           //adds the drive steering to midpower
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
            driveLeftFront.setPower(.95 * leftPower);
            driveLeftBack.setPower(leftPower);       //creates belt tension between the drive pulleys
            driveRightFront.setPower(.95 * rightPower);
            driveRightBack.setPower(rightPower);


        }
        while (!lineDetected       //run while the line is not detected
                && this.getRuntime() < 12 && this.opModeIsActive());         //safety timeout of 12 seconds

        lineDetected = false; //reset the variable to be used again

        telemetry.addData("DRIVE STRAIGHT BACKWARDS TO LINE DONE", telemetryVariable);

    }

    void driveStraightBackwardsUntilRedLineIsDetected(int heading, double midPower) {

        /*
         * Purpose of this method:
         *  drive backwards with spinners running until the robot senses a red line
         *
         * How to use this method:
         *  Programmer inputs heading and the midpower for driving
         *       Heading = ABSOLUTE heading of the robot on the field
         *       MidPower = the desired midpower that you wish to have the robot run with
         *
         */

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

        targetHeading = heading;                //drive straight ahead at the initial/default heading


        //resets start time before starting the drive
        this.resetStartTime();


        do {

            //get and assign the raw infrared value the sensor detects
            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            if (rawDetectedODS > RED_ODS_THRESHOLD) {
                lineDetected = true;
            }

            spin.setPower(1);  // Eject debris while driving, to clear path

            // get the Z-axis heading info.
            //this is a signed heading not a basic heading
            currentHeading = gyro.getIntegratedZValue();

            headingError = targetHeading - currentHeading;  //find the error between the headings

            driveSteering = headingError * driveGain;       //create the proportion for the steering

            leftPower = midPower + driveSteering;           //adds the drive steering to midpower
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

        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

        targetHeading = heading;                //drive straight ahead at the initial/default heading


        //resets start time before starting the drive
        this.resetStartTime();


        do {

            //get and assign the raw infrared value the sensor detects
            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            if (rawDetectedODS > ODS_THRESHOLD) {
                lineDetected = true;
            }

            spin.setPower(1);  // Eject debris while driving, to clear path

            // get the Z-axis heading info.
            //this is a signed heading not a basic heading
            currentHeading = gyro.getIntegratedZValue();

            headingError = targetHeading - currentHeading;  //find the error between the headings

            driveSteering = headingError * driveGain;       //create the proportion for the steering

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
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        driveGain = 0.05;       //OK for spin move

        lineDetected = false;

        this.resetStartTime();

        do {
            //get and assign the raw infrared value the sensor detects
            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            if (rawDetectedODS > ODS_THRESHOLD) {
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

    void spinMoveClockwiseUntilWhiteLineIsDetected() {

        /*
         * How to use this method:
         *  Run the code and watch the robot spin to the line
         */

        //SPIN MOVE
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        driveGain = 0.05;       //OK for spin move

        this.resetStartTime();

        do {
            //get and assign the raw infrared value the sensor detects
            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            if (rawDetectedODS > ODS_THRESHOLD) {
                lineDetected = true;
            }

            //takes the current heading of the gyro
            currentHeading = gyro.getIntegratedZValue();

            //telemetry to print the current signed heading
            telemetry.addData("current signed heading: ", currentHeading);

            //for CCW spin, left tread runs forwards
            leftPower = .7;

            //for CCW spin, right tread runs backwards
            rightPower = -.7;


            //when spinning CW, left front is leading, left back is trailing
            //right front is trailing, right back is leading
            //trailing gets calculated full power, leading gets 95% of calculated full  power

            driveLeftFront.setPower(0.95 * leftPower);
            driveLeftBack.setPower(leftPower);
            driveRightFront.setPower(rightPower);
            driveRightBack.setPower(0.95 * rightPower);

        } while (!lineDetected       //run while the line is not detected
                && this.getRuntime() < 6 && this.opModeIsActive());

        lineDetected = false;

        telemetry.addData("SPIN CW TO LINE DONE", telemetryVariable);


    }

    void lineFollowBackwardsLeftSideOfLine() {

        /*
         *
         * Purpose of this method:
         *  Run the code and watch the robot follow the line left side of the line until the touch sensors are triggered.
         *  The robot will drive backwards with spin motors running.
         *
         */

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        driveGain = 0.05;       //OK for spin move

        this.resetStartTime();


        while (this.opModeIsActive() /* && code here for touch sensor not active*/) {

            spin.setPower(1);  // Eject debris while driving, to clear path

            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            if (rawDetectedODS > ODS_THRESHOLD) {
                leftPower = -0.9;
                rightPower = -0.5;
            } else if (rawDetectedODS < ODS_THRESHOLD) {
                leftPower = -0.5;
                rightPower = -0.9;
            } else {
                leftPower = -0.7;
                rightPower = -0.7;
            }

            //when driving backwards, reverse leading and trailing
            //left front is now trailing, left back is now leading
            //trailing gets full power
            driveLeftFront.setPower(-leftPower);
            driveLeftBack.setPower(-.95 * leftPower);       //creates belt tension between the drive pulleys
            driveRightFront.setPower(-rightPower);
            driveRightBack.setPower(-.95 * rightPower);

        }
    }

    void lineFollowBackwardsRightSideOfLine() {

        /*
         *
         * Purpose of this method:
         *  Run the code and watch the robot follow the line left side of the line until the touch sensors are triggered.
         *  The robot will drive backwards with spin motors running.
         *
         */

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        driveGain = 0.05;       //OK for spin move

        this.resetStartTime();


        while (this.opModeIsActive() /* && code here for touch sensor not active*/) {

            spin.setPower(1);  // Eject debris while driving, to clear path

            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            if (rawDetectedODS > ODS_THRESHOLD) {
                leftPower = -0.5;
                rightPower = -0.9;
            } else if (rawDetectedODS < ODS_THRESHOLD) {
                leftPower = -0.9;
                rightPower = -0.9;
            } else {
                leftPower = -0.7;
                rightPower = -0.7;
            }

            //when driving forward, left front is leading, left back is now trailing, same for right
            //trailing gets full power, leading gets 95% of full power
            driveLeftFront.setPower(0.95 * leftPower);
            driveLeftBack.setPower(leftPower);
            driveRightFront.setPower(0.95 * rightPower);
            driveRightBack.setPower(rightPower);

        }
    }

    void proportionalLineFollowBackwardsLeftSideOfLine(double midPower) {

        /*
         *
         * Purpose of this method:
         *  Run the code and watch the robot follow the line left side of the line until the touch sensors are triggered.
         *  The robot will drive backwards with spin motors running.
         *
         *
         * How to use this method:
         *  Programmer inputs the midpower for driving
         *       MidPower = the desired midpower that you wish to have the robot run with
         *
         */

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        driveGain = 0.05;       //OK for spin move

        this.resetStartTime();


        do {

            spin.setPower(1);  // Eject debris while driving, to clear path

            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            proportionalODSError = DESIREDLINEFOLLOWNUMBER - rawDetectedODS; //lets say more white, this would be -

            driveSteering = proportionalODSError * driveGain; //this would be -

            //left power needs to be more than right power to correct ourselves
            leftPower = midPower - driveSteering;           //adds the drive steering to midpower because we are driving backwards
            if (leftPower > 1.0) {                            //cuts ourselves off at 1, the maximum motor power
                leftPower = 1.0;
            }
            if (leftPower < 0.2) {
                leftPower = 0.2;
            }
            rightPower = midPower + driveSteering;          //subtraction because we are driving backwards
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

        } while (this.opModeIsActive() /* && code here for touch sensor not active*/);

    }

    void proportionalLineFollowBackwardsRightSideOfLine(double midPower) {

        /*
         *
         * Purpose of this method:
         *  Run the code and watch the robot follow the line left side of the line until the touch sensors are triggered.
         *  The robot will drive backwards with spin motors running.
         *
         *
         * How to use this method:
         *  Programmer inputs the midpower for driving
         *       MidPower = the desired midpower that you wish to have the robot run with
         *
         */

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        driveGain = 0.05;       //OK for spin move

        this.resetStartTime();


        do {

            spin.setPower(1);  // Eject debris while driving, to clear path

            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            proportionalODSError = DESIREDLINEFOLLOWNUMBER - rawDetectedODS; //lets say more white, this would be -

            driveSteering = proportionalODSError * driveGain; //this would be -

            //left power needs to be less than right power to correct ourselves
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

        } while (this.opModeIsActive() /* && code here for touch sensor not active*/);

    }

    void LineFollowForwardsNonProportional() {

        OpticalDistanceSensor floorODS = hardwareMap.opticalDistanceSensor.get("ods2");
        this.resetStartTime();

        do {
            spin.setPower(1);  // Eject debris while driving, to clear path
            double reflectance = floorODS.getLightDetected();
            double reflectanceThreshold = .25;

            //if sensor is on line: right motor rotates faster to move it off line
            if (reflectance >= reflectanceThreshold) {
                rightPower = -0.4;
                leftPower = .2;
            } else {
                //if sensor is not on line: left motor moves faster to rotate on line
                rightPower = .2;
                leftPower = -.4;

                driveLeftFront.setPower(-leftPower);
                driveLeftBack.setPower(-.95 * leftPower);       //creates belt tension between the drive pulleys
                driveRightFront.setPower(-rightPower);
                driveRightBack.setPower(-.95 * rightPower);

            }
        } while (this.opModeIsActive() /* && code here for touch sensor not active*/);

    }

    void proportionalLineFollowForDistanceWithACORNLeftSideOfLineLeftSideSensor(double distance, double midPower) {
        /*
         * How to use this method:
         *  Programmer inputs distance and midpower
         *      Distance = INCREMENTAL distance that you wish to have the robot run this time
         *      MidPower = the desired midpower that you wish to have the robot run with
         * This follows the line with the sensor on the right side of the left side of the robot
         *
         * Drive backwards a desired distance while following the left side of the white line
         */

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

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


            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            proportionalODSError = DESIREDLINEFOLLOWNUMBER - rawDetectedODS; //lets say more white, this would be -

            driveSteering = proportionalODSError * driveGain; //this would be -

            leftPower = midPower;                           //This allows the left side of the robot to move at a constant speed
            rightPower = midPower + driveSteering;          //Right side is the one that gets altered with the driving
            if (rightPower > 1.0) {                         //Addition because need to decrease power when more white is seen
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
                && this.getRuntime() < 17 && this.opModeIsActive());         //safety timeout of 17 seconds


        telemetry.addData("DRIVE STRAIGHT PROPORTIONAL BACKWARDS DONE", telemetryVariable);

    }

    void proportionalLineFollowForDistanceWithACORNRightSideOfLineRightSideSensor(double distance, double midPower) {
        /*
         * How to use this method:
         *  Programmer inputs distance and midpower
         *      Distance = INCREMENTAL distance that you wish to have the robot run this time
         *      MidPower = the desired midpower that you wish to have the robot run with
         * This follows the line with the sensor on the right side of the right side of the robot
         *
         * Drive backwards a desired distance while following the right side of the white line
         */

        OpticalDistanceSensor floorODS = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods2");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

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


            rawDetectedODS = floorODS.getLightDetectedRaw();

            telemetry.addData("ODS Reading : ", rawDetectedODS);

            proportionalODSError = DESIREDLINEFOLLOWNUMBER - rawDetectedODS; //lets say more white, this would be -

            driveSteering = proportionalODSError * driveGain; //this would be -

            leftPower = midPower + driveSteering;                           //This allows the right side of the robot to move at a constant speed
            rightPower = midPower;                                          //left side is the one that gets altered with the driving
            if (leftPower > 1.0) {                                         //Addition because need to decrease power when more white is seen
                leftPower = 1.0;
            }
            if (leftPower < 0.2) {
                leftPower = 0.2;
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
                && this.getRuntime() < 17 && this.opModeIsActive());         //safety timeout of 17 seconds


        telemetry.addData("DRIVE STRAIGHT PROPORTIONAL BACKWARDS DONE", telemetryVariable);

    }

    double calculateDistanceTravelledByEncodersOnWhiteLine(double initialEncoderValue, double concludingEncoderValue, int angleOfTriangleCalculationsDegrees) {

        /*
         * How to use this method:
         *  Programmer has the code input initial and concluding encoder counts as well as the current angle used for degree calculations
         *      encoder counts = absolute encoder counts for the robot
         *      angle = absolute angle of the robot on the field
         *  Purpose:
         *      Return a double value of the distance the robot has travelled on the white line
         */


        totalEncoderMovement = Math.abs((int) (concludingEncoderValue - initialEncoderValue));

        calculatedDistanceInches = (totalEncoderMovement / encoderCountsPerRotation) * circumference;

        angleOfTriangleCalculationsRadians = Math.toRadians((double) (angleOfTriangleCalculationsDegrees));

        distanceTravelledOnWhiteLine = Math.abs((Math.sin(angleOfTriangleCalculationsRadians) * calculatedDistanceInches));

        return distanceTravelledOnWhiteLine;

    }


}//finish the code

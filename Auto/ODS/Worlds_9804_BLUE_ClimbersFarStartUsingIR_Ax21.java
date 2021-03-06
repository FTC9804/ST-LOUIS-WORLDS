package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * FTC Team 9804 Bomb Squad -- Autonomous
 * Made by the programmers of FTC Team 9804 Bomb Squad
 *
 * Drives a predetermined set distance
 *
 * Ax10 4-8-16 at 8:32 pm Steve -- introduce new autonomous blue program with optical distance sensor methods
 * Ax11 4-8-16 at 9:13 pm Steve -- new code with layout of movement patterns
 * Ax21 4-9-16 at 6:35 pm Steve -- code with primitive line follow
 *
 *
 *
 * ~~~~MOVEMENT~~~~
(1) The Robot is aimed at the white line at about a 45 degree angle from the wall towards the white line.
(2) When it reaches 90 (or a determined encoder distance), drive backward at 0.3 power
(3) As soon as the robot sees white, STOP (with safety time limit)
(4) Overshoot line with the distance between the sensor and the center of the robot (7 inch ish ??)
(5) Spin CW until white line
(6) Move backward a tested distance
(7) Line follow until right and left sensors are pressed
(8) Move backward another set distance
(9) Rotate 45 degrees CW
(10) Move forward a tested distance
(11) Rotate 45 degrees CCW
(12) Move forward another set distance
(13) deploy climbers
 *
 *
 *
 *
 * GENERAL RULE:
 *  FWD: leftPower = midPower - driveSteering;
 *  BWD: leftPower = midPower + drive Steering
 *  CCW: positive
 *  CW: negative
 *  Heading = ABSOLUTE heading of the robot on the field
 *  Distance = INCREMENTAL distance of the robot on the field
 *
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
 * VUTK         Red             right encoder           1
 * VF7F         Green           driveLeftFront          1               m6
 * VF7F         Green           driveLeftBack           2               m5
 * VF7F         Green           left encoder            1
 * VSI1         White           hookPoles               3               s3
 * VSI1         White           grabRight               4               s2
 * VSI1         White           box                     5               s4
 * VSI1         White           grabLeft                6               s1
 * VCT7         Pink            ziplineBar              5               s8
 * VCT7         Pink            allClear (not used)     2               s7
 * VCT7         Pink            shelterDrop             6               s6
 * VCT7         Pink            windowWiper             1               s5
 * VCFP         Yellow          gyro                    I2C5            gyro
 * VCFP         Yellow          led extend              DO7             led1
 * VCFP         Yellow          led retract             DO1             led2
 * VCFP         Yellow          magnet extend           DO0             mag1
 * VCFP         Yellow          magnet retract          DO2             mag2
 * VCFP         Yellow          IR ODS                  ***             ir
 *              Blue            Power Distro <-> Phones
 *
 *
 *
 * FOR THE WHITE LINE DETECTION, THE FUNCTION IS WRITTEN. THESE THINGS NEED TO BE ADDED
 *
 * //This assigns the raw infared value to RawDetectedIR
            RawDetectedIR = floorIr.getLightDetectedRaw();

            //sends the value to the function where true/false wil be assigned
            foundWhiteLine(RawDetectedIR);

           //declare the IR Sensor aimed at the floor.
        OpticalDistanceSensor floorIr = (OpticalDistanceSensor) hardwareMap.gyroSensor.get("IR");

 */


public class Worlds_9804_BLUE_ClimbersFarStartUsingIR_Ax21 extends LinearOpMode {

    //drive motors
    //front is the side with the arms, back is the side with the spinners
    DcMotor driveLeftBack;
    DcMotor driveLeftFront;
    DcMotor driveRightBack;
    DcMotor driveRightFront;
    //two encoders are on rear motors, 2 are mounted on tread module
    DcMotor spin;

    //servos to lock in place on ramp
    Servo grabLeft;
    Servo grabRight;

    //servo for moving the hopper
    Servo box;

    //servo to place climber in shelter
    Servo shelterDrop;

    //servo to unleash the hooks
    Servo hookPoles;

    //servo to launch the wings to hit the zipline bar
    Servo ziplineBar;
//    Servo allClear;     //not used

    //servo to push away debris from ramp
    Servo windowWiper;

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
    double sweepOpened = 0.75;
    double sweepClosed = 0;
    double sweepPosition = sweepClosed;
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
//    double allClearInitialize = 0;
//    double allClearPosition = allClearInitialize;


    int rawDetectedIR;                  //IR Light detection value
    static int IR_THRESHOLD = 100;      //IR Light threshold ****NEEDS TO BE TESTED*****
    boolean lineDetected = false;       //boolean to leave do-while loop in methods

    //USE CONFIGURATION FILE 'JABBED' ON BOTH MAIN AND B PHONES


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
        //2 encoders are on rear motors, two are mounted in tread module

        spin = hardwareMap.dcMotor.get("m8");


        //give the configuration names for the servos
        grabLeft = hardwareMap.servo.get("s1");
        grabRight = hardwareMap.servo.get("s2");

        windowWiper = hardwareMap.servo.get("s5");

        box = hardwareMap.servo.get("s4");

        hookPoles = hardwareMap.servo.get("s3");

//        allClear = hardwareMap.servo.get("s7");
        ziplineBar = hardwareMap.servo.get("s8");

        shelterDrop = hardwareMap.servo.get("s6");

        //sets initial positions for the servos to activate to
        grabLeft.setPosition(grabLeftUp);
        grabRight.setPosition(grabRightUp);
        windowWiper.setPosition(sweepPosition);
        box.setPosition(boxPosition);
        hookPoles.setPosition(hookPolesPosition);
//        allClear.setPosition(allClearPosition);
        ziplineBar.setPosition(ziplineBarPosition);
        shelterDrop.setPosition(shelterDropPosition);


        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");    //initialize gyro to be used in code

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

            driveStraightBackwards(0, 90, 0.75);

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            driveStraightBackwardsUntilWhiteLineIsDetected(0, 0.3);

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            driveStraightBackwards(0, 7, 0.4);

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            spinMoveClockwiseUntilWhiteLineIsDetected();

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            currentHeading = gyro.getIntegratedZValue();

            driveStraightForwards(currentHeading, 12, 0.5);

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            //LINE FOLLOW HERE -- touch sensors need to be implemented

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            currentHeading = gyro.getIntegratedZValue();

            driveStraightBackwards(currentHeading, 12, 0.5);

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            spinMoveClockwise((currentHeading - 45));

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            currentHeading = gyro.getIntegratedZValue();

            driveStraightBackwards(currentHeading, 12, 0.5);

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            //xx

            currentHeading = gyro.getIntegratedZValue();

            spinMoveCounterClockwise((currentHeading + 45));

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            currentHeading = gyro.getIntegratedZValue();

            driveStraightBackwards(currentHeading, 12, 0.5);

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            scoreShelterDrop(2);

            stopMotors();       //stop motors to prevent further movement

            waitOneFullHardwareCycle();

            telemetry.addData("CODE COMPLETE", telemetryVariable);

            runMe = false;

        }

    } //finish the opmode

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
        windowWiper.setPosition(sweepOpened);
        this.resetStartTime();
        while (this.getRuntime() < 1 && this.opModeIsActive()) { //give a short period of time for the hardware to execute the command
            stopMotors();
        }
        windowWiper.setPosition(sweepClosed);
        this.resetStartTime();
        while (this.getRuntime() < 0.5 && this.opModeIsActive()) { //give a short period of time for the hardware to execute the command
            stopMotors();
        }


    }

    //NON IR DRIVING METHODS

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

    //IR DRIVING METHODS

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

        OpticalDistanceSensor floorIR = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ir");

        telemetry.clearData();      //clear all telemetry data before starting

        driveGain = 0.05;           //gain for proportional control

        targetHeading = heading;                //drive straight ahead at the initial/default heading


        //resets start time before starting the drive
        this.resetStartTime();


        do {

            //get and assign the raw infrared value the sensor detects
            rawDetectedIR = floorIR.getLightDetectedRaw();

            if (rawDetectedIR > IR_THRESHOLD) {
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
        while ( !lineDetected       //run while the line is not detected
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

        OpticalDistanceSensor floorIR = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ir");

        driveGain = 0.05;       //OK for spin move

        this.resetStartTime();

        do {
            //get and assign the raw infrared value the sensor detects
            rawDetectedIR = floorIR.getLightDetectedRaw();

            if (rawDetectedIR > IR_THRESHOLD) {
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

        } while ( !lineDetected       //run while the line is not detected
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

        OpticalDistanceSensor floorIR = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ir");

        driveGain = 0.05;       //OK for spin move

        this.resetStartTime();

        do {
            //get and assign the raw infrared value the sensor detects
            rawDetectedIR = floorIR.getLightDetectedRaw();

            if (rawDetectedIR > IR_THRESHOLD) {
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

        } while ( !lineDetected       //run while the line is not detected
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

        OpticalDistanceSensor floorIR = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ir");

        driveGain = 0.05;       //OK for spin move

        this.resetStartTime();


        while (this.opModeIsActive() /* && code here for touch sensor not active*/){

            spin.setPower(1);  // Eject debris while driving, to clear path

            rawDetectedIR = floorIR.getLightDetectedRaw();

            if (rawDetectedIR > IR_THRESHOLD) {
                leftPower = -0.9;
                rightPower = -0.5;
            }
            else if (rawDetectedIR < IR_THRESHOLD) {
                leftPower = -0.5;
                rightPower = -0.9;
            }
            else {
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

    void lineFollowBackwardsRightSideOfLine(){

        /*
         *
         * Purpose of this method:
         *  Run the code and watch the robot follow the line left side of the line until the touch sensors are triggered.
         *  The robot will drive backwards with spin motors running.
         *
         */

        OpticalDistanceSensor floorIR = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ir");

        driveGain = 0.05;       //OK for spin move

        this.resetStartTime();


        while (this.opModeIsActive() /* && code here for touch sensor not active*/){

            spin.setPower(1);  // Eject debris while driving, to clear path

            rawDetectedIR = floorIR.getLightDetectedRaw();

            if (rawDetectedIR > IR_THRESHOLD) {
                leftPower = -0.5;
                rightPower = -0.9;
            }
            else if (rawDetectedIR < IR_THRESHOLD) {
                leftPower = -0.9;
                rightPower = -0.9;
            }
            else {
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

}//finish the code

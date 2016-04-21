package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

//=============================================================================================
/*
* the IR Distance Sensor returns a value beteeen 0 and 1024 according to Team Torch. 0 is no light returned.
* Use "floorIR.getLightDetectedRaw() < LIGHT_THRESHOLD && opModeIsActive()" as the conditional for line detected

*This should be used to determine a good threshold for the infared sensor based on the distace from the floor and
*lighting conditions.

*/

public class IR_Distance_Sensor_Threshold_Tester_v00 extends LinearOpMode {

    int RawDetectedIR;

    //Static Values for the IR Thresholds

    static double IR_THRESHOLD1 = 40;
    static double IR_THRESHOLD2 = 50;
    static double IR_THRESHOLD3 = 60;
    static double IR_THRESHOLD4 = 70;
    static double IR_THRESHOLD5 = 80;
    static double IR_THRESHOLD6 = 90;
    static double IR_THRESHOLD7 = 100;
    static double IR_THRESHOLD8 = 110;

    //Just to make sure we dont have to make it any greater
    static double IR_THRESHOLD9 = 200;

    //bools displayed to phone
    boolean THRESHOLD1PASSED = false;
    boolean THRESHOLD2PASSED = false;
    boolean THRESHOLD3PASSED = false;
    boolean THRESHOLD4PASSED = false;
    boolean THRESHOLD5PASSED = false;
    boolean THRESHOLD6PASSED = false;
    boolean THRESHOLD7PASSED = false;
    boolean THRESHOLD8PASSED = false;
    boolean THRESHOLD9PASSED = false;

    @Override
    public void runOpMode() throws InterruptedException {


        //declare the IR Sensor aimed at the floor.
        OpticalDistanceSensor floorIr = (OpticalDistanceSensor) hardwareMap.gyroSensor.get("IR");

        hardwareMap.logDevices();

        while (this.opModeIsActive()) {

            //This assigns the raw infared value to RawDetectedIR
            RawDetectedIR = floorIr.getLightDetectedRaw();

            //sends the value to the function where true/false wil be assigned
            changeTelemetryWhenLineDetected(RawDetectedIR);

            //display whether or not the Thresholds have been passed
            telemetry.addData("Passed IR_THRESHOLD1: ", THRESHOLD1PASSED);
            telemetry.addData("Passed IR_THRESHOLD2: ", THRESHOLD2PASSED);
            telemetry.addData("Passed IR_THRESHOLD3: ", THRESHOLD3PASSED);
            telemetry.addData("Passed IR_THRESHOLD4: ", THRESHOLD4PASSED);
            telemetry.addData("Passed IR_THRESHOLD5: ", THRESHOLD5PASSED);
            telemetry.addData("Passed IR_THRESHOLD6: ", THRESHOLD6PASSED);
            telemetry.addData("Passed IR_THRESHOLD7: ", THRESHOLD7PASSED);
            telemetry.addData("Passed IR_THRESHOLD8: ", THRESHOLD8PASSED);
            telemetry.addData("Passed IR_THRESHOLD8: ", THRESHOLD9PASSED);

        } //finish the opmode
    }

        void changeTelemetryWhenLineDetected(double lightDetected){

            if (lightDetected > IR_THRESHOLD1){
                THRESHOLD1PASSED = true;
            } else {
                THRESHOLD1PASSED = false;
            }

            if (lightDetected > IR_THRESHOLD2){
                THRESHOLD2PASSED = true;
            } else {
                THRESHOLD2PASSED = false;
            }

            if (lightDetected > IR_THRESHOLD3){
                THRESHOLD3PASSED = true;
            } else {
                THRESHOLD3PASSED = false;
            }

            if (lightDetected > IR_THRESHOLD4){
                THRESHOLD4PASSED = true;
            } else {
                THRESHOLD4PASSED = false;
            }

            if (lightDetected > IR_THRESHOLD5){
                THRESHOLD5PASSED = true;
            } else {
                THRESHOLD5PASSED = false;
            }

            if (lightDetected > IR_THRESHOLD6){
                THRESHOLD6PASSED = true;
            } else {
                THRESHOLD6PASSED = false;
            }

            if (lightDetected > IR_THRESHOLD7){
                THRESHOLD7PASSED = true;
            } else {
                THRESHOLD7PASSED = false;
            }

            if (lightDetected > IR_THRESHOLD8){
                THRESHOLD8PASSED = true;
            } else {
                THRESHOLD8PASSED = false;
            }

            if (lightDetected > IR_THRESHOLD9){
                THRESHOLD9PASSED = true;
            } else {
                THRESHOLD9PASSED = false;
            }

        }

    }//finish the code

package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


public class DistanceSensorTest extends OpMode{

    OpticalDistanceSensor floorIR;

    boolean bluedetected = false;
    boolean redDetected = false;
    boolean whiteDetected = false;
    boolean greyDetected = false;

    String colorDetected = "no color detected";

    @Override
    public void init(){
        floorIR = hardwareMap.opticalDistanceSensor.get("ir");
    }

    @Override
    public void loop(){
        double reflectance = floorIR.getLightDetected();
        telemetry.addData("Reflectance", reflectance);

        double rawValue = floorIR.getLightDetectedRaw();
        telemetry.addData("RawLight", rawValue);

        if ((reflectance >= 125)||(reflectance <= 153)){
            bluedetected = true;
            colorDetected = "blue";
        } else {
            bluedetected = false;
        }

        if ((reflectance >= 532)||(reflectance <= 632)){
            redDetected = true;
            colorDetected = "red";

        } else {
            redDetected = false;
        }

        if ((reflectance >= 700)||(reflectance <= 750)){
            whiteDetected = true;
            colorDetected = "white";
        } else {
            whiteDetected = false;
        }

        if ((reflectance >= 108)||(reflectance <= 125)){
            greyDetected = true;
            colorDetected = "tile";
        } else {
            greyDetected = false;
        }

        telemetry.addData("Color Detected:", colorDetected);
    }
}

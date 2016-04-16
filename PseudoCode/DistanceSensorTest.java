package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


public class DistanceSensorTest extends OpMode{

    OpticalDistanceSensor floorIR;
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
    }
}

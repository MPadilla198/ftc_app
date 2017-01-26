package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Miguel Padilla on 11/12/16.
 */

@Autonomous(name="RangeTester", group="AutoOp")
public class RangeTester extends LinearOpMode {

    // Sensors
    private ColorSensor colorSensor2;

    // Setup for range sensor because it's stupid and I hate it......
    byte[] range1Cache;
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14);
    public static final int RANGE1_REG_START = 0x04;
    public static final int RANGE1_READ_LENGTH = 2;
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        // Retrieve sensor ports
        colorSensor2 = hardwareMap.colorSensor.get("cs2");
        colorSensor2.enableLed(false);

        // Initialzing range sensor stuff
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        ////////////////////////////////
        // Start Program
        ////////////////////////////////
        while (true) {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            telemetry.addData("Red: ", colorSensor2.red());
            telemetry.addData("Green: ", colorSensor2.green());
            telemetry.addData("Blue: ", colorSensor2.blue());
            telemetry.addData("Ultra Sonic: ", range1Cache[0] & 0xFF);
            telemetry.addData("ODS: ", range1Cache[1] & 0xFF);
            telemetry.update();
        }
    }
}

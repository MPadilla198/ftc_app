package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Miguel Padilla on 11/12/16.
 */

@Autonomous(name="BasicAutoBlue", group="AutoOp")
public class BasicAutoL extends LinearOpMode {

    // Robot dimmensions
    private final static double ticksPerInch = 2000.0/28.625; // ticks per inch

    private int brEncoderPosition = 0;
    private int blEncoderPosition = 0;
    private int frEncoderPosition = 0;
    private int flEncoderPosition = 0;

    // Instantiate Motor objects
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;

    // Sensors
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private ColorSensor colorSensor3;
    private ColorSensor whiteLineFinder;

    // Setup for range sensor because it's stupid and I hate it......
    byte[] range1Cache;
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14);
    private static final int RANGE1_REG_START = 0x04;
    private static final int RANGE1_READ_LENGTH = 2;
    private I2cDevice RANGE1;
    private I2cDeviceSynch RANGE1Reader;

    private enum travelDir {
        NORTH(1, -1, 1, -1),
        NORTHEAST(1, 0, 0, -1),
        EAST(-1, -1, 1, 1),
        SOUTHEAST(0, -1, 1, 0),
        SOUTH(-1, 1, -1, 1),
        SOUTHWEST(-1, 0, 0, 1),
        WEST(1, 1, -1, -1),
        NORTHWEST(0, 1, -1 , 0);

        public final int brVal, blVal, frVal, flVal;

        travelDir(int br, int bl, int fr, int fl) {
            this.brVal = br;
            this.blVal = bl;
            this.frVal = fr;
            this.flVal = fl;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        // Retrieve motor ports
        backLeftMotor = hardwareMap.dcMotor.get("bl"); // 1.0 == clock-wise
        backRightMotor = hardwareMap.dcMotor.get("br"); // 1.0 == clock-wise
        frontLeftMotor = hardwareMap.dcMotor.get("fl"); // 1.0 == clock-wise
        frontRightMotor = hardwareMap.dcMotor.get("fr"); // 1.0 == clock-wise

        // Retrieve sensor ports
        colorSensor1 = hardwareMap.colorSensor.get("cs1");
        colorSensor1.enableLed(false);
        colorSensor2 = hardwareMap.colorSensor.get("cs2");
        colorSensor2.enableLed(false);
        colorSensor3 = hardwareMap.colorSensor.get("cs3");
        colorSensor3.enableLed(false);
        whiteLineFinder = hardwareMap.colorSensor.get("wlf");
        whiteLineFinder.enableLed(true);

        // Initialzing range sensor stuff
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        // Resets encoders, but this cuts power to the motors
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Opens power to the motors
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ////////////////////////////////
        // Start Program
        ////////////////////////////////
        startWhiteLineTravel();

        // Some way to travel to the wall and stop before it hits; slightly less than 3 inches away
        travelToWall();

        linearTravel((int) (ticksPerInch * 1.5), 0.2, travelDir.NORTH);

        // Checks colors and hits blue button
        if (colorSensor2.blue() > colorSensor2.red() && colorSensor1.blue() > colorSensor1.red()) {
            hitAndCheckButton1();
            linearTravel((int)(ticksPerInch * 5.5), 0.2, travelDir.SOUTH);
        } else {
            linearTravel((int)(ticksPerInch * 5.5), 0.2, travelDir.SOUTH);
            hitAndCheckButton2();
        }

        // travels close to the beacon then uses the color sensor to get it perfectly on the white line and lined up with the button
        sinusoidalTravel((int)(ticksPerInch * 42), 1.0, travelDir.SOUTH);
        secondWhiteLineTravel();
        linearTravel((int)(ticksPerInch * 2.5), 0.2, travelDir.NORTH);

        if (colorSensor2.blue() > colorSensor2.red() && colorSensor1.blue() > colorSensor1.red()) {
            hitAndCheckButton1();
            linearTravel((int)(ticksPerInch * 5.5), 0.2, travelDir.SOUTH);
        } else {
            linearTravel((int)(ticksPerInch * 5.5), 0.2, travelDir.SOUTH);
            hitAndCheckButton2();
        }

        linearTravel((int)(ticksPerInch * 72 * Math.sqrt(2)), 1.0, travelDir.NORTHWEST);

        stop();
    }

    private void startWhiteLineTravel() {
        // North East
        // Goes diagnally straight towards beacon until it finds the white line
        blEncoderPosition -= ticksPerInch * 200; // 200 is arbitrary doesn't matter
        frEncoderPosition += ticksPerInch * 200;

        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);

        backLeftMotor.setPower(-0.2);
        frontRightMotor.setPower(0.2);

        int currentColor = (whiteLineFinder.blue() + whiteLineFinder.red() + whiteLineFinder.green()) / 3;
        int lastColor = (whiteLineFinder.blue() + whiteLineFinder.red() + whiteLineFinder.green()) / 3;

        while (Math.abs(currentColor - lastColor) < 5) {
            lastColor = currentColor;
            currentColor = (whiteLineFinder.blue() + whiteLineFinder.red() + whiteLineFinder.green()) / 3;
        }

        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        blEncoderPosition = backLeftMotor.getCurrentPosition(); // Not sure if currentPosition is the right method name for finding current encoder value
        frEncoderPosition = frontLeftMotor.getCurrentPosition();
    }
    
    private void travelToWall() {
        final double speed = 0.2;
        final travelDir dir = travelDir.EAST;
        final int dDistance = (int)(ticksPerInch * 100); // Arbitrary

        brEncoderPosition += dir.brVal * dDistance;
        blEncoderPosition += dir.blVal * dDistance;
        frEncoderPosition += dir.frVal * dDistance;
        flEncoderPosition += dir.flVal * dDistance;

        backRightMotor.setTargetPosition(brEncoderPosition);
        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);
        frontLeftMotor.setTargetPosition(flEncoderPosition);

        backRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        
        while (RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH)[0] > 15) {
            // (:
        }

        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        brEncoderPosition = backRightMotor.getCurrentPosition();
        blEncoderPosition = backLeftMotor.getCurrentPosition();
        frEncoderPosition = frontRightMotor.getCurrentPosition();
        flEncoderPosition = frontLeftMotor.getCurrentPosition();
    }

    private void secondWhiteLineTravel() {
        final double speed = 0.2;
        final travelDir dir = travelDir.SOUTH;
        final int dDistance = (int)(ticksPerInch * 100); // Arbitrary

        brEncoderPosition += dir.brVal * dDistance;
        blEncoderPosition += dir.blVal * dDistance;
        frEncoderPosition += dir.frVal * dDistance;
        flEncoderPosition += dir.flVal * dDistance;

        backRightMotor.setTargetPosition(brEncoderPosition);
        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);
        frontLeftMotor.setTargetPosition(flEncoderPosition);

        backRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);

        int currentColor = (whiteLineFinder.blue() + whiteLineFinder.red() + whiteLineFinder.green()) / 3;
        int lastColor = (whiteLineFinder.blue() + whiteLineFinder.red() + whiteLineFinder.green()) / 3;

        while(Math.abs(lastColor - currentColor) < 5) {
            // (:
            lastColor = currentColor;
            currentColor = (whiteLineFinder.blue() + whiteLineFinder.red() + whiteLineFinder.green()) / 3;
        }

        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        brEncoderPosition = backRightMotor.getCurrentPosition();
        blEncoderPosition = backLeftMotor.getCurrentPosition();
        frEncoderPosition = frontRightMotor.getCurrentPosition();
        flEncoderPosition = frontLeftMotor.getCurrentPosition();
    }

    private void sinusoidalTravel(int dDistance, double maxSpeed, travelDir dir) {
        maxSpeed -= 0.1;

        final int brInitPos = brEncoderPosition;
        final int blInitPos = blEncoderPosition;
        final int frInitPos = frEncoderPosition;
        final int flInitPos = flEncoderPosition;

        brEncoderPosition += dir.brVal * dDistance;
        blEncoderPosition += dir.blVal * dDistance;
        frEncoderPosition += dir.frVal * dDistance;
        flEncoderPosition += dir.flVal * dDistance;

        backRightMotor.setTargetPosition(brEncoderPosition);
        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);
        frontLeftMotor.setTargetPosition(flEncoderPosition);

        do {
            // (:
            double brCurrentSpeed = maxSpeed * Math.sin(Math.PI * ((double) (brEncoderPosition - brInitPos) / (double) dDistance + 0.1));
            double blCurrentSpeed = maxSpeed * Math.sin(Math.PI * ((double) (blEncoderPosition - blInitPos) / (double) dDistance + 0.1));
            double frCurrentSpeed = maxSpeed * Math.sin(Math.PI * ((double) (frEncoderPosition - frInitPos) / (double) dDistance + 0.1));
            double flCurrentSpeed = maxSpeed * Math.sin(Math.PI * ((double) (flEncoderPosition - flInitPos) / (double) dDistance + 0.1));

            backRightMotor.setPower(brCurrentSpeed);
            backLeftMotor.setPower(blCurrentSpeed);
            frontRightMotor.setPower(frCurrentSpeed);
            frontLeftMotor.setPower(flCurrentSpeed);

        } while(backRightMotor.isBusy() ||
                backLeftMotor.isBusy() ||
                frontRightMotor.isBusy() ||
                frontLeftMotor.isBusy());
    }

    private void linearTravel(int dDistance, double speed, travelDir dir) {
        brEncoderPosition += dir.brVal * dDistance;
        blEncoderPosition += dir.blVal * dDistance;
        frEncoderPosition += dir.frVal * dDistance;
        flEncoderPosition += dir.flVal * dDistance;

        backRightMotor.setTargetPosition(brEncoderPosition);
        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);
        frontLeftMotor.setTargetPosition(flEncoderPosition);

        backRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);

        do {
            // (:
        } while(backRightMotor.isBusy() ||
                backLeftMotor.isBusy() ||
                frontRightMotor.isBusy() ||
                frontLeftMotor.isBusy());
    }

    private void hitAndCheckButton1() {
        do {
            linearTravel(3, 0.25, travelDir.EAST);
            linearTravel(3, 0.25, travelDir.WEST);
        } while (colorSensor1.blue() > colorSensor1.red() && colorSensor2.blue() > colorSensor2.red());
    }

    private void hitAndCheckButton2() {
        do {
            linearTravel(3, 0.25, travelDir.EAST);
            linearTravel(3, 0.25, travelDir.WEST);
        } while (colorSensor3.blue() > colorSensor3.red() && colorSensor2.blue() > colorSensor2.red());
    }
/*
    private void addAndUpdateData(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }*/
}

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
      travelDir(br, bl, fr, fl) {
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
        backLeftMotor = hardwareMap.dcMotor.get("br"); // 1.0 == clock-wise
        backRightMotor = hardwareMap.dcMotor.get("bl"); // 1.0 == clock-wise
        frontLeftMotor = hardwareMap.dcMotor.get("fr"); // 1.0 == clock-wise
        frontRightMotor = hardwareMap.dcMotor.get("fl"); // 1.0 == clock-wise
        collectorMotor = hardwareMap.dcMotor.get("cl");

        //backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);


        colorSensor1 = hardwareMap.colorSensor.get("cs1");
        colorSensor1.enableLed(false);
        colorSensor2 = hardwareMap.colorSensor.get("cs2");
        colorSensor2.enableLed(false);
        colorSensor3 = hardwareMap.colorSensor.get("cs3");
        colorSensor3.enableLed(false);
        whiteLineFinder = hardwareMap.colorSensor.get("wlf");
        whiteLineFinder.enableLed(true);

        //possibly have to set direction for motors in reverse


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

        // North East
        // Goes diagnally straight towards beacon
        blEncoderPosition -= ticksPerInch * (48 * Math.sqrt(2) * (48.0/37.0) + 1.5); // The 48/37 is the result of a change in inches per tick because of omniwheel drive train structure
        frEncoderPosition += ticksPerInch * (48 * Math.sqrt(2) * (48.0/37.0) + 1.5); // -5 is for error

        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);

        backLeftMotor.setPower(-0.175);
        frontRightMotor.setPower(0.175);

        while (backLeftMotor.isBusy() || frontRightMotor.isBusy()){
        }

        // West
        // Just hit button and is backing away an inch
        flEncoderPosition -= ticksPerInch * 3;
        blEncoderPosition += ticksPerInch * 3;
        frEncoderPosition -= ticksPerInch * 3;
        brEncoderPosition += ticksPerInch * 3;

        frontLeftMotor.setTargetPosition(flEncoderPosition);
        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);
        backRightMotor.setTargetPosition(brEncoderPosition);

        frontLeftMotor.setPower(-0.2);
        backLeftMotor.setPower(0.2);
        frontRightMotor.setPower(-0.2);
        backRightMotor.setPower(0.2);

        while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
        }

        if (colorSensor.blue() < colorSensor.red()) {
            sleep(4500);

            // East
            // Goes towards button to hit it
            flEncoderPosition += ticksPerInch * 3;
            blEncoderPosition -= ticksPerInch * 3;
            frEncoderPosition += ticksPerInch * 3;
            brEncoderPosition -= ticksPerInch * 3;

            frontLeftMotor.setTargetPosition(flEncoderPosition);
            backLeftMotor.setTargetPosition(blEncoderPosition);
            frontRightMotor.setTargetPosition(frEncoderPosition);
            backRightMotor.setTargetPosition(brEncoderPosition);

            frontLeftMotor.setPower(0.1);
            backLeftMotor.setPower(-0.1);
            frontRightMotor.setPower(0.1);
            backRightMotor.setPower(-0.1);

            while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
            }

            // West
            // Just hit button and is backing away an inch
            flEncoderPosition -= ticksPerInch * 3;
            blEncoderPosition += ticksPerInch * 3;
            frEncoderPosition -= ticksPerInch * 3;
            brEncoderPosition += ticksPerInch * 3;

            frontLeftMotor.setTargetPosition(flEncoderPosition);
            backLeftMotor.setTargetPosition(blEncoderPosition);
            frontRightMotor.setTargetPosition(frEncoderPosition);
            backRightMotor.setTargetPosition(brEncoderPosition);

            frontLeftMotor.setPower(-0.2);
            backLeftMotor.setPower(0.2);
            frontRightMotor.setPower(-0.2);
            backRightMotor.setPower(0.2);

            while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
            }
        }

        // North
        // Moves forward 48 inches to hit the next beacon
        flEncoderPosition -= ticksPerInch * 45;
        blEncoderPosition -= ticksPerInch * 45;
        frEncoderPosition += ticksPerInch * 45;
        brEncoderPosition += ticksPerInch * 45;

        frontLeftMotor.setTargetPosition(flEncoderPosition);
        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);
        backRightMotor.setTargetPosition(brEncoderPosition);

        frontLeftMotor.setPower(-0.15);
        backLeftMotor.setPower(-0.15);
        frontRightMotor.setPower(0.15);
        backRightMotor.setPower(0.15);

        while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
        }

        /*55
        // Goes towards button to get closer
        flEncoderPosition += ticksPerInch * 4;
        blEncoderPosition -= ticksPerInch * 4;
        frEncoderPosition += ticksPerInch * 4;
        brEncoderPosition -= ticksPerInch * 4;

        frontLeftMotor.setTargetPosition(flEncoderPosition);
        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);
        backRightMotor.setTargetPosition(brEncoderPosition);

        frontLeftMotor.setPower(0.1);
        backLeftMotor.setPower(-0.1);
        frontRightMotor.setPower(0.1);
        backRightMotor.setPower(-0.1);

        while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
        }*/

        //
        // Hitting beacon 2
        //

        if (colorSensor.blue() > colorSensor.red()) {
            // East
            // Goes towards button to hit it
            flEncoderPosition += ticksPerInch * 3;
            blEncoderPosition -= ticksPerInch * 3;
            frEncoderPosition += ticksPerInch * 3;
            brEncoderPosition -= ticksPerInch * 3;

            frontLeftMotor.setTargetPosition(flEncoderPosition);
            backLeftMotor.setTargetPosition(blEncoderPosition);
            frontRightMotor.setTargetPosition(frEncoderPosition);
            backRightMotor.setTargetPosition(brEncoderPosition);

            frontLeftMotor.setPower(0.1);
            backLeftMotor.setPower(-0.1);
            frontRightMotor.setPower(0.1);
            backRightMotor.setPower(-0.1);

            while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
            }

            // Increase position to offset for not having to hit the second button
            flEncoderPosition -= ticksPerInch * 7.5;
            blEncoderPosition -= ticksPerInch * 7.5;
            frEncoderPosition += ticksPerInch * 7.5;
            brEncoderPosition += ticksPerInch * 7.5;
        } else {
            // North
            // Moves forward 5.5 inches to hit the next button
            flEncoderPosition -= ticksPerInch * 7.5;
            blEncoderPosition -= ticksPerInch * 7.5;
            frEncoderPosition += ticksPerInch * 7.5;
            brEncoderPosition += ticksPerInch * 7.5;

            frontLeftMotor.setTargetPosition(flEncoderPosition);
            backLeftMotor.setTargetPosition(blEncoderPosition);
            frontRightMotor.setTargetPosition(frEncoderPosition);
            backRightMotor.setTargetPosition(brEncoderPosition);

            frontLeftMotor.setPower(-0.15);
            backLeftMotor.setPower(-0.15);
            frontRightMotor.setPower(0.15);
            backRightMotor.setPower(0.15);

            while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
            }
            
            // East
            // Goes towards button to hit it
            flEncoderPosition += ticksPerInch * 3;
            blEncoderPosition -= ticksPerInch * 3;
            frEncoderPosition += ticksPerInch * 3;
            brEncoderPosition -= ticksPerInch * 3;

            frontLeftMotor.setTargetPosition(flEncoderPosition);
            backLeftMotor.setTargetPosition(blEncoderPosition);
            frontRightMotor.setTargetPosition(frEncoderPosition);
            backRightMotor.setTargetPosition(brEncoderPosition);

            frontLeftMotor.setPower(0.1);
            backLeftMotor.setPower(-0.1);
            frontRightMotor.setPower(0.1);
            backRightMotor.setPower(-0.1);

            while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
            }
        }

        // West
        // Just hit button and is backing away to charge corner
        flEncoderPosition -= ticksPerInch * 8;
        blEncoderPosition += ticksPerInch * 8;
        frEncoderPosition -= ticksPerInch * 8;
        brEncoderPosition += ticksPerInch * 8;

        frontLeftMotor.setTargetPosition(flEncoderPosition);
        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);
        backRightMotor.setTargetPosition(brEncoderPosition);

        frontLeftMotor.setPower(-0.15);
        backLeftMotor.setPower(0.15);
        frontRightMotor.setPower(-0.15);
        backRightMotor.setPower(0.15);

        while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
        }

        // North
        // Rushes toward center vortex
        flEncoderPosition -= ticksPerInch * 6;
        blEncoderPosition -= ticksPerInch * 6;
        frEncoderPosition += ticksPerInch * 6;
        brEncoderPosition += ticksPerInch * 6;

        frontLeftMotor.setTargetPosition(flEncoderPosition);
        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);
        backRightMotor.setTargetPosition(brEncoderPosition);

        frontLeftMotor.setPower(-1);
        backLeftMotor.setPower(-1);
        frontRightMotor.setPower(1);
        backRightMotor.setPower(1);

        while (frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()){
        }

        // South West
        blEncoderPosition += ticksPerInch * (72 * Math.sqrt(2) * (48.0/37.0) - 0); // The 48/37 is the result of a change in inches per tick because of omniwheel drive train structure
        frEncoderPosition -= ticksPerInch * (72 * Math.sqrt(2) * (48.0/37.0) - 0); // -5 is for error

        backLeftMotor.setTargetPosition(blEncoderPosition);
        frontRightMotor.setTargetPosition(frEncoderPosition);

        backLeftMotor.setPower(1.0);
        frontRightMotor.setPower(-1.0);

        while (backLeftMotor.isBusy() || frontRightMotor.isBusy()){
        }

        stop();
    }
    
    private void sinusoidalTravel(int dDistance, double maxSpeed, travelDir dir) {
      int brInitPos = brEncoderPosition;
      int blInitPos = blEncoderPosition;
      int frInitPos = frEncoderPosition;
      int flInitPos = flEncoderPosition;
      
      brEncoderPosition += dir.brVal * dDistance;
      blEncoderPosition += dir.blVal * dDistance;
      frEncoderPosition += dir.frVal * dDistance;
      flEncoderPosition += dir.flVal * dDistance;
      
      do {
        // (:
        
        double brCurrentSpeed = maxSpeed * Math.sin(Math.PI * (brEncoderPosition - brInitPos) / (double) dDistance)
      } while(backRightMotor.isBusy() ||
              backLeftMotor.isBusy() ||
              frontRightMotor.isbusy() ||
              frontLeftMotor.isBusy());
    }

    private void addAndUpdateData(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }
}

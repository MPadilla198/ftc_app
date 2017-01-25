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
        backLeftMotor = hardwareMap.dcMotor.get("bl"); // 1.0 == clock-wise
        backRightMotor = hardwareMap.dcMotor.get("br"); // 1.0 == clock-wise
        frontLeftMotor = hardwareMap.dcMotor.get("fl"); // 1.0 == clock-wise
        frontRightMotor = hardwareMap.dcMotor.get("fr"); // 1.0 == clock-wise

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

        startWhiteLineTravel();

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
		
		double currentColor = (whiteLineFinder.blue() + whiteLineFinder.red() + whiteLineFinder.green()) / 3.0;
		double lastColor = (whiteLineFinder.blue() + whiteLineFinder.red() + whiteLineFinder.green()) / 3.0;

        while (Math.abs(currentColor - lastColor) < 3) {
			lastColor = currentColor;
			currentColor = (whiteLineFinder.blue() + whiteLineFinder.red() + whiteLineFinder.green()) / 3.0;
        }
		
		backLeftMotor.setPower(0);
		frontRightMotor.setPower(0);
		
		blEncoderPosition = backLeftMotor.currentPosition(); // Not sure if currentPosition is the right method name for finding current encoder value
		frEncoderPosition = frontLeftMotor.currentPosition(); 
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
		
		backRightMotor.setPosition(brEncoderPosition);
		backLeftMotor.setPosition(blEncoderPosition);
		frontRightMotor.setPosition(frEncoderPosition);
		frontLeftMotor.setPosition(flEncoderPosition);
      
      	do {
        	// (:
        
        	double brCurrentSpeed = maxSpeed * Math.sin(Math.PI * ((double) (brEncoderPosition - brInitPos) / (double) dDistance + 0.1));
        	double blCurrentSpeed = maxSpeed * Math.sin(Math.PI * ((double) (blEncoderPosition - blInitPos) / (double) dDistance + 0.1));
        	double bfCurrentSpeed = maxSpeed * Math.sin(Math.PI * ((double) (frEncoderPosition - frInitPos) / (double) dDistance + 0.1));
        	double flCurrentSpeed = maxSpeed * Math.sin(Math.PI * ((double) (flEncoderPosition - flInitPos) / (double) dDistance + 0.1));
			
			backRightMotor.setPower(brCurrentSpeed);
			backLeftMotor.setPower(blCurrentSpeed);
			frontRightMotor.setPower(frCurrentSpeed);
			frontLeftMotor.setPower(flCurrentSpeed);
			
      	} while(backRightMotor.isBusy() ||
              	backLeftMotor.isBusy() ||
              	frontRightMotor.isbusy() ||
              	frontLeftMotor.isBusy());
    }
	
	private void linearTravel(int dDistance, double speed, travelDir dir) {
		maxSpeed -= 0.1;
		
      	final int brInitPos = brEncoderPosition;
      	final int blInitPos = blEncoderPosition;
      	final int frInitPos = frEncoderPosition;
      	final int flInitPos = flEncoderPosition;
      
      	brEncoderPosition += dir.brVal * dDistance;
      	blEncoderPosition += dir.blVal * dDistance;
      	frEncoderPosition += dir.frVal * dDistance;
      	flEncoderPosition += dir.flVal * dDistance;
		
		backRightMotor.setPosition(brEncoderPosition);
		backLeftMotor.setPosition(blEncoderPosition);
		frontRightMotor.setPosition(frEncoderPosition);
		frontLeftMotor.setPosition(flEncoderPosition);
		
		backRightMotor.setPower(speed);
		backLeftMotor.setPower(speed);
		frontRightMotor.setPower(speed);
		frontLeftMotor.setPower(speed);
      
      	do {
        	// (:		
      	} while(backRightMotor.isBusy() ||
              	backLeftMotor.isBusy() ||
              	frontRightMotor.isbusy() ||
              	frontLeftMotor.isBusy());
	}
	
	private void checkOrHitButton1() {
		while (colorSensor1.blue() > colorSensor1.red() && colorSensor2.blue() > colorSensor2.red()) {
			linearTravel(3, 0.25, travelDir.EAST);
			linearTravel(3, 0.25, travelDir.WEST);
		}
	}
	
	private void checkOrHitButton2() { 
		while (colorSensor3.blue() > colorSensor3.red() && colorSensor2.blue() > colorSensor2.red()) {
			linearTravel(3, 0.25, travelDir.EAST);
			linearTravel(3, 0.25, travelDir.WEST);
		}
	}

    private void addAndUpdateData(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }
}

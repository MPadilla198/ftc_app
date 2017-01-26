import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;
/**
 *This is a test program to see how accurate range sensors are.
 */
@TeleOp(name = "RangeTester")
public class RangeTester extends SynchronousOpMode {
 
    @Override
    public void main() throws InterruptedException {
        I2cDevice range;
        range = hardwareMap.i2cDevice.get("range");
        I2cDeviceReader rangeReader = new I2cDeviceReader(range, 0x28, 0x04, 2);
 
        telemetry.addData("readerLength", rangeReader.getReadBuffer().length);
        telemetry.update();
 
        waitForStart();
 
        while (opModeIsActive()) {
            byte rangeReadings[] = rangeReader.getReadBuffer();
            telemetry.addData("Uncalibrated: ", rangeReadings[0]);
            telemetry.addData("Calibrated: ", calibratedUltrasonic(rangeReadings[0]));
            telemetry.update();
        }
    }
    public int calibratedUltrasonic(byte reading) {
        //convert byte to int
        int uncalibratedDistance = reading;
        if (uncalibratedDistance == -1) {
            return -1;
        }
 
        int calibratedDistance;
 
        if (uncalibratedDistance >= 0) {
            return calibratedDistance = (int) (((double)uncalibratedDistance / 9) * 10);
        }
        else {
            calibratedDistance = uncalibratedDistance * -1;
            telemetry.addData("after flip: ", calibratedDistance);
            calibratedDistance = 255 - calibratedDistance;
            telemetry.addData("after 255-: ", calibratedDistance);
            return (int) (((double)calibratedDistance / 9) * 10);
        }
    }
}

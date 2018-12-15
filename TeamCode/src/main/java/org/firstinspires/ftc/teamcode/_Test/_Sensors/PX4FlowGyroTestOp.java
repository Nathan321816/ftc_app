package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.PX4Flow;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;


/**
 * Created by phanau on 12/13/18.
 * Optical flow sensor + gyro position integration test
 */
@Autonomous(name="Test: PX4FlowGyro Test 1", group ="Test")
//@Disabled
public class PX4FlowGyroTestOp extends OpMode {

    private PX4Flow mFlow;
    private int x;
    private int y;
    private SensorLib.PositionIntegrator mPosInt;
    private int mFlowBearing;
    BNO055IMUHeadingSensor mGyro;           // gyro to use for heading information

    public PX4FlowGyroTestOp() {
    }

    public void init() {
        mFlow = hardwareMap.get(PX4Flow.class, "PX4Flow");
        mPosInt = new SensorLib.PositionIntegrator();

        // get hardware IMU and wrap gyro in HeadingSensor object usable below
        mGyro = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mGyro.init(7);  // orientation of REV hub in my ratbot

    }

    public void loop() {
        // read current integrated data from sensor
        mFlow.readIntegral();

        // accumulate incremental dx, dy
        int dx = mFlow.pixel_flow_x_integral();
        int dy = mFlow.pixel_flow_y_integral();
        x += dx;
        y += dy;

        // accumulate bearing (Z-rotation)
        mFlowBearing += mFlow.gyro_z_rate_integral();

        // assume sensor is mounted with +X axis in direction of driving
        // NOTE: the axes marked on the PX4Flow circuit board appear to be reversed (x-y)

        // the camera is currently mounted on the back of the ratbot with
        // the front of the robot in the marked -Y direction and X pointing right
        // so forward motion is +X and motion to the right is +Y

        // convert bearing from int radians*10000 to double degrees and reverse direction
        // since z axis of camera points down, not up.
        double flowBearingDeg = -Math.toDegrees((double)(mFlowBearing)/10000.0);

        // get bearing from IMU gyro to compare to camera heading (appears more reliable)
        double imuBearingDeg = mGyro.getHeading();

        // update accumulated field position - note args to move(right,forward)
        mPosInt.move(dy, dx, imuBearingDeg);

        // log data to DriverStation
        telemetry.addData("count", mFlow.frame_count_since_last_readout());
        telemetry.addData("dx", dx);
        telemetry.addData("dy", dy);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        //telemetry.addData("gyro dx", mFlow.gyro_x_rate_integral());
        //telemetry.addData("gyro dy", mFlow.gyro_y_rate_integral());
        //telemetry.addData("gyro dz", mFlow.gyro_z_rate_integral());
        telemetry.addData("PX4Flow bearing(deg)", String.format("%.2f", flowBearingDeg));
        telemetry.addData("IMUgyro bearing(deg)", String.format("%.2f", imuBearingDeg));
        telemetry.addData("posInt", String.format("%.2f",mPosInt.getX())+", "+String.format("%.2f",mPosInt.getY()));
        telemetry.addData("quality", mFlow.quality_integral());
        telemetry.addData("time", mFlow.integration_timespan());
    }

    public void stop() {}

}

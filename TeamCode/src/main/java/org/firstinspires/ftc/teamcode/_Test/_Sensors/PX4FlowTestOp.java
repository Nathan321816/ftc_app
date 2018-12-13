package org.firstinspires.ftc.teamcode._Test._Sensors;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.PX4Flow;


/**
 * Created by phanau on 2/4/18.
 * Optical flow sensor hardware test
 */
@Autonomous(name="Test: PX4Flow Test 1", group ="Test")
//@Disabled
public class PX4FlowTestOp extends OpMode {

    private PX4Flow mSensor;
    private int x;
    private int y;

    public PX4FlowTestOp() {
    }

    public void init() {
        mSensor = hardwareMap.get(PX4Flow.class, "PX4Flow");
    }

    public void loop() {
        // read current integrated data from sensor
        mSensor.readIntegral();

        // accumulate incremental dx, dy
        int dx = mSensor.pixel_flow_x_integral();
        int dy = mSensor.pixel_flow_y_integral();
        x += dx;
        y += dy;

        // log data to DriverStation
        telemetry.addData("count", mSensor.frame_count_since_last_readout());
        telemetry.addData("dx", dx);
        telemetry.addData("dy", dy);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        //telemetry.addData("gyro dx", mSensor.gyro_x_rate_integral());
        //telemetry.addData("gyro dy", mSensor.gyro_y_rate_integral());
        //telemetry.addData("gyro dz", mSensor.gyro_z_rate_integral());
        telemetry.addData("quality", mSensor.quality_integral());
        telemetry.addData("time", mSensor.integration_timespan());
    }

    public void stop() {}

}

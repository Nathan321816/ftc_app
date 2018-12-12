package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.ToggleButton;

import static android.os.SystemClock.sleep;


/**
 * Created by phanau on 1/22/16.
 * Test REV Robotics RevHub IMU
 */

@Autonomous(name="Test: REV IMU Test 1", group ="Test")
//@Disabled
public class RevGyroTestOp extends OpMode {

    private BNO055IMUHeadingSensor mIMU;
    ToggleButton mButton;

    public RevGyroTestOp() {
    }

    public void init() {
        // get hardware IMU and wrap gyro in HeadingSensor object usable below
        mIMU = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mIMU.init(3);  // 3: upright crosswise with REV face forward

        // create a toggle-button that cycles through the N possible orientations of the IMU
        mButton = new ToggleButton(false, mIMU.numOrientations(), 0);
    }

    public void loop() {

        // use controller input X to cycle through various orientations
        if (mButton.process(gamepad1.x))
            mIMU.setOrientation(mButton.value());
        telemetry.addData("usage", "press X to cycle through orientations");
        telemetry.addData("index", mIMU.getOrientation());
        telemetry.addData("orientation", mIMU.getAngularOrientation());
        telemetry.addData("position", mIMU.getPosition());
    }

}

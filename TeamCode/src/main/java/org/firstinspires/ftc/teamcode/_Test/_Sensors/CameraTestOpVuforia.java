/**
 * CameraTestOp - a simple operating mode that tries to acquire an image from the
 * phone camera and get some data from the image
 * Created by phanau on 12/9/15.
 */

package org.firstinspires.ftc.teamcode._Test._Sensors;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.CameraLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;


@Autonomous(name="Test: CameraLib Test Vuforia", group ="Test")
//@Disabled
public class CameraTestOpVuforia extends OpMode {

    int mLoopCount;
    VuforiaLib_RoverRuckus mVLib;


    // Constructor
    public CameraTestOpVuforia() {
    }

    @Override
    public void init() {
        mVLib = new VuforiaLib_RoverRuckus();
        mVLib.init(this, null);     // pass it this OpMode (so it can do telemetry output) and use its license key for now
    }

    @Override public void start()
    {
        /** Start tracking the data sets we care about. */
        mVLib.start();
    }

    public void loop() {

        // test image access through Vuforia
        Bitmap b = mVLib.getBitmap(8);
        if (b != null) {
            CameraLib.CameraImage frame = new CameraLib.CameraImage(b);
            CameraLib.Size camSize = frame.cameraSize();
            telemetry.addData("Size", String.valueOf(camSize.width) + "x" + String.valueOf(camSize.height));
            final int bandSize = 4;         // width of each band of pixel columns below
            final float minFrac = 0.4f;     // minimum fraction of pixels in band that needs to be of same color to mark it as "dominant"
            telemetry.addData("hue columns", frame.columnHue(bandSize, null, minFrac));
            telemetry.addData("dom columns", frame.columnDomColor(bandSize, null, minFrac));
            /*
            telemetry.addData("hue a(1/3)", frame.scanlineHue(camSize.height / 3, bandSize));
            telemetry.addData("hue b(1/2)", frame.scanlineHue(camSize.height / 2, bandSize));
            telemetry.addData("hue c(2/3)", frame.scanlineHue(2*camSize.height / 3, bandSize));
            telemetry.addData("dom a(1/3)", frame.scanlineDomColor(camSize.height / 3, bandSize));
            telemetry.addData("dom b(1/2)", frame.scanlineDomColor(camSize.height / 2, bandSize));
            telemetry.addData("dom c(2/3)", frame.scanlineDomColor(2*camSize.height / 3, bandSize));
            telemetry.addData("centroid a(1/3)", frame.scanlineGray(camSize.height / 3).fCentroid());
            telemetry.addData("centroid b(1/2)", frame.scanlineGray(camSize.height / 2).fCentroid());
            telemetry.addData("centroid c(2/3)", frame.scanlineGray(2*camSize.height / 3).fCentroid());
            */
        }
    }

    public void stop() {
        mVLib.stop();
    }

}


/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode._Test._Sensors;

import android.graphics.Bitmap;
import android.graphics.RectF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode._Libs.CameraLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_3D;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2016;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;

import java.nio.ByteBuffer;

/**
 * This OpMode is an attempt to use the Vuforia library to determine
 * positioning and orientation of the red and blue cryptoboxes from the 2017-8 game on the FTC field.
 * It uses a generalized form of our Vuforia wrapper class (VuforiaLib_3D) that lets the client
 * specify the names of the various images to be recognized, and assumes the client is supplying
 * their own database (.dat and .xml files) with the given name (here "CryptoRedBlue1").
 */

@Autonomous(name="Test: Vuforia 3D Test 1", group ="Test")
@Disabled
public class Vuforia3DTest1 extends OpMode {

    VuforiaLib_3D mVLib;

    @Override public void init() {
        mVLib = new VuforiaLib_3D();
        String names[] = { "red2", "red1", "blue1", "blue2" };
        mVLib.init(this, null, "CryptoRedBlue1", names);
        // pass it this OpMode (so it can do telemetry output) and use its license key for now
    }

    @Override public void start()
    {
        /** Start tracking the data sets we care about. */
        mVLib.start();
    }

    @Override public void loop()
    {
        mVLib.loop();       // update location info and do debug telemetry

        telemetry.addData("Visible", mVLib.getName());

        OpenGLMatrix lastLocation = mVLib.getLastLocation();
        if (mVLib.haveLocation())
            telemetry.addData("Position", VuforiaLib_3D.formatPosition(lastLocation));
        else
            telemetry.addData("Position", "Unknown");
        if (mVLib.haveHeading())
            telemetry.addData("Orientation", VuforiaLib_3D.formatOrientation(lastLocation));
        else
            telemetry.addData("Orientation", "Unknown");

        // test image access through Vuforia
        RectF crop = new RectF(0.1f, 0.1f, 0.9f, 0.9f);
        Bitmap b = mVLib.getBitmap(crop, 4);
        if (b != null) {
            CameraLib.CameraImage frame = new CameraLib.CameraImage(b);
            CameraLib.Size camSize = frame.cameraSize();
            telemetry.addData("Size", String.valueOf(camSize.width) + "x" + String.valueOf(camSize.height));
            final int bandSize = 4;
            telemetry.addData("hue   ", frame.columnHue(bandSize));
        }

    }

    @Override public void stop()
    {
        mVLib.stop();
    }

}

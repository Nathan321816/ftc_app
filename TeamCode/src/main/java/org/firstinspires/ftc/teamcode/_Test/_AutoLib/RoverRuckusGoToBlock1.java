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
package org.firstinspires.ftc.teamcode._Test._AutoLib;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.RectF;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BlobFinder;
import org.firstinspires.ftc.teamcode._Libs.RS_Posterize;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.SetPosterizer;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;

import java.util.ArrayList;


/**
 * This OpMode uses a Step that uses images from the VuforiaLib_RoverRuckus library to determine
 * the position of the orange block and move toward it to knock it off its perch.
 */



// this is a guide step that uses camera image data to
// guide the robot to the indicated bin of the cryptobox
//
class GoToBlockGuideStep extends AutoLib.MotorGuideStep implements SetPosterizer {

    VuforiaLib_RoverRuckus mVLib;
    OpMode mOpMode;

    // robot-specific settings - where is your camera relative to the block you're trying to place?
    // if the camera is on the right side of the block, then mCameraOnRight=true and mCameraOffset is positive.
    // if the camera is on the left side of the block, then mCameraOnRight=false and mCameraOffset is negative.
    // if the camera is above the block, then set mCameraOnRight=false and set mCameraOffset to the positive offset of the camera from the left edge of the block.
    final boolean mCameraOnRight = false;     // e.g. orientation of phone on robot (assumed landscape) is with camera on the left
    final float mCameraOffset = -0.5f;        // e.g. camera lens is 0.5 x bin width to the left of the center of the cryptobox column on the block's left side

    final int minDoneCount = 2;         // require "done" test to succeed this many consecutive times
    int mDoneCount;

    SetBitmap mSBM;                     // interface through which we tell the controlling opMode about our Bitmap so it can display it
    Bitmap mBmOut;                      // processed bitmap we post to RC phone screen

    RS_Posterize mRsPosterizer;         // RenderScript process used to posterize images

    Paint mPaintRed, mPaintGreen, mPaintBlue;   // used to draw info overlays on image

    AutoLib.MotorGuideStep mMotorGuideStep;     // step used to actually control the motors based on directives from this step and gyro input

    boolean mbLookForBlue;

    public GoToBlockGuideStep(OpMode opMode, SetBitmap sbm, VuforiaLib_RoverRuckus VLib, AutoLib.MotorGuideStep motorGuideStep) {
        mOpMode = opMode;
        mVLib = VLib;
        mMotorGuideStep = motorGuideStep;
        mDoneCount = 0;
        mSBM = sbm;

        // create stuff we'll use to draw debug info overlays on the image
        mPaintRed = new Paint();
        mPaintRed.setColor(Color.RED);
        mPaintRed.setStyle(Paint.Style.STROKE);
        mPaintGreen = new Paint();
        mPaintGreen.setColor(Color.GREEN);
        mPaintGreen.setStyle(Paint.Style.STROKE);
        mPaintBlue = new Paint();
        mPaintBlue.setColor(Color.BLUE);
        mPaintBlue.setStyle(Paint.Style.STROKE);
    }

    public void set(ArrayList<AutoLib.SetPower> motorSteps){
        mMotorGuideStep.set(motorSteps);
    }

    public void setPosterizer(RS_Posterize posterizer) {
        mRsPosterizer = posterizer;
    }

    public boolean loop() {
        super.loop();

        // get most recent frame from camera (through Vuforia)
        RectF rect = mCameraOnRight ? new RectF(0,0f,1.0f,0.5f)        // use bottom half of frame
                                    : new RectF(0,0.5f,1.0f,1.0f);
        Bitmap bmIn = mVLib.getBitmap(rect, 4);                      // get cropped, downsampled image from Vuforia

        if (bmIn != null) {

            mOpMode.telemetry.addData("image size", "%d x %d", bmIn.getWidth(), bmIn.getHeight());

            // create the output bitmap for the posterization RenderScript
            Bitmap bmOut = Bitmap.createBitmap(bmIn.getWidth(), bmIn.getHeight(), Bitmap.Config.RGB_565);

            // do some processing on the input bitmap in RenderScript to generate the output image
            mRsPosterizer.runScript(bmIn, bmOut);

            // optionally rotate image 180 degrees if phone orientation makes it upside down
            if (mCameraOnRight)
                mBmOut = RotateBitmap(bmOut, 180);
            else
                mBmOut = bmOut;

            // do some data extraction on the raw and processed bitmaps
            mOpMode.telemetry.addData("Size", String.valueOf(bmIn.getWidth()) + "x" + String.valueOf(bmIn.getHeight()));
            mOpMode.telemetry.addData("Center In", String.format("0x%08x", bmIn.getPixel(bmIn.getWidth()/2, bmIn.getHeight()/2)));
            mOpMode.telemetry.addData("Center Out", String.format("0x%08x", mBmOut.getPixel(mBmOut.getWidth()/2, mBmOut.getHeight()/2)));

            BlobFinder bf = new BlobFinder(mBmOut);
            final int sample = 4;       // look for blobs at every Nth pixel
            final int blobColor = 0xFFFFFF00;
            int count = bf.find(blobColor, sample);    // posterized yellow value
            Point centroid = bf.getCentroid();
            mOpMode.telemetry.addData("blob count", count);
            mOpMode.telemetry.addData("centroid", centroid.x + "," + centroid.y);

            Point blobMin = bf.getBoundsMin();
            Point blobMax = bf.getBoundsMax();

            // determine if detected blob has an aspect ratio that is reasonable for a block
            int dx = Math.abs(blobMin.x-blobMax.x);
            int dy = Math.abs(blobMin.y-blobMax.y);
            float maxAspect = 1.6f;
            boolean bAspectOkay = (dx < maxAspect*dy) && (dy < maxAspect*dx);

            // add annotations to the bitmap showing detected blob center and limits
            final int XS = 5;     // cross size
            Canvas canvas = new Canvas(mBmOut);
            canvas.drawLine(centroid.x-XS, centroid.y, centroid.x+XS, centroid.y, mPaintGreen);
            canvas.drawLine(centroid.x, centroid.y-XS, centroid.x, centroid.y+XS, mPaintGreen);
            canvas.drawRect(blobMin.x, blobMin.y, blobMax.x, blobMax.y, bAspectOkay ? mPaintGreen : mPaintRed);

            final float tanCameraHalfFOV = 28.0f/50.0f;       // horizontal half angle FOV of S5 camera is atan(28/50) or about 29.25 degrees
            float distance = -1;                                           // distance to block in inches --- -1 means "don't know"
            float angError = 0;

            // if we're probably really seeing the block (and not some other goo in the image), estimate distance and steer toward it
            // unfortunately, this strategy also turns off guidance when we're close enough that the block is bigger than the image height.
            // so check for that, and also where the block is made more rectangular because it's off the top of bottom of the view, too.
            boolean bBlockAtEdgeOfFrameH = (blobMin.y <= 0) || (blobMax.y >= mBmOut.getHeight()-1);
            boolean bAlways = false;
            if (bAlways || bAspectOkay || bBlockAtEdgeOfFrameH) {
                // estimate distance from the block by its size in the camera image
                final int minBlockSize = 10;                                    // minimum pixel count for credible block detection
                if (count > minBlockSize) {
                    float blockSize = blobMax.x - blobMin.x;                   // width is best measure of edge length of block in pixels since height is frame-limited
                    float viewFrac = blockSize/mBmOut.getWidth();
                    distance = 2.0f / (viewFrac*2*tanCameraHalfFOV);            // distance to block given it is 2" wide;
                    mOpMode.telemetry.addData("distance (in)", distance);

                    // the above computed target point should be in the middle of the image if we're on course -
                    // if not, correct our course to center it --
                    // compute fractional error = fraction of image offset of target from center = [-1 .. +1]
                    float error = ((float)centroid.x - (float)mBmOut.getWidth()/2.0f) / ((float)mBmOut.getWidth()/2.0f);

                    // compute motor correction from error through PID --
                    // for now, convert image-pixel error to angle in degrees --
                    // image coordinates are positive to the right but angles are positive to the left (CCW)
                    angError = (float)(Math.atan(error * tanCameraHalfFOV) * 180.0/Math.PI);
                    mOpMode.telemetry.addData("data", "error=%f angError=%f", error, angError);

                    // tell the subsidiary motor guide step which way to steer -- the heading (orientation) will either be
                    // constant (squirrely wheels) or change to match the direction, depending on the motor guide step we're given.
                    ((AutoLib.SetDirectionHeadingPower)mMotorGuideStep).setRelativeDirection(angError);
                }
            }

            if (distance != -1) {             // have valid distance data ...
                if (distance < 6.0f) {       // for now, complete this step when we're close
                    // require completion test to pass some min number of times in a row to believe it
                    mDoneCount++;
                    if (mDoneCount >= minDoneCount) {
                        // return "done" -- assume the next step will stop motors if needed
                        return true;
                    }
                } else
                    mDoneCount = 0;         // reset the "done" counter
            }
            mOpMode.telemetry.addData("data", "doneCount=%d", mDoneCount);

            // tell the calling opMode about the bitmap so it can display it
            mSBM.setBitmap(mBmOut);

        }
        else
            mOpMode.telemetry.addData("getBitmap()", "no new frame");


        return false;  // haven't found anything yet
    }

    public void stop() {
    }

    public static Bitmap RotateBitmap(Bitmap source, float angle)
    {
        Matrix matrix = new Matrix();
        matrix.postRotate(angle);
        return Bitmap.createBitmap(source, 0, 0, source.getWidth(), source.getHeight(), matrix, true);
    }

}


// for now, run this directly -- later this might be subordinated to red and blue OpModes
@Autonomous(name="RoverRuckusGoToBlock1", group ="Auto")
//@Disabled
public class RoverRuckusGoToBlock1 extends OpMode implements SetBitmap {

    boolean bDone;
    AutoLib.Sequence mSequence;             // the root of the sequence tree
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    VuforiaLib_RoverRuckus mVLib;               // Vuforia wrapper object used by Steps
    RS_Posterize mRsPosterize;              // RenderScript posterization filter

    ImageView mView;
    Bitmap mBitmap; //the bitmap you will display
    //private static final Object bmLock = new Object(); //synchronization lock so we don't display and write

    AutoLib.MotorGuideStep mGuideStep;      // member variable so that init() can set it and start() can use it

    double mTime;   // time of last loop() call -- used to compute frames per second

    public void setBitmap(Bitmap b)
    {
        mBitmap = b;
    }

    boolean getHardware(AutoLib.HardwareFactory mf, DcMotor[] motors) {
        // get the motors: depending on the factory we are given, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        try {
            motors[0] = mf.getDcMotor("fr");
            motors[1] = mf.getDcMotor("br");
            motors[2] = mf.getDcMotor("fl");
            motors[3] = mf.getDcMotor("bl");
            return (motors[0]!=null && motors[1]!=null && motors[2]!=null && motors[3]!=null);
        }
        catch (Exception c) {
            return false;
        }
    }

    public void init() {
        init(false);    // for now, red/blue doesn't matter
    }

    public void init(boolean bLookForBlue)
    {
        // get the hardware
        mMotors = new DcMotor[4];
        if (!getHardware(new AutoLib.RealHardwareFactory(this), mMotors)) {
            getHardware(new AutoLib.TestHardwareFactory(this), mMotors);
        }

        boolean invertLeft = true;     // current ratbot ...
        if (invertLeft) {
            mMotors[2].setDirection(DcMotor.Direction.REVERSE);
            mMotors[3].setDirection(DcMotor.Direction.REVERSE);
        }
        else {
            mMotors[0].setDirection(DcMotor.Direction.REVERSE);
            mMotors[1].setDirection(DcMotor.Direction.REVERSE);
        }

        // best to do this now, which is called from opmode's init() function
        mVLib = new VuforiaLib_RoverRuckus();
        mVLib.init(this, null);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();
        // make a step that will steer the robot given guidance from the GoToBlockGuideStep

        // construct a PID controller for correcting heading errors
        final float Kp = 0.01f;        // degree heading proportional term correction per degree of deviation
        final float Ki = 0.005f;        // ... integrator term
        final float Kd = 0.0f;         // ... derivative term
        final float KiCutoff = 0.0f;   // maximum angle error for which we update integrator
        SensorLib.PID pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        final float power = 0.2f;
        AutoLib.MotorGuideStep motorGuideStep = new AutoLib.ErrorGuideStep(this, pid, null, power);
        // make a step that analyzes a camera image from Vuforia and steers toward the biggest orange blob, presumably the block.
        // it uses a ErrorGuideStep to process the heading error it computes into motor steering commands.
        mGuideStep  = new GoToBlockGuideStep(this, this, mVLib, motorGuideStep);
        // make and add the Step that combines all of the above to go to the orange block
        mSequence.add(new AutoLib.GuidedTerminatedDriveStep(this, mGuideStep, motorGuideStep, mMotors));
        // stop all motors
        mSequence.add(new AutoLib.MoveByTimeStep(mMotors, power, 0, true));

        mView = (ImageView)((Activity)hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.OpenCVOverlay);
        mView.post(new Runnable() {
            @Override
            public void run() {
                mView.setAlpha(1.0f);
            }
        });
        mBitmap = null;
    }

    @Override public void start()
    {
        // start out not-done
        bDone = false;

        // start Vuforia scanning
        mVLib.start();

        // create a RenderScript posterizer -- we do it here because it takes too long to do in init()
        mRsPosterize = new RS_Posterize();
        mRsPosterize.createScript(this.hardwareMap.appContext);

        // tell the camera-based guide step about the RenderScript posterizer it should use
        ((SetPosterizer)mGuideStep).setPosterizer(mRsPosterize);

        // get start time for frames/sec computation
        mTime = getRuntime();
    }

    @Override
    public void loop() {
        // log frames per second
        double dTime = getRuntime() - mTime;
        if (dTime > 0)
            telemetry.addData("frames/sec", 1.0/dTime);
        mTime = getRuntime();       // for next time ...

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");

        //display!
        mView.post(new Runnable() {
            @Override
            public void run() {
                //synchronized (bmLock) {
                    mView.setImageBitmap(mBitmap);
                    mView.invalidate();
                //}
            }
        });
    }

    @Override
    public void stop() {
        super.stop();
        if (mRsPosterize != null)
            mRsPosterize.destroyScript();
        //mVLib.stop();       // Vuforia claims to do this automatically on OpMode stop but ...
        mView.post(new Runnable() {
            @Override
            public void run() {
                mView.setAlpha(0.0f);
            }
        });     // hide the overlay window
    }

}


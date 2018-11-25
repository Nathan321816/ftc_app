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
package org.firstinspires.ftc.teamcode._Auto;

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

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BlobFinder;
import org.firstinspires.ftc.teamcode._Libs.RS_Posterize;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.SetPosterizer;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;


/**
 * This OpMode uses a Step that uses images from the VuforiaLib_RoverRuckus library to determine
 * the position of the orange block and move toward it to knock it off its perch.
 */

interface SetBitmap {
    public void setBitmap(Bitmap bm);
}

interface SetErrorAngle {
    public void setErrorAngle (float angle);
}

interface SetDistance {
    public void setDistance(float distance);
}

// a test Step that needs to pass the test multiple times to actually terminate
abstract class CountedTestStep extends AutoLib.MotorGuideStep  {
    int mDoneCount = 0;
    int mDoneLimit = 0;
    CountedTestStep(int doneLimit) {
        mDoneLimit = doneLimit;
        mDoneCount = 0;
    }
    public boolean test(boolean result) {
        if (result) {       // return true when result is true N times in a row
            mDoneCount++;
            if (mDoneCount >= mDoneLimit) {
                return true;        // return "done" to terminate the super-step
            }
        } else
            mDoneCount = 0;         // reset the "done" counter
        return false;
    }
}

// used by GoToBlockGuideStep to control termination in various different situations
abstract class CountedDistanceAngleTestStep extends CountedTestStep implements SetErrorAngle, SetDistance, AutoLib.SetMotorSteps {
    CountedDistanceAngleTestStep(int doneLimit) { super(doneLimit); }
}

class AngleTerminationStep extends CountedDistanceAngleTestStep {
    @Override
    public void setDistance(float distance) {}
    @Override
    public void setErrorAngle(float angle) { angError = angle; }
    @Override
    public void set(ArrayList<AutoLib.SetPower> motorsteps) {}

    private float angError = 0.0f;
    private float mAngleLimit;

    public AngleTerminationStep(float angle, int doneLimit) {
        super(doneLimit);
        mAngleLimit = angle;
    }

    public boolean loop() {
        // when we're more or less pointing at the block ...
        if (angError != 0)  // have valid data ...
            return test( Math.abs(angError) < mAngleLimit);
        return false;       // not done - continue running super-step
    }
}

class DistanceTerminationStep extends CountedDistanceAngleTestStep {
    @Override
    public void setDistance(float distance) { mDistance = distance; }
    @Override
    public void setErrorAngle(float angle) { }
    @Override
    public void set(ArrayList<AutoLib.SetPower> motorsteps) {}

    private float mDistance;
    private float mDistanceLimit;

    public DistanceTerminationStep(float distance, int doneLimit) {
        super(doneLimit);
        mDistanceLimit = distance;
    }

    public boolean loop() {
        // when we're really close ...
        if (mDistance > 0)              // have valid distance data ...
            return test(mDistance < mDistanceLimit);
        return false;       // not done - continue running super-step
    }
}

// this is a guide step that uses camera image data to
// guide the robot to the indicated bin of the cryptobox
//
class GoToBlockGuideStep extends AutoLib.MotorGuideStep {

    VuforiaLib_RoverRuckus mVLib;
    OpMode mOpMode;
    CountedDistanceAngleTestStep mTermTestStep;

    final int minDoneCount = 5;         // require "done" test to succeed this many consecutive times
    int mDoneCount;

    Point mBmSize = null;
    AutoLib.MotorGuideStep mMotorGuideStep;     // step used to actually control the motors based on directives from this step and gyro input

    final float tanCameraHalfFOV = 28.0f/50.0f;       // horizontal half angle FOV of S5 camera is atan(28/50) or about 29.25 degrees
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";

    public GoToBlockGuideStep(OpMode opMode, VuforiaLib_RoverRuckus VLib, AutoLib.MotorGuideStep motorGuideStep, CountedDistanceAngleTestStep termTestStep) {
        mOpMode = opMode;
        mVLib = VLib;
        mMotorGuideStep = motorGuideStep;
        mDoneCount = 0;
        mTermTestStep = termTestStep;
    }

    public void set(ArrayList<AutoLib.SetPower> motorSteps) {
        mMotorGuideStep.set(motorSteps);
    }

    public boolean loop() {
        super.loop();

        float distance = -1;
        float angError = 0;

        TFObjectDetector tfod = mVLib.getTfod();
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                mOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());

                Recognition goldMineral = null;          // detected block
                if (updatedRecognitions.size() >= 1) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineral = recognition;
                        }
                    }
                }

                // one time, get size of Vuforia image so we can compute error angle
                if (mBmSize == null)
                    mBmSize = mVLib.getBitmapSize();

                // if we have new data ...
                if (goldMineral != null && mBmSize != null) {
                    int goldMineralX = (int)(goldMineral.getRight()+goldMineral.getLeft())/2;
                    mOpMode.telemetry.addData("Gold Mineral X", goldMineralX);

                    // the target point should be in the middle of the image if we're on course -
                    // if not, correct our course to center it --
                    // compute fractional error = fraction of image offset of target from center = [-1 .. +1]
                    // get size of Vuforia image
                    int bmWidth = mBmSize.x;
                    mOpMode.telemetry.addData("bitmap width", bmWidth);
                    float error = ((float) goldMineralX - (float) bmWidth / 2.0f) / ((float) bmWidth / 2.0f);

                    float blockSize = goldMineral.getRight()-goldMineral.getLeft();
                        // width is best measure of edge length of block in pixels since height is frame-limited
                    float viewFrac = blockSize/bmWidth;
                    distance = 2.0f / (viewFrac*2*tanCameraHalfFOV);            // distance to block given it is 2" wide;

                    // compute motor correction from error through PID --
                    // for now, convert image-pixel error to angle in degrees --
                    // image coordinates are positive to the right but angles are positive to the left (CCW)
                    angError = (float) (Math.atan(error * tanCameraHalfFOV) * 180.0 / Math.PI);
                    mOpMode.telemetry.addData("data", "error=%f angError=%f", error, angError);

                    // tell the subsidiary motor guide step which way to steer -- the heading (orientation) will either be
                    // constant (squirrely wheels) or change to match the direction, depending on the motor guide step we're given.
                    ((AutoLib.SetDirectionHeadingPower) mMotorGuideStep).setRelativeDirection(angError);

                    // run the subsidiary step to actually update the subsidiary motor steps
                    mMotorGuideStep.loop();
                }
            }

        }

        // tell the termination test step about the current angular error and distance to the block
        if (mTermTestStep != null) {
            mTermTestStep.setDistance(distance);
            mTermTestStep.setErrorAngle(angError);
        }

        return true;  // let the termination step control the sequencer
    }

    public void stop() {
    }

}


// for now, run this directly -- later this might be subordinated to red and blue OpModes
@Autonomous(name="RoverRuckusGoToBlockTF", group ="Auto")
//@Disabled
public class RoverRuckusGoToBlockTF extends OpMode  {

    boolean bDone;
    AutoLib.Sequence mSequence;             // the root of the sequence tree
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    VuforiaLib_RoverRuckus mVLib;           // Vuforia wrapper object used by Steps
    Point mBmSize;                          // size of the image Vuforia would return as a bitmap
    AutoLib.MotorGuideStep mGuideStep;      // member variable so that init() can set it and start() can use it
    double mTime;   // time of last loop() call -- used to compute frames per second


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
        final float Ki = 0.05f;        // ... integrator term
        final float Kd = 0.0f;         // ... derivative term
        final float KiCutoff = 10.0f;   // maximum angle error for which we update integrator
        SensorLib.PID pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        // turn in place to face target within some error bound
        AutoLib.ErrorGuideStep motorGuideStep1 = new AutoLib.ErrorGuideStep(this, pid, null, 0);
        motorGuideStep1.setMaxPower(0.25f);
        // make a step that analyzes a camera image from Vuforia and steers toward the biggest orange blob, presumably the block.
        // it uses a ErrorGuideStep to process the heading error it computes into motor steering commands,
        // and a DistanceAngleTestStep to determine when we're done (here, close enough in angle error).
        CountedDistanceAngleTestStep angleTermStep = new AngleTerminationStep(10.0f, 3);
        mGuideStep  = new GoToBlockGuideStep(this, mVLib, motorGuideStep1, angleTermStep);
        // make and add the Step that combines all of the above to turn toward the orange block
        mSequence.add(new AutoLib.GuidedTerminatedDriveStep(this, motorGuideStep1, angleTermStep, mMotors));

        // move toward the target under continuing image-based guidance, stopping when we're "close"
        AutoLib.ErrorGuideStep motorGuideStep2 = new AutoLib.ErrorGuideStep(this, pid, null, 0.25f);
        motorGuideStep2.setMaxPower(0.25f);
        // make a step that analyzes a camera image from Vuforia and steers toward the biggest orange blob, presumably the block.
        // it uses a ErrorGuideStep to process the heading error it computes into motor steering commands,
        // and a DistanceAngleTestStep to determine when we're done (here, close enough in distance).
        CountedDistanceAngleTestStep distanceTermStep = new DistanceTerminationStep(6.0f, 2);
        mGuideStep  = new GoToBlockGuideStep(this, mVLib, motorGuideStep2, distanceTermStep);
        // make and add the Step that combines all of the above to turn toward the orange block
        mSequence.add(new AutoLib.GuidedTerminatedDriveStep(this, motorGuideStep2, distanceTermStep, mMotors));

        // continue on unguided for 1 sec to push the block and then stop all motors
        // this should be replaced by a step that terminates on something like a touch sensor hit ... TBD
        mSequence.add(new AutoLib.MoveByTimeStep(mMotors, 0.25f, 1,true));
    }

    @Override public void start()
    {
        // start out not-done
        bDone = false;

        // start Vuforia scanning
        mVLib.start();

        // optionally turn on the flashlight
        // mVLib.lightOn(true);

        /** Activate Tensor Flow Object Detection. */
        TFObjectDetector tfod = mVLib.initTfod(this);
        if (tfod != null) {
            tfod.activate();
        }

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
    }

    @Override
    public void stop() {
        super.stop();
    }

}

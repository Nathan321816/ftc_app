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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
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

// this guide step  uses Vuforia TensorFlow to detect the yellow block and
// tell a gyro-based Turn Step how far to turn.
// this avoids the problem of the camera losing track of the target as we turn.
class BearingToBlockGuideStep extends AutoLib.Step {

    VuforiaLib_RoverRuckus mVLib;
    OpMode mOpMode;
    ArrayList<AutoLib.SetDirectionHeadingPower> mSteps;

    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";

    BearingToBlockGuideStep(OpMode opMode, VuforiaLib_RoverRuckus VLib, ArrayList<AutoLib.SetDirectionHeadingPower> steps) {
        mOpMode = opMode;
        mVLib = VLib;
        mSteps = steps;
    }

    public boolean loop() {
        super.loop();
        mOpMode.telemetry.addData("BearingToBlockGuideStep", "");

        Recognition goldMineral = null;
        TFObjectDetector tfod = mVLib.getTfod();
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                mOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() >= 1) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineral = recognition;
                            mOpMode.telemetry.addData("Gold Mineral Detected", recognition);
                        }
                    }
                }
            }
        }

        // if we have data ...
        if (goldMineral != null) {

            // get TF's estimate of angle error - positive (CCW) means we're pointing to the left of where we should be
            float angError = (float)(goldMineral.estimateAngleToObject(AngleUnit.DEGREES));
            mOpMode.telemetry.addData("TF angle err", angError);

            // tell the turn and drive steps the bearing to turn to so we face the block.
            // assuming gyro considers initial heading zero, we want to point at the block at heading -angError
            for (AutoLib.SetDirectionHeadingPower step : mSteps) {
                step.setHeading(-angError);
            }

            return true;    // done ... next step (presumably a Turn) can now run
        }

        return false;  // haven't gotten data yet ...
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
    double mTime;   // time of last loop() call -- used to compute frames per second
    BNO055IMUHeadingSensor mGyro;           // gyro to use for heading information


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

        // get hardware IMU and wrap gyro in HeadingSensor object usable below
        mGyro = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mGyro.init(3);  // 3: upright crosswise with REV face forward


        // best to do this now, which is called from opmode's init() function
        mVLib = new VuforiaLib_RoverRuckus();
        mVLib.init(this, null);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // turn in place to face target within some error bound
        // this step actually executes the turn
        SensorLib.PID pid1 = new SensorLib.PID(0.02f, 0.02f, 0, 10.0f);
        AutoLib.AzimuthTolerancedTurnStep turnStep = new AutoLib.AzimuthTolerancedTurnStep(this, 0, mGyro, pid1, mMotors, 0.3f, 5.0f, 3.0f);
        // this one drives along the same heading to the target
        AutoLib.AzimuthTimedDriveStep driveStep = new AutoLib.AzimuthTimedDriveStep(this, 0, mGyro, pid1, mMotors, 0.3f, 1.2f, true);

        // this step uses the Vuforia TensorFlow code to determine the angle to the target (yellow cube)
        // and tells the gyro-based turn step how far to turn.
        ArrayList<AutoLib.SetDirectionHeadingPower> steps = new ArrayList<AutoLib.SetDirectionHeadingPower> ();
        steps.add(turnStep);
        steps.add(driveStep);
        mSequence.add(new BearingToBlockGuideStep(this, mVLib, steps));

        // after the Bearing step completes, we can run the Turn step
        mSequence.add(turnStep);

        // next, move toward the target under gyro-control along same heading
        mSequence.add(driveStep);
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

        /** Deactivate Tensor Flow Object Detection. */
        TFObjectDetector tfod = mVLib.initTfod(this);
        if (tfod != null) {
            tfod.shutdown();
        }
    }

}


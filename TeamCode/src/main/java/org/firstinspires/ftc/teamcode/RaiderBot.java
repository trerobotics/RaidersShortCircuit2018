/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;


public class RaiderBot
{
    /* Public OpMode members. */
    public DcMotor FrDrive = null;
    public DcMotor FlDrive = null;
    public DcMotor BrDrive = null;
    public DcMotor BlDrive = null;

    public DcMotor leftConveyor = null;
    public DcMotor rightConveyor = null;

    public DcMotor relic = null;

    public Servo relicFlip = null;
    public Servo relicFinger = null;
    public Servo jewelArm=null;
    public Servo blockFlipLeft = null;
    public Servo blockFlipRight = null;

    private BNO055IMU imu = null;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    OpticalDistanceSensor ods = null;
    public ColorSensor jewelSensor = null;

    public final double FLIPPER_UP = 0;
    public final double FLIPPER_DOWN = 1;
    public final double RELIC_OPEN = 1;
    public final double RELIC_CLOSED = 0;
    public final double RELIC_UP = 0;
    public final double RELIC_DOWN = 1;
    public final double JEWEL_DOWN = 1;
    public final double JEWEL_UP = 0;

    private final float SLOW_SPEED_SCALE = .1f;
    private final float maxSpeed = .3f;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    Telemetry telemetry;

    /* Constructor */
    public RaiderBot(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FrDrive = hwMap.get(DcMotor.class, "FrDrive");
        FlDrive = hwMap.get(DcMotor.class, "FlDrive");
        BrDrive = hwMap.get(DcMotor.class, "BrDrive");
        BlDrive = hwMap.get(DcMotor.class, "BlDrive");
        leftConveyor = hwMap.get(DcMotor.class, "leftConveyor");
        rightConveyor = hwMap.get(DcMotor.class, "rightConveyor");
        relic = hwMap.get(DcMotor.class,  "RelicMotor");
        blockFlipLeft = hwMap.get(Servo.class, "blockFlipperLeft");
        jewelArm = hwMap.get(Servo.class, "jewelArm");
        jewelSensor = hwMap.get(ColorSensor.class, "jewelSensor");
        blockFlipRight = hwMap.get(Servo.class, "blockFlipperRight");
        relicFlip = hwMap.get(Servo.class, "relicFlipper");
        relicFinger = hwMap.get(Servo.class, "relicFinger");
        ods = hwMap.get(OpticalDistanceSensor.class, "ods");

        FrDrive.setDirection(DcMotor.Direction.REVERSE);
        FlDrive.setDirection(DcMotor.Direction.FORWARD);
        BrDrive.setDirection(DcMotor.Direction.REVERSE);
        BlDrive.setDirection(DcMotor.Direction.FORWARD);
        leftConveyor.setDirection(DcMotor.Direction.FORWARD);
        rightConveyor.setDirection(DcMotor.Direction.REVERSE);


        FrDrive.setPower(0);
        FlDrive.setPower(0);
        BrDrive.setPower(0);
        BlDrive.setPower(0);
        leftConveyor.setPower(0);
        rightConveyor.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        FrDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FlDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FlDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BlDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftConveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftConveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        blockFlipLeft.setDirection(Servo.Direction.FORWARD);
        blockFlipRight.setDirection(Servo.Direction.REVERSE);
    }


    public void drive(float xInput, float yInput, float zInput)
    {
        FrDrive.setPower(Range.clip((yInput - xInput - zInput), -maxSpeed, maxSpeed));
        FlDrive.setPower(Range.clip((yInput + xInput + zInput), -maxSpeed, maxSpeed));
        BrDrive.setPower(Range.clip((yInput + xInput - zInput), -maxSpeed, maxSpeed));
        BlDrive.setPower(Range.clip((yInput - xInput + zInput), -maxSpeed, maxSpeed));
    }

    public void rotate(double angle, float power)
    {

        if(angle < Math.round(getAngle()))
        {
            drive(0,0, power);
        } else if (angle > Math.round(getAngle()))
        {
            drive(0,0, -power);
        } else
        {
            drive(0,0,0);
        }

        telemetry.addData("Target Angle", angle);
        telemetry.addData("Current Angle", Math.round(getAngle()));
        telemetry.update();
    }

    //region methods for conveyor and flipper
    public void runConveyor(float speed)
    {
        leftConveyor.setPower(speed);
        rightConveyor.setPower(speed);
    }

    public void flipblock()
    {
        if(blockFlipLeft.getPosition() == FLIPPER_DOWN)
        {
            blockFlipLeft.setPosition(FLIPPER_UP);
            blockFlipRight.setPosition(FLIPPER_UP);
        } else
        {
            blockFlipLeft.setPosition(FLIPPER_DOWN);
            blockFlipRight.setPosition(FLIPPER_DOWN);
        }
    }
    //endregion


    public void toggleRelicFlip()
    {
        if(relicFlip.getPosition() == RELIC_DOWN)
        {
            relicFlip.setPosition(RELIC_UP);
        } else
        {
            relicFlip.setPosition(RELIC_DOWN);
        }
    }

    public void RelicUp()
    {
        relicFlip.setPosition(RELIC_UP);
    }

    public void RelicDown()
    {
        relicFlip.setPosition(RELIC_DOWN);
    }

    public void toggleRelicFinger()
    {
        if(relicFinger.getPosition() == RELIC_CLOSED)
        {
            relicFinger.setPosition(RELIC_OPEN);
        } else
        {
            relicFinger.setPosition(RELIC_CLOSED);
        }
    }

    public void moveRelic(float power)
    {
        relic.setPower(power);
    }


    //region Imu methods for reseting, getting, and checking angle

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    //endregion

    public boolean isClose(double thresh)
    {
        return ods.getLightDetected() > thresh;
    }

    public void debugOds()
    {
        telemetry.addData("ODS", ods.getLightDetected());
        telemetry.update();
    }

    public void runJewelBlue()
    {
        jewelArm.setPosition(JEWEL_DOWN);

        if(jewelSensor.blue() > jewelSensor.red())
        {
            rotate(10, .3f);
        } else
        {
            rotate(-10, .3f);
        }
    }

    public void runJewelRed()
    {
        jewelArm.setPosition(JEWEL_DOWN);

        if(jewelSensor.blue() > jewelSensor.red())
        {
            rotate(-10, .3f);
        } else
        {
            rotate(10, .3f);
        }
    }
}


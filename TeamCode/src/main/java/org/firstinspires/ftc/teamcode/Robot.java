package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;


public class Robot
{
    // Motor Objects
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    public DcMotor arm;

    // Servo objects
    public Servo leftFinger = null;
    public Servo rightFinger = null;

    //Imu objects
    IMU imu;


    // Cryptobox Constants
    int[] glyphPos;
    int currArmPos = 0;

    // Robot constants
    private final float P_COEF = .06f;
    private final float SLOW_SPEED_SCALE = .5f;

    private List<DcMotor> driveMotors = new ArrayList<>();
    private float maxSpeed = 1f;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private Telemetry telemetry;

    /* Constructor */
    public Robot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    //------------------------------------
    //Miscellaneous Functions
    //------------------------------------
    public void setMaxSpeed(float max){
        maxSpeed = max;
    }

    // Sets the mode of a list of DcMotors
    public void setMode(List<DcMotor> list, DcMotor.RunMode runmode) {
        for(int i = 0; i < list.size(); i++) {
            list.get(i).setMode(runmode);

        }
    }

    // Sets the mode of a single DcMotor
    public void setMode(DcMotor motor, DcMotor.RunMode runmode)
    {
        motor.setMode(runmode);
    }




    //------------------------------------
    // Robot Driving Functions
    //------------------------------------

    // makes the robot move. Takes in three axis for movement, a boolean for whether the robot should keep its
    // current heading, and a boolean for whether the robot should be slowed down
    public void drive(float xInput, float yInput, float zInput, boolean keepheading, boolean slowRobot)
    {
        float slowSpeedMultiplier;
        if(slowRobot) {
            slowSpeedMultiplier = SLOW_SPEED_SCALE;
        } else
        {
            slowSpeedMultiplier = 0;
        }

        double steer = imu.getError(0) * P_COEF;

        if(keepheading)
        {
            telemetry.addLine().addData("offset", steer);
            telemetry.update();
        } else
        {
            frontRight.setPower((Range.clip(yInput - xInput - zInput, -maxSpeed, maxSpeed)) * slowSpeedMultiplier);
            frontLeft.setPower((Range.clip(yInput + xInput + zInput, -maxSpeed, maxSpeed)) * slowSpeedMultiplier);
            backRight.setPower((Range.clip(yInput + xInput - zInput, -maxSpeed, maxSpeed)) * slowSpeedMultiplier);
            backLeft.setPower((Range.clip(yInput - xInput + zInput, -maxSpeed, maxSpeed)) * slowSpeedMultiplier);
        }
    }

    // makes the robot move. Takes in three axis for movement, and a boolean for whether the robot should keep its
    // current heading. WIthOUT SLOW SPEED. USED FOR STUFF LIKE AUTONOMOUS.
    public void drive(float xInput, float yInput, float zInput, boolean keepheading)
    {

        double steer = imu.getError(0) * P_COEF;

        if(keepheading)
        {
            telemetry.addLine().addData("offset", steer);
            telemetry.update();
        } else
        {
            frontRight.setPower(Range.clip(yInput - xInput - zInput, -maxSpeed, maxSpeed));
            frontLeft.setPower(Range.clip(yInput + xInput + zInput, -maxSpeed, maxSpeed));
            backRight.setPower(Range.clip(yInput + xInput - zInput, -maxSpeed, maxSpeed));
            backLeft.setPower(Range.clip(yInput - xInput + zInput, -maxSpeed, maxSpeed));
        }
    }

    //------------------------------------------------------
    // Arm and Gripper Functions
    //------------------------------------------------------

    // Arm position is set to whatever the current position is plus the new position.
    public void additiveArmMovement(int newTargetArmPos)
    {
        setMode(arm, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(arm, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(arm.getCurrentPosition() + newTargetArmPos);

        setMode(arm, DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(.3);
        while (arm.isBusy())
        {

        }
        arm.setPower(0);
    }

    // Step arm through four values corresponding to the height of the glyphs
    // TODO: Get this working. IDK what's wrong with it and its 2 in the morning
    public void stepArmMovement(boolean up, boolean down)
    {

        if(down && currArmPos < 0)
        {
            telemetry.addData("Arm","going down");
            Log.d("arm", "going down");
            absoluteArmMovement(glyphPos[currArmPos--]);
        } else if(up && currArmPos > 2)
        {
            telemetry.addData("Arm", "going up");
            Log.d("arm", "going down");
            absoluteArmMovement(currArmPos++);
        } else {
            telemetry.addLine().addData("Arm", "Arm is not moving");
        }

        telemetry.update();
    }

    // Arm moves to the parameter's value
    // TODO: figure out if this is the problem with stepArmMovement. I don't think it is but who knows at this point
    public void absoluteArmMovement(int newTargetArmPos)
    {
        setMode(arm, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition( newTargetArmPos);

        setMode(arm, DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(.3);
        while (arm.isBusy())
        {

        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Closes the servos for the gripper
    public void closeArm()
    {
        leftFinger.setPosition(1);
        rightFinger.setPosition(0);
    }

    // Opens the servos for the gripper
    public void openArm()
    {
        leftFinger.setPosition(.5);
        rightFinger.setPosition(.5);
    }

    //------------------------------------------------------
    // Cryptobox Alignment Functions
    //------------------------------------------------------

    // this function moves the robot in a certain direction until the cryptobox is found
    public void findCryptoBox(float xInput, float yInput, float zInput, CryptoboxDetector cryptoboxDetector, Telemetry telemetry)
    {
        boolean isCryptoboxDetected = cryptoboxDetector.isCryptoBoxDetected();
        drive(xInput, yInput, zInput, true);
        while (!isCryptoboxDetected)
        {
            telemetry.addData("Cryptobox", cryptoboxDetector.isCryptoBoxDetected());
            telemetry.update();
            isCryptoboxDetected = cryptoboxDetector.isCryptoBoxDetected();
        }

        drive(0,0,0, true);
    }

    // This function should line up the robot with the cryptobox
    // column: 0 is left, 1 is center, 2 is right
    public void alignWithCryptoBox(float xInput, float yInput, float zInput, int column, CryptoboxDetector cryptoboxDetector,
                                   Telemetry telemetry)
    {
        drive(xInput, yInput, zInput, true);

        switch (column) {
            case 0:
                while(cryptoboxDetector.getCryptoBoxLeftPosition() > 0 || cryptoboxDetector.getCryptoBoxLeftPosition() < 0)
                {
                    telemetry.addData("Lining up with left CryptoBox.\n %s left to align",
                            cryptoboxDetector.getCryptoBoxLeftPosition());
                    telemetry.update();
                }
                break;
            case 1:
                while(cryptoboxDetector.getCryptoBoxCenterPosition() > 0 || cryptoboxDetector.getCryptoBoxCenterPosition() < 0)
                {
                    telemetry.addData("Lining up with center CryptoBox.\n %s left to align",
                            cryptoboxDetector.getCryptoBoxCenterPosition());
                    telemetry.update();
                }
                break;
            case 2:
                while(cryptoboxDetector.getCryptoBoxRightPosition() > 0 || cryptoboxDetector.getCryptoBoxRightPosition() < 0)
                {
                    telemetry.addData("Lining up with right CryptoBox.\n %s left to align",
                            cryptoboxDetector.getCryptoBoxRightPosition());
                    telemetry.update();
                }
                break;
            default:
                telemetry.addData("Error", "could not align");
                telemetry.update();
                break;
        }
        drive(0,0,0, true);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        frontRight = hwMap.get(DcMotor.class, "front_right_motor");
        frontLeft = hwMap.get(DcMotor.class, "front_left_motor");
        backRight = hwMap.get(DcMotor.class, "back_right_motor");
        backLeft = hwMap.get(DcMotor.class, "back_left_motor");
        arm = hwMap.get(DcMotor.class, "arm");
        leftFinger = hwMap.get(Servo.class, "left_hand");
        rightFinger = hwMap.get(Servo.class, "right_hand");

        imu = new IMU();
        imu.InitIMU(ahwMap, "imu");


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotors.add(frontRight);
        driveMotors.add(frontLeft);
        driveMotors.add(backRight);
        driveMotors.add(backLeft);


        // Set all motors to zero power
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        arm.setPower(0);

        setMode(driveMotors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.getCurrentPosition();
        while(!imu.imu.isGyroCalibrated()){ }

        glyphPos = new int[]{75, 450, 775, 1100};
    }
 }


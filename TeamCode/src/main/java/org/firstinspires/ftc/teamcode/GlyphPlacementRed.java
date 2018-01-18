package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Auto: Red vision  ", group ="Brockbot")
public class GlyphPlacementRed extends LinearOpMode {


    private Robot robot = new Robot(telemetry);
    private CryptoboxDetector cryptoboxDetector = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
     private ClosableVuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        robot.init(hardwareMap);

        // Get and start the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        ClosableVuforiaLocalizer.Parameters parameters = new ClosableVuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Licence key for vuforia
        parameters.vuforiaLicenseKey = "ARkiT2//////AAAAGXLiKH9KtE9khLFDV47Uxk9/k2tW2LHnSHMrhIMYiYNHVv3fi+lJGozeGF6jAADNUoNwbQpYuCQcfZLk0vmFjwf+BVjButuNSmi8IhbiyQZXcwurS/9iujsvZjnITkiSIgtAKhGtra6JNiGxo0ywdgmSzK0Hn2i2OFeIm1jPBwNnfiC8eftRU9BwIZTM9ao6OFJTOfggL5zsLqO9VVJItZs/6PW7KHPsv7pHixKY2iaE2oKUVclP+aL0OJO7+kvmVdVoOJnnWVersBkJAjZMSz7TwCE598DBrB0Te2Pbn3oTMdg+lcpvlqfpdqasHLy9/Y7Nw2FRxgbH/npoYAqFnZxSBnLIQSad2KV1h3M4xcl+";

        // Set active camera to back
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = new ClosableVuforiaLocalizer(parameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            telemetry.update();
            // Load VuMarks. RelicRecoveryVuMark is an Enum that holds the VuMarks for
            // the left, center and right of the cryptobox
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuMark = RelicRecoveryVuMark.RIGHT;
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                // We found an instance of the VuMark. show which one it is for debug purposes.
                telemetry.addData("VuMark", "%s visible", vuMark);
                vuforia.close();

                cryptoboxDetector = new CryptoboxDetector();
                cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

                cryptoboxDetector.downScaleFactor = .5;
                cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.RED;
                cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.BALANCED;
                cryptoboxDetector.rotateMat = false;

                switch (vuMark) {
                    case LEFT:
                        cryptoboxDetector.enable();

                        // Close grabber and raise arm
                        robot.closeArm();
                        robot.additiveArmMovement(400);

                        //robot.findCryptoBox(-0.15f, 0f, 0f, cryptoboxDetector);
                        //robot.alignWithCryptoBox(-.1f, 0, 0, 0, cryptoboxDetector, telemetry);
                        // call functions to place glyph in column
                        break;
                    case CENTER:
                        cryptoboxDetector.enable();

                        // Close grabber and raise arm
                        robot.closeArm();
                        robot.additiveArmMovement(400);
                        robot.imu.getError(0);
                        //robot.findCryptoBox(-.15f, 0,0, cryptoboxDetector);
                        //robot.alignWithCryptoBox(-.1f, 0,0,1, cryptoboxDetector, telemetry);
                        cryptoboxDetector.disable();
                        break;
                    case RIGHT:
                        cryptoboxDetector.enable();

                        // Close grabber and raise arm
                        robot.closeArm();
                        robot.additiveArmMovement(400);

                        // Align with Cryptobox
                        //robot.findCryptoBox(0, .15f,0, cryptoboxDetector, telemetry);
                        //robot.alignWithCryptoBox(-.1f, 0,0,2, cryptoboxDetector, telemetry);
                        sleep(10000);
                        cryptoboxDetector.disable();
                        break;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
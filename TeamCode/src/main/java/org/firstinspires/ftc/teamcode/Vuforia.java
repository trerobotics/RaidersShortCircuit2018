package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.ClosableVuforiaLocalizer;

/**
 * Created by HS East Robotics on 4/12/2018.
 */

public class Vuforia {

    private String licenceKey = "ARkiT2//////AAAAGXLiKH9KtE9khLFDV47Uxk9/k2tW2LHnSHMrhIMYiYNHVv3fi+lJGozeGF6jAADNUoNwbQpYuCQcfZLk0vmFjwf+BVjButuNSmi8IhbiyQZXcwurS/9iujsvZjnITkiSIgtAKhGtra6JNiGxo0ywdgmSzK0Hn2i2OFeIm1jPBwNnfiC8eftRU9BwIZTM9ao6OFJTOfggL5zsLqO9VVJItZs/6PW7KHPsv7pHixKY2iaE2oKUVclP+aL0OJO7+kvmVdVoOJnnWVersBkJAjZMSz7TwCE598DBrB0Te2Pbn3oTMdg+lcpvlqfpdqasHLy9/Y7Nw2FRxgbH/npoYAqFnZxSBnLIQSad2KV1h3M4xcl+";

    private HardwareMap hwMap;

    private ClosableVuforiaLocalizer vuforiaLocalizer;

    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry)
    {
        hwMap = hardwareMap;
        this.telemetry = telemetry;

        int cameraMoniterViewId = hwMap.appContext.getResources().getIdentifier("cameraMoniterViewId", "id", hwMap.appContext.getPackageName());
        ClosableVuforiaLocalizer.Parameters parameters = new ClosableVuforiaLocalizer.Parameters(cameraMoniterViewId);

        parameters.vuforiaLicenseKey = licenceKey;

        parameters.cameraDirection = ClosableVuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = new ClosableVuforiaLocalizer(parameters);

        relicTrackables = this.vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
    }


    public RelicRecoveryVuMark getVuMark()
    {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    public void close()
    {
        vuforiaLocalizer.close();
        telemetry.addData("Vuforia", "Closed");
        telemetry.update();
    }

}

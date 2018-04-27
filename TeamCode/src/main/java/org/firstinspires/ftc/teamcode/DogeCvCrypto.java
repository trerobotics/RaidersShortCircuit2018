package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.ViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by HS East Robotics on 4/19/2018.
 */

public class DogeCvCrypto {

    private Context context;
    private ViewDisplay viewDisplay;
    private RaiderBot raiderBot;

    public CryptoboxDetector cryptoboxDetector;

    // This is dependant on the robot being init beforehand
    public DogeCvCrypto(Context context, ViewDisplay viewDisplay, RaiderBot robot)
    {
        this.context = context;
        this.viewDisplay = viewDisplay;
        this.raiderBot = robot;
    }

    public void initialize()
    {
        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(context, CameraViewDisplay.getInstance());

        cryptoboxDetector.downScaleFactor = .5;
        cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.BLUE;
        cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.BALANCED;
        cryptoboxDetector.rotateMat = false;
    }

    public void enable()
    {
        cryptoboxDetector.enable();
    }

    public void driveToCrypto(float speed, RelicRecoveryVuMark relicRecoveryVuMark)
    {
        raiderBot.drive(speed, 0,0);

        if(relicRecoveryVuMark == RelicRecoveryVuMark.LEFT)
        {
            if(cryptoboxDetector.getCryptoBoxLeftPosition() == cryptoboxDetector.getFrameSize().width / 2)
            {
                raiderBot.drive(0,0,0);
            } else if (cryptoboxDetector.getCryptoBoxLeftPosition() < cryptoboxDetector.getFrameSize().width / 2)
            {
                raiderBot.drive(-speed, 0,0);
            } else
            {
                // Continue
            }
        }
        else if (relicRecoveryVuMark == RelicRecoveryVuMark.CENTER)
        {
            if(cryptoboxDetector.getCryptoBoxCenterPosition() == cryptoboxDetector.getFrameSize().width / 2)
            {
                raiderBot.drive(0,0,0);
            }else if (cryptoboxDetector.getCryptoBoxLeftPosition() < cryptoboxDetector.getFrameSize().width / 2)
            {
                raiderBot.drive(-speed, 0,0);
            } else
            {
                // Continue
            }
        }
        else if(relicRecoveryVuMark == RelicRecoveryVuMark.RIGHT)
        {
            if (cryptoboxDetector.getCryptoBoxRightPosition() == cryptoboxDetector.getFrameSize().width / 2)
            {
                raiderBot.drive(0,0,0);
            }else if (cryptoboxDetector.getCryptoBoxLeftPosition() < cryptoboxDetector.getFrameSize().width / 2)
            {
                raiderBot.drive(-speed, 0,0);
            } else
            {
                // Continue
            }
        }
        else
        {
            raiderBot.drive(0,0,0);
        }
    }
}

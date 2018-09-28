package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.concurrent.BlockingQueue;

/**
 * Created by 19kmunz on 9/27/18.
 */

public class VuforiaOp extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "Ac6JKiL/////AAABmRYN0606UUpPn+wMuRQTR5FGl8XU1SlRVjmJoYWhFfFJmgUAJUgquvOr44OcxMriCuQzPI6RHjfT8IAnF6eBkr8pK1cdft9+x2pzqPnCiDuY88MdJNy8XdgfgK4u/Hf3quxhZPp7E3wYJzEsmuDuM0hUD0bvieLvOwHhR4lNq1yXT/R60mTYRDPWCBrXUWKLdzaRFaWMNZw15Rt89IIWo2moLWM7HEsI3nRDw1ZWZnY4it0+r0treNRAplIBYfUSuvpjGA7gBWidsov5VN5Y+L7LZElojiy2JyN2HWbCkhanHCUZjdaL02dp2iaCFImeE3uGetbZF4qmyr/d5xvowS6ylquPM147s9HP+t3xPrsE";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        //VuforiaRoverRuckus roverRuckus = new VuforiaRoverRuckus();
        //VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17")

    }
}

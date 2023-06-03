package jp.jaxa.iss.kibo.rpc.planner;

import android.util.Log;

import gov.nasa.arc.astrobee.PendingResult;
import gov.nasa.arc.astrobee.internal.BaseRobot;
import gov.nasa.arc.astrobee.internal.BaseRobotImpl;
import gov.nasa.arc.astrobee.internal.CommandBuilder;
import gov.nasa.arc.astrobee.internal.Publishable;
import gov.nasa.arc.astrobee.types.FlightMode;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.objdetect.QRCodeDetector;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1(){
        int loop_counter = 0;
        int target_id;

        // the mission starts
        api.startMission();

        /*
        * Move Point7
        * */
        MoveToWaypoint(waypoints_config.wp1);
        MoveToWaypoint(waypoints_config.point7);

        // Savefig
        Mat image = new Mat();
        image = api.getMatNavCam();
        api.saveMatImage(image,"point7_nav.png");

        // turn on the front flash light
        api.flashlightControlFront(0.05f);

        // turn off the front flash light
        api.flashlightControlFront(0.00f);

        // get QR code content
        String mQrContent = read_QRcode(image);


        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();

        /* ********************************************************** */
        /* write your own code to move Astrobee to the goal positiion */
        /* ********************************************************** */

        // send mission completion
        api.reportMissionCompletion(mQrContent);
    }

    private void MoveToWaypoint(Waypoint name){

        final int LOOP_MAX = 10;

        int count = 0;
        while(count < LOOP_MAX){
            final Point point = new Point(
                    (float)(name.posX + name.avoidX*count),
                    (float)(name.posY + name.avoidY*count),
                    (float)(name.posZ + name.avoidZ*count)  );
            final Quaternion quaternion = new Quaternion(
                    (float)name.quaX,
                    (float)name.quaY,
                    (float)name.quaZ,
                    (float)name.quaW    );

            Result result = api.moveTo(point, quaternion, true);
            ++count;

            if(result.hasSucceeded()){
                break;
            }
            Log.w(TAG, "move Failure, retry");
        }
    }

    // You can add your method
    private String read_QRcode(Mat image){
        String QRcode_content = "";
        try{
            api.saveMatImage(image,"QR.png");
//        Mat mini_image = new Mat(image, new Rect(320, 240, 640, 480));
            Mat mini_image = new Mat(image, new Rect(480, 360, 320, 240));
            api.saveMatImage(mini_image,"QR_mini.png");

            MatOfPoint2f points = new MatOfPoint2f();
            Mat straight_qrcode = new Mat();
            QRCodeDetector qrc_detector = new QRCodeDetector();
            Boolean detect_success = qrc_detector.detect(mini_image, points);
            Log.i(TAG,"detect_success is " + detect_success.toString());

            QRcode_content = qrc_detector.detectAndDecode(mini_image, points, straight_qrcode);
            if(QRcode_content != null){
                Mat straight_qrcode_gray = new Mat();
                straight_qrcode.convertTo(straight_qrcode_gray, CvType.CV_8UC1);
                api.saveMatImage(straight_qrcode_gray,"QR_binary.png");
            }
            Log.i(TAG,"QRCode_content is " + QRcode_content);

        } catch(Exception e){
            ;
        }

        /**
         * QRCode_CONTENT to REPORT_MESSEGE
         */
        switch(QRcode_content){
            case "JEM":
                QRcode_content = "STAY_AT_JEM";
                break;
            case "COLUMBUS":
                QRcode_content = "GO_TO_COLUMBUS";
                break;
            case "RACK1":
                QRcode_content = "CHECK_RACK_1";
                break;
            case "ASTROBEE":
                QRcode_content = "I_AM_HERE";
                break;
            case "INTBALL":
                QRcode_content = "LOOKING_FORWARD_TO_SEE_YOU";
                break;
            case "BLANK":
                QRcode_content = "NO_PROBLEM";
                break;
            default:
                QRcode_content = "";
                break;
        }

        return QRcode_content;

    }
}

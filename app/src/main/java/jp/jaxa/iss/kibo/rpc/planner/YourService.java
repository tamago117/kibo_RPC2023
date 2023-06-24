package jp.jaxa.iss.kibo.rpc.planner;

import android.util.Log;

<<<<<<< Updated upstream
import gov.nasa.arc.astrobee.PendingResult;
import gov.nasa.arc.astrobee.internal.BaseRobot;
import gov.nasa.arc.astrobee.internal.BaseRobotImpl;
import gov.nasa.arc.astrobee.internal.CommandBuilder;
import gov.nasa.arc.astrobee.internal.Publishable;
import gov.nasa.arc.astrobee.types.FlightMode;
=======
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.List;
>>>>>>> Stashed changes
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

<<<<<<< Updated upstream
=======
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.CvType;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import org.opencv.objdetect.QRCodeDetector;

import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
>>>>>>> Stashed changes
import java.util.List;
<<<<<<< Updated upstream

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.objdetect.QRCodeDetector;

=======
>>>>>>> Stashed changes
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes

    @Override
    protected void runPlan1(){
        int loop_counter = 0;
        int target_id;

        // the mission starts
        api.startMission();
<<<<<<< Updated upstream

        /*
        * Move Point7
        * */
        MoveToWaypoint(waypoints_config.wp1);
        MoveToWaypoint(waypoints_config.point7);

<<<<<<< Updated upstream
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
=======
        ///////////////ここでQRを読み込む///////////////////
        Mat image = new Mat();
        image = api.getMatNavCam();
        api.saveMatImage(image,"wp2.png");
        String report = read_QRcode(image);
        ////////////////////////////////////////////////////


=======
        Log.i(TAG, "start!!!!!!!!!!!!!!!!");
        MoveToWaypoint(waypoints_config.wp1); // initial point
        Global.Nowplace = 7;
//        MoveToWaypoint(waypoints_config.wp2); // QR point
//        Global.Nowplace = 8;
//        ///////////////ここでQRを読み込む///////////////////
//        Mat image = new Mat();
//        api.flashlightControlFront(0.0f);
//        image = api.getMatNavCam();
//        api.saveMatImage(image,"wp2.png");
//        Global.report = read_QRcode(image);
        ////////////////////////////////////////////////////

>>>>>>> Stashed changes
        //////////////ここから探索//////////////////////////
        //Long ActiveTime = Time.get(0); //現在のフェーズの残り時間(ミリ秒)
        //Long MissionTime = Time.get(1); //ミッション残り時間(ミリ秒)
        //List<Long> Time = api.getTimeRemaining();

        while (true){
            Log.i(TAG,"runPlan1内での現在位置"+Global.Nowplace);
            GoTarget(api.getActiveTargets());
        }
<<<<<<< Updated upstream
        Log.i(TAG,"go to goal");
        MoveToWaypoint(waypoints_config.goal_point);

>>>>>>> Stashed changes
        api.notifyGoingToGoal();

        /* ********************************************************** */
        /* write your own code to move Astrobee to the goal positiion */
        /* ********************************************************** */
=======
        //Log.i(TAG,"go to goal");
        //MoveToWaypoint(waypoints_config.goal_point);

        //api.notifyGoingToGoal();
        //api.reportMissionCompletion(Global.report);
>>>>>>> Stashed changes

        // send mission completion
        api.reportMissionCompletion(mQrContent);
    }

<<<<<<< Updated upstream
=======
    @Override
    protected void runPlan2(){
        // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){
        final Point point = new Point((float)pos_x, (float)pos_y, (float)pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        api.moveTo(point, quaternion, true);
    }

    private void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                                       double qua_x, double qua_y, double qua_z,
                                       double qua_w) {
        final Point point = new Point((float)pos_x, (float)pos_y, (float)pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);
        Result result = api.relativeMoveTo(point, quaternion, true);

        // 移動コマンドが正常動作しているかをStatus列挙型で出力 詳しくは下記URL
        // https://github.com/nasa/astrobee_android/blob/a8560ab0270ac281d8eadeb48645f4224582985e/astrobee_api/api/src/main/java/gov/nasa/arc/astrobee/Result.java
        if(result.hasSucceeded()){
            String str = result.getStatus().toString();
            Log.i(TAG, "[relativeMoveToWrapper]:"+str);
        }else{
            Log.w(TAG, " api.relativeMoveTo Error : result.hasSucceeded()=false");
        }
    }


>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
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
=======
    // Kinematics Github
    // https://github.com/nasa/astrobee_android/blob/a8560ab0270ac281d8eadeb48645f4224582985e/astrobee_api/api/src/main/java/gov/nasa/arc/astrobee/Kinematics.java
    private void LoggingKinematics(){
        //Kinematics no Log
        Kinematics kinematics = api.getRobotKinematics();
        Log.i(TAG, "[LoggingKinematics]: 状態" + kinematics.getConfidence().toString());
        Log.i(TAG, "[LoggingKinematics]: 絶対座標" + kinematics.getPosition().toString());
        Log.i(TAG, "[LoggingKinematics]: 方向座標" + kinematics.getOrientation().toString());
        Log.i(TAG, "[LoggingKinematics]: 線速度" + kinematics.getLinearVelocity().toString());      // 線速度
        Log.i(TAG, "[LoggingKinematics]: 角速度" + kinematics.getAngularVelocity().toString());     // 角速度
        Log.i(TAG, "[LoggingKinematics]: 加速度" + kinematics.getLinearAcceleration().toString());  // 加速度
    }

    private  void GoTarget(List<Integer> ActiveTargets){
        int index = ActiveTargets.size();
        int i = 0;
        double [] distance = new double[2];
        int [] point = new int[2];

        Log.i(TAG,"アクティブターゲット"+ActiveTargets.toString());
        //最短距離となるように目標ターゲットの順番を変更
        if (index == 2) {
            distance[0] = minimum_distance(Global.Nowplace,ActiveTargets.get(0)-1);
            distance[1] = minimum_distance(Global.Nowplace,ActiveTargets.get(1)-1);
            point[0] = TargetPoint(ActiveTargets.get(0));
            point[1] = TargetPoint(ActiveTargets.get(1));

            //残り時間が近くなったら点数の高いところから移動する．
            if(api.getTimeRemaining().get(1)<Global.RemainingTime + 0.5*60*1000 && point[0] < point[1]){
                //順番を交換
                int temp = ActiveTargets.get(0);
                ActiveTargets.set(0,ActiveTargets.get(1));
                ActiveTargets.set(1,temp);
                Log.i(TAG,"アクティブターゲットを交換"+ActiveTargets.toString());
            }
            //距離の近いところから行くために配列の順番を変更
            else if(distance[0] > distance[1]){
                //順番を交換
                int temp = ActiveTargets.get(0);
                ActiveTargets.set(0,ActiveTargets.get(1));
                ActiveTargets.set(1,temp);
                Log.i(TAG,"アクティブターゲットを交換"+ActiveTargets.toString());
            }
        }
        //

        while(i < index){
            Log.i(TAG, "Let's go Target" + ActiveTargets.get(i).toString());
            Log.i(TAG,"Gotarget内での現在位置"+Global.Nowplace);
            List<Integer>route = dijkstra(Global.Nowplace,ActiveTargets.get(i)-1); //-1はゼロオリジンへの修正
            Log.i(TAG,"Route"+route.toString());

            if(ActiveTargets.get(i)!=3) {
                Complete_confirme(false);
            }

            for(int n = 1; n<route.size();n++){ //n = 0はスタート地点なのでスキップ
                //Log.i(TAG, "Let's go to node " +route.get(n).toString());
                //ここにフェーズタイムの監視機能を入れる．if文とbreak
                Waypoint2Number(route.get(n));
                if(Global.Nowplace==8 && Global.report=="MISS"){
                    try {
                        Thread.sleep(7000);
                    } catch (InterruptedException e) {
                    }
                    Mat image = new Mat();
                    api.flashlightControlFront(0.0f);
                    image = api.getMatNavCam();
                    api.saveMatImage(image,"wp2.png");
                    Global.report = read_QRcode(image);
                }
            }
            api.laserControl(true);
            api.takeTargetSnapshot(ActiveTargets.get(i));
            if(ActiveTargets.get(i)==3) {
                Complete_confirme(true);
            }
            ++i;
        }
    }

    public static List<Integer> dijkstra(int start, int end) {
        double [][] A = adjacency_matrix.graph;
        int n = A.length; // 頂点数
        double[] distances = new double[n]; // 始点から各頂点までの最短距離
        boolean[] visited = new boolean[n]; // 頂点の訪問状態
        int [] prev = new int[n]; //直前の頂点
        List<Integer> path = new ArrayList<>(); //パスの保存

        // distances配列を初期化し、始点以外の頂点を無限大に設定
        Arrays.fill(distances, INF);
        distances[start] = 0.0;

        for (int i = 0; i < n; i++) {
            // 未訪問の頂点のうち、距離が最小の頂点を見つける
            double minDist = INF;
            int minIndex = -1;

            for (int j = 0; j < n; j++) {
                if (!visited[j] && distances[j] < minDist) {
                    minDist = distances[j];
                    minIndex = j;
                }
            }

            // 見つからなかった場合、終了
            if (minIndex == -1) {
                break;
            }

            // 見つかった頂点を訪問済みとする
            visited[minIndex] = true;

            // 隣接する頂点の距離を更新する
            for (int j = 0; j < n; j++) {
                if (!visited[j] && A[minIndex][j] != INF) {
                    double distance = distances[minIndex] + A[minIndex][j];
                    if (distance < distances[j]) {
                        distances[j] = distance;
                        prev[j] = minIndex;
                    }
                }
            }
        }
        if (distances[end] == INF) {
            return path; // 到達不可能な場合、空のリストを返す
        }
        // 終点から始点までの最短経路を復元
        int current = end;
        path.add(current);
        while (current != start) {
            current = prev[current];
            path.add(0,current);
        }
        path.add(0,start);
        return path;
    }
    // ゼロオリジンで考えたときのウェイポイントの番号
    private void Waypoint2Number(int n){
        Global.Nowplace = n; //現在位置の変更
        Log.i(TAG,"Now_place is "+ Global.Nowplace);
        switch (n){
            case 0:
                MoveToWaypoint(waypoints_config.point1);
                break;
            case 1:
                MoveToWaypoint(waypoints_config.point2);
                break;
            case 2:
                MoveToWaypoint(waypoints_config.point3);
                break;
            case 3:
                MoveToWaypoint(waypoints_config.point4);
                break;
            case 4:
                MoveToWaypoint(waypoints_config.point5);
                break;
            case 5:
                MoveToWaypoint(waypoints_config.point6);
                break;
            case 6:
                MoveToWaypoint(waypoints_config.goal_point);
                break;
            case 7:
                MoveToWaypoint(waypoints_config.wp1);
                break;
            case 8:
                MoveToWaypoint(waypoints_config.wp2);
                break;
            case 9:
                MoveToWaypoint(waypoints_config.wp3);
>>>>>>> Stashed changes
                break;
        }

        return QRcode_content;

    }


    /**
     * FUNCTIONs ABOUT QRCODE
     */

    /**
     * FUNCTIONs ABOUT QRCODE
     */
    private String read_QRcode(Mat image){
        String QRcode_content = "";
        /*
            NavCamのカメラ行列と歪み係数の取得
         */
        double[][] NavCamIntrinsics = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3,3,CvType.CV_32FC1);
        cameraMatrix.put(0,0,NavCamIntrinsics[0]);
        Mat distortionCoefficients = new Mat();
        distortionCoefficients.put(0,0,NavCamIntrinsics[1]);

        try{
            Log.i(TAG, "OPENCV_VERSION is " + Core.VERSION );
            /*
                Rect(int x, int y, int width, int height )
             */
            api.saveMatImage(image,"QR.png");
<<<<<<< Updated upstream
            Mat mini_image = new Mat(image, new Rect(700, 360, 240, 240)); // ここの値は切り取る領域
            api.saveMatImage(mini_image,"QR_mini.png");
=======
>>>>>>> Stashed changes

            Mat image_undistorted = new Mat();
            Imgproc.undistort(image,image_undistorted,cameraMatrix,distortionCoefficients);
            api.saveMatImage(image_undistorted,"QR_undistorted.png");

            Mat mini_image_undistorted = new Mat(image_undistorted, new Rect(320, 320, 320, 320)); // ここの値は切り取る領域
//            Mat mini_image_undistorted = new Mat(image_undistorted, new Rect(520, 360, 360, 360)); // 旧Waypoint2
            api.saveMatImage(mini_image_undistorted,"QR_mini_undistorted.png");

            // Colabで検証したものの再構築
            Mat im = new Mat();
            mini_image_undistorted.copyTo(im);
            Imgproc.Canny(im, im, 30, 70);
            api.saveMatImage(im,"mini_canny.png");
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(im, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(im, contours, -1, new Scalar(255,255,255), 5);
            Core.bitwise_not(im,im);
            api.saveMatImage(im,"mini_canny_draw_bitwise.png");

            Imgproc.findContours(im, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<MatOfPoint> contour_list = new ArrayList<MatOfPoint>();
            for (MatOfPoint contour : contours) {
                // 輪郭を近似する
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                double epsilon = 0.05 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                // 近似された輪郭の頂点数を取得する
                int vertices = approxCurve.toArray().length;

                // 四角形の場合は頂点数が4となる
                if (vertices == 4) {
                    double area = Imgproc.contourArea(approxCurve);
                    Log.i(TAG, String.valueOf(area));
                    Log.i(TAG, Arrays.toString(approxCurve.toArray()));
                    if(area > 4500 && area < 6500){
                            contour_list.add(0, new MatOfPoint(approxCurve.toArray()));
                            Log.i(TAG, Arrays.toString(approxCurve.toArray()));
                    }
                }
            }

            // sort approx
            org.opencv.core.Point[] tempPoints = new org.opencv.core.Point[8];
            tempPoints[0] = contour_list.get(0).toArray()[0];
            tempPoints[1]  = contour_list.get(0).toArray()[1];
            tempPoints[2]  = contour_list.get(0).toArray()[2];
            tempPoints[3]  = contour_list.get(0).toArray()[3];
            tempPoints[4] = contour_list.get(0).toArray()[0];
            tempPoints[5]  = contour_list.get(0).toArray()[1];
            tempPoints[6]  = contour_list.get(0).toArray()[2];
            tempPoints[7]  = contour_list.get(0).toArray()[3];
            double[] vector = new double[4];
            double max_vector = 0;
            int max_vector_index = -1;
            // 右下を見つける
            for(int i=0;i<4;i++){
                vector[i] = tempPoints[i].x * tempPoints[i].x * tempPoints[i].y * tempPoints[i].y;
                if(max_vector < vector[i]){
                    max_vector = vector[i];
                    max_vector_index = i;
                }
            }
            // 右下、右上、左上、左下
            org.opencv.core.Point[] srcPoints = new org.opencv.core.Point[4];
            srcPoints[0] = tempPoints[max_vector_index];
            srcPoints[1] = tempPoints[max_vector_index+1];
            srcPoints[2] = tempPoints[max_vector_index+2];
            srcPoints[3] = tempPoints[max_vector_index+3];
            MatOfPoint2f src = new MatOfPoint2f(srcPoints);

//            MatOfPoint2f src = new MatOfPoint2f(contour_list.get(0).toArray());
            Log.i(TAG, Arrays.toString(contour_list.get(0).toArray()));
            org.opencv.core.Point[] dstPoints = new org.opencv.core.Point[4];
            dstPoints[0] = new org.opencv.core.Point(825, 474);
            dstPoints[1] = new org.opencv.core.Point(825, 0);
            dstPoints[2] = new org.opencv.core.Point(0, 0);
            dstPoints[3] = new org.opencv.core.Point(0, 474);
            MatOfPoint2f dst = new MatOfPoint2f(dstPoints);
            Mat perspectiveMatrix = Imgproc.getPerspectiveTransform(src, dst);
            Mat mini_undistorted_warp = new Mat();
            Imgproc.warpPerspective(mini_image_undistorted, mini_undistorted_warp, perspectiveMatrix, new Size(825, 474));
            api.saveMatImage(mini_undistorted_warp,"mini_warp.png");

            QRCodeDetector qrc_detector = new QRCodeDetector();
            Mat mini_binary = new Mat();
            QRcode_content = qrc_detector.detectAndDecode(mini_undistorted_warp);
            Log.i(TAG, "QRCode_content is{{{" + QRcode_content + "}}}");
            QRcode_content = QRMessegeConverter(QRcode_content);
            if(QRcode_content != "MISS"){
                Log.i(TAG, "QRCode_content is" + QRcode_content + "");
                return QRcode_content;
            }
            for(int n=45; n<75; n++){
                Log.i(TAG, "threshhold param =" + String.valueOf(n));
                Imgproc.threshold(mini_undistorted_warp, mini_binary, n*2, 255,  Imgproc.THRESH_BINARY);
                QRcode_content = qrc_detector.detectAndDecode(mini_binary);
                Log.i(TAG, "QRCode_content is{{{" + QRcode_content + "}}}");
                api.saveMatImage(mini_binary,"mini_warp_binary"+String.valueOf(n)+".png");
                QRcode_content = QRMessegeConverter(QRcode_content);
                if(QRcode_content != "MISS"){
                    Log.i(TAG, "QRCode_content is" + QRcode_content + "");
                    return QRcode_content;
                }
            }
            Log.i(TAG, "QRCode_content is{{{" + QRcode_content + "}}}");

        } catch(Exception e){
            Log.i(TAG, "Error Occation");
            ;
        }

        return QRcode_content;
    }
    private String QRMessegeConverter(String string) {
        /**
         * QRCode_CONTENT to REPORT_MESSEGE
         */
        switch (string) {
            case "JEM":
                string = "STAY_AT_JEM";
                break;
            case "COLUMBUS":
                string = "GO_TO_COLUMBUS";
                break;
            case "RACK1":
                string = "CHECK_RACK_1";
                break;
            case "ASTROBEE":
                string = "I_AM_HERE";
                break;
            case "INTBALL":
                string = "LOOKING_FORWARD_TO_SEE_YOU";
                break;
            case "BLANK":
                string = "NO_PROBLEM";
                break;
            default:
                string = "MISS";
                break;
        }
        return string;
    }
    private double minimum_distance(int start,int end){
        List<Integer> route = dijkstra(start,end);
        double distance = 0;
        for (int n = 0; n < route.size() - 1; n++) {
            distance = adjacency_matrix.graph[route.get(n)][route.get(n + 1)];
        }
        return distance;

    }

    private void Complete_confirme(boolean terminate) {
        //Long ActiveTime = Time.get(0); //現在のフェーズの残り時間(ミリ秒)
        //Long MissionTime = Time.get(1); //ミッション残り時間(ミリ秒)

        if(!terminate){
            if (api.getTimeRemaining().get(1) < Global.RemainingTime) {
                Log.i(TAG, "go to goal");
                List<Integer> route = dijkstra(Global.Nowplace, 6);
                Log.i(TAG, "Route" + route.toString());
                for (int n = 1; n < route.size(); n++) { //n = 0はスタート地点なのでスキップ
                    //Log.i(TAG, "Let's go to node " +route.get(n).toString());
                    Waypoint2Number(route.get(n));
                }
                api.notifyGoingToGoal();
                api.reportMissionCompletion(Global.report);
            }
        }else{
            if(api.getTimeRemaining().get(1)<Global.RemainingTime){
                api.notifyGoingToGoal();
                api.reportMissionCompletion(Global.report);
            }
        }
    }
    private int TargetPoint(int Target){
        switch (Target){
            case 1:
                return 30;
            case 2:
                return 20;
            case 3:
                return 40;
            case 4:
                return 20;
            case 5:
                return 30;
            case 6:
                return 30;
        }
        return 0;
    }
}


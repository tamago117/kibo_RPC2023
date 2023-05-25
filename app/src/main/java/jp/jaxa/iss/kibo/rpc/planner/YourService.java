package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;


import java.util.List;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

import org.opencv.core.MatOfDouble;

import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();
    static String report = "STAY_AT_JEM"; //　ここにQRに対応するメッセージを書き込む

    @Override
    protected void runPlan1(){

        /*//////////////////////////////
        //      mission start         //
        /*//////////////////////////////
        api.startMission();

        //move 1 to 6
        //Long ActiveTime = Time.get(0); //現在のフェーズの残り時間(ミリ秒)
        //Long MissionTime = Time.get(1); //ミッション残り時間(ミリ秒)
        //List<Long> Time = api.getTimeRemaining();

        List<Integer> ActiveTargets = api.getActiveTargets(); //ターゲットの取得

        while (api.getTimeRemaining().get(1) <(5-4.0)*60*1000){
            GoTarget(api.getActiveTargets());
        }
        MoveToWaypoint(waypoints_config.goal_point);
        api.notifyGoingToGoal();
        api.reportMissionCompletion(report);

        /*
        //時間調整
        while (Time.get(1) > (5-3.5)*60*1000){
            Time = api.getTimeRemaining();
            Log.i(TAG, "残り時間" + Time.get(1).toString());

        }
        */

    }

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

    private void Print_AR(List<Mat> corners, Mat markerIds) {
        for (int n = 0; n < 4; n++) {
            Log.i(TAG, "markerIds:" + Arrays.toString(markerIds.get(n,0)));
            Log.i(TAG, "左上:" + Arrays.toString(corners.get(n).get(0, 0)));
            Log.i(TAG, "右上:" + Arrays.toString(corners.get(n).get(0, 1)));
            Log.i(TAG, "右下:" + Arrays.toString(corners.get(n).get(0, 2)));
            Log.i(TAG, "左下:" + Arrays.toString(corners.get(n).get(0, 3)));
        }
    }


    //右下のマーカを見つける
    private int findBottomRight(List<Mat> corners){
        Log.i(TAG,"start findBottomRight");
        // out = 関数のreturn
        int out = 0;
        int temp = 0;

        //corners.get(n).get(0, 0) -> n番目のマーカの右下のxy座標を取得
        for(int n=0; n<4; n++){
            Log.i(TAG,"Loop" + n );
            // 三平方の定理で一番数字が大きいものは遠いことを用いる
            // a^2 + b^2 = c^2
            double[] ab = corners.get(n).get(0,2);
            int c = (int)ab[0] * (int)ab[0] + (int)ab[1] * (int)ab[1];
            if(temp < c ){
                temp = c;
                out = n;
                Log.i(TAG,"change");
            }
        }
        // 右下（一番遠い）のは配列の何番目かをreturn
        Log.i(TAG,"finish findBottomRight");
        return out;
    }

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
        //現状は番号の若い順に行く仕様になっている
        // ----- 最短距離で移動出来るようにActiveTargetの内容を入れ替える必要ものをここ or 別関数に入れる ------------

        while(i < index){
            Log.i(TAG, "Let's go " + ActiveTargets.get(i).toString());
            switch (ActiveTargets.get(i)){
                case 1:
                    MoveToWaypoint(waypoints_config.point1);
                    break;
                case 2:
                    MoveToWaypoint(waypoints_config.point2);
                    break;
                case 3:
                    MoveToWaypoint(waypoints_config.point3);
                    break;
                case 4:
                    MoveToWaypoint(waypoints_config.point4);
                    break;
                case 5:
                    MoveToWaypoint(waypoints_config.point5);
                    break;
                case 6:
                    MoveToWaypoint(waypoints_config.point6);
                    break;
            }
            api.laserControl(true);
            api.takeTargetSnapshot(ActiveTargets.get(i));
            ++i;
        }
    }
}

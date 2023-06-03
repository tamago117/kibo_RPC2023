package jp.jaxa.iss.kibo.rpc.planner;

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
    private static final double INF = Double.POSITIVE_INFINITY; // 非接続を示すための無限大の値
    private final String TAG = this.getClass().getSimpleName();
    static String report = "STAY_AT_JEM"; //　ここにQRに対応するメッセージを書き込む
    static int Now_place; //現在位置

    @Override
    protected void runPlan1(){

        /*//////////////////////////////
        //      mission start         //
        /*//////////////////////////////
        api.startMission();
        Log.i(TAG, "start!!!!!!!!!!!!!!!!");
        MoveToWaypoint(waypoints_config.wp1); // initial point
        Now_place = 9;

        //Long ActiveTime = Time.get(0); //現在のフェーズの残り時間(ミリ秒)
        //Long MissionTime = Time.get(1); //ミッション残り時間(ミリ秒)
        //List<Long> Time = api.getTimeRemaining();

        while (api.getTimeRemaining().get(1) >(5-4.0)*60*1000){
            GoTarget(api.getActiveTargets(),Now_place);
        }
        Log.i(TAG,"go to goal");
        MoveToWaypoint(waypoints_config.goal_point);
        api.notifyGoingToGoal();
        api.reportMissionCompletion(report);


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

    private  void GoTarget(List<Integer> ActiveTargets, int Now_place){
        int index = ActiveTargets.size();
        int i = 0;
        //現状は番号の若い順に行く仕様になっている
        // ----- 最短距離で移動出来るようにActiveTargetの内容を入れ替える必要ものをここ or 別関数に入れる ------------

        while(i < index){
            Log.i(TAG, "Let's go " + ActiveTargets.get(i).toString());
            List<Integer> route = getShortestPath(Now_place,ActiveTargets.get(i));
            for(int n = 1; n<route.size();n++){
                Log.i(TAG, "Let's go to node " +route.get(n).toString());
                Waypoint2Number(route.get(n));
            }
            api.laserControl(true);
            api.takeTargetSnapshot(ActiveTargets.get(i));
            ++i;
        }
    }

    public static double[] dijkstra(double[][] A, int start) {
        int n = A.length; // 頂点数
        double[] distances = new double[n]; // 始点から各頂点までの最短距離
        boolean[] visited = new boolean[n]; // 頂点の訪問状態

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
                    }
                }
            }
        }

        return distances;
    }

    public static List<Integer> getShortestPath(int start, int end) {
        double[][] A = adjacency_matrix.graph;
        double[] distances = dijkstra(A, start);
        List<Integer> path = new ArrayList<>();

        if (distances[end] == INF) {
            return path; // 到達不可能な場合、空のリストを返す
        }

        // 終点から始点までの最短経路を復元
        int current = end;
        path.add(current);
        while (current != start) {
            for (int prev = 0; prev < A.length; prev++) {
                if (A[prev][current] != INF && distances[current] == distances[prev] + A[prev][current]) {
                    current = prev;
                    path.add(0, current);
                    break;
                }
            }
        }

        return path;
    }

    private String[] StringArray2DoubleArray(double[] array){
        String[] stringArray = new String[array.length];
        for (int i = 0; i < array.length; i++) {
            stringArray[i] = Double.toString(array[i]);
        }
        return stringArray;
    }

    private void Waypoint2Number(int n){
        Now_place = n; //現在位置の変更
        switch (n){
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
            case 7:
                MoveToWaypoint(waypoints_config.point7);
                break;
            case 8:
                MoveToWaypoint(waypoints_config.goal_point);
                break;
            case 9:
                MoveToWaypoint(waypoints_config.wp1);
                break;
            case 10:
                MoveToWaypoint(waypoints_config.wp2);
                break;
            case 11:
                MoveToWaypoint(waypoints_config.wp3);
                break;
        }
    }
}

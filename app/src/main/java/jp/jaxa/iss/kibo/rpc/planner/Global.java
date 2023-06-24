package jp.jaxa.iss.kibo.rpc.planner;

public class Global {
    public static int Nowplace = 0;
    public static int QRflag = 0;
    public static String report= "MISS";
    //0.5は30秒Over
    //0.75は10秒~1分ぐらい
    //1.0は50％ぐらい
    public static double RemainingTime = 1.2*60*1000; //残り時間
    public static double PhaseRemaintime = 0;
    public static boolean Finflag = false;
}

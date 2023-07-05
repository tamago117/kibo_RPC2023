package jp.jaxa.iss.kibo.rpc.planner;

public class Global {
    public static int Nowplace = 0;
    public static int QRflag = 0;
    public static String report= "MISS";
    //0.5は30秒Over
    //0.75は10秒~1分ぐらい
    //1.0は50％ぐらい
    public static long RemainingTime = 134*60*10; //残り時間(1.3*60*1000を変更した)
    public static long second_54 = 90*60*10;
    public static long second_45 = 75*60*10;
    public static long PhaseRemaintime = 0;
    public static boolean Finflag = false;
}

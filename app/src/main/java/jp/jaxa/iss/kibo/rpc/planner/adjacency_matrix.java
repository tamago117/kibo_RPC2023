package jp.jaxa.iss.kibo.rpc.planner;

public interface adjacency_matrix{
    // point1 ~ point7 => 1 ~ 7
    // goal => 8
    // wp1 ~ wp3 => 9 ~ 11
    double INF = Double.POSITIVE_INFINITY;
    double[][] graph = {
            {INF,INF,INF,INF,1.74895768959686,0.823279417937799,1.2416058150637,INF,INF,1.32305442064943,2.45462889252123},
            {INF,INF,INF,INF,INF,INF,INF,INF,0.63671265104441,0.785374432993589,0},
            {INF,INF,INF,INF,INF,INF,INF,INF,INF,1.15252895842144,0.802192620260246},
            {INF,INF,INF,INF,1.52542223662827,INF,INF,0.610897086259217,INF,2.09270662062316,0.856090532595707},
            {1.74895768959686,INF,INF,1.52542223662827,INF,INF,INF,INF,INF,0.757982849410197,0.750385234396307},
            {0.823279417937799,INF,INF,INF,INF,INF,0.459747756927643,INF,1.07557240574496,0.786123399982472,0},
            {1.2416058150637,INF,INF,INF,INF,0.459747756927643,INF,INF,1.46999591836168,0.877356256032862,0},
            {INF,INF,INF,0.610897086259217,INF,INF,INF,INF,INF,INF,0.974905969824783},
            {INF,0.63671265104441,INF,INF,INF,1.07557240574496,1.46999591836168,INF,INF,1.26394066316422,2.42851518422266},
            {1.32305442064943,0.785374432993589,1.15252895842144,2.09270662062316,0.757982849410197,0.786123399982472,0.877356256032862,INF,1.26394066316422,INF,1.24725699035924},
            {2.45462889252123,INF,0.802192620260246,0.856090532595707,0.750385234396307,INF,INF,0.974905969824783,2.42851518422266,1.24725699035924,0},
    };
}

package jp.jaxa.iss.kibo.rpc.planner;



public interface waypoints_config {
        // set Waypoint value
    // Write Position and Quaternion here.
    // Waypoint(pos_x, pos_y, pos_z, qua_x, qua_y, qua_z, qua_w, avoidX, avoidY, avoidZ)
    //X : right,left Y : back, front Z : down,up

        //point1 ~ 6 => 0~5
        //goal point => 6
        //wp1,2 => 7~8
        //start -> (wp1)
        //goal -> (point4, wp3)
        // point1 -> ( point5, point6, wp2, wp3)
        Waypoint point1 = new Waypoint(11.318, -9.8, 5.251,
                0.707, -0.707, 0, 0,
                0, 0, 0);
        // point2 -> (wp1, wp2)
        Waypoint point2 = new Waypoint(10.625, -9.144, 4.6,
                0, 0.707, 0, 0.707,
                0, 0, 0);    // Point
        // point3 -> (wp2, wp3)
        Waypoint point3 = new Waypoint(10.722, -7.769, 4.6,
                0, 0.707, 0, 0.707,
                0, 0, 0);    // Point
        // point4 -> (point5, wp2, wp3, goal)
        Waypoint point4 = new Waypoint(10.6, -6.616, 5.205,
                0, 0, 1, 0,
                0, 0, 0);    // Point
        // point5 -> (point1, point4, wp2, wp3)
        Waypoint point5 = new Waypoint(10.992, -8.084, 5.34,
                0, -0.707, 0, 0.707,
                0, 0, 0);    // Point
        // point6 -> (point1, wp1, wp2)
        Waypoint point6 = new Waypoint(11.21, -9.045, 4.941,
                0, 0, 0, 1,
                0, 0, 0);    // Point
        Waypoint point7 = new Waypoint(11.362, -8.615, 4.883,
                -0.5, 0.5, 0.5, 0.5,
                0, 0, 0);    // QRcode

        Waypoint goal_point = new Waypoint(11.143, -6.7607, 4.9654,
                0, 0, -0.707, 0.707,
                0, 0, 0);    // Point

        // wp1 -> (point2, point6, wp1, wp3, start)
        Waypoint wp1 = new Waypoint(10.508, -9.769, 4.567,
                0, 0, 0, 1,
                0, 0, 0);    // way point
        // wp2 -> (point1, point2, point3, point4, point5, point6, wp1, wp3)
        Waypoint wp2 = new Waypoint(10.742, -8.708, 5.251,
                -0.5, 0.5, 0.5, 0.5,
                0, 0, 0.0);    // way point
        //wp3 -> (point3, point4, point5, wp2, wp3, goal)
        Waypoint wp3 = new Waypoint(10.571, -7.463, 5.326,
                0, 0, -0.707, 0.707,
                0, 0, 0.0);    // way point


}

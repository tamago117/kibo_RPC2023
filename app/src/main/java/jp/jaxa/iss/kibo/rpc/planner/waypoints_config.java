package jp.jaxa.iss.kibo.rpc.planner;



public interface waypoints_config {
        // set Waypoint value
    // Write Position and Quaternion here.
    // Waypoint(pos_x, pos_y, pos_z, qua_x, qua_y, qua_z, qua_w, avoidX, avoidY, avoidZ)
    //X : right,left Y : back, front Z : down,up

        //start -> (wp1)
        //goal -> (point4, wp3)

        // point1 -> (point2, point5, point6, wp2)
        Waypoint point1 = new Waypoint(11.318, -9.053, 5.251,
                0.707, -0.707, 0, 0,
                0, 0, 0);
        // point2 -> (point1, point3, point4, point5, point6, wp1, wp2, goal)
        Waypoint point2 = new Waypoint(10.625, -9.144, 4.924,
                0, 0.707, 0, 0.707,
                0, 0, 0);    // Point
        // point3 -> (point2, point4, point5, point6, wp2, goal)
        Waypoint point3 = new Waypoint(10.722, -7.769, 4.9,
                0, 0.707, 0, 0.707,
                0, 0, 0);    // Point
        // point4 -> (point2, point3, point5, point6, wp2, goal)
        Waypoint point4 = new Waypoint(11.067, -6.616, 4.979,
                0, -0.102377467, 0.994745612, 0, //Quaternion(0,0.994745612,-0.102377467,0)
                0, 0, 0);    // Point
        // point5 -> (point1, point2, point3, point4, point6, wp2, goal)
        Waypoint point5 = new Waypoint(10.992, -8.084, 4.906,
                0, -0.707, 0, 0.707,
                0, 0, 0);    // Point
        // point6 -> (point1, point2, point3, point4, point5, wp1, wp2, goal)
        Waypoint point6 = new Waypoint(11.21, -9.045, 4.941,
                0, 0, 0, 1,
                0, 0, 0);    // Point

        Waypoint goal_point = new Waypoint(11.143, -6.7607, 4.9654,
                0, 0, -0.707, 0.707,
                0, 0, 0);    // Point


        //start -> wp1 -> wp2(QR code読み取り) -> 探索開始

        // wp1 -> (point2, point6, wp2, start)
        Waypoint wp1 = new Waypoint(10.508, -9.769, 4.567,
                0, 0, 0, 1,
                0, 0, 0);    // way point
        // wp2 -> (point1, point2, point3, point4, point5, point6, wp1, goal)
        Waypoint wp2 = new Waypoint(11.381, -9.036, 4.894,
                0.36710763, 0.36710763, -0.604344308, 0.604344308,
                0, 0, 0.0);    // way point + QR

}

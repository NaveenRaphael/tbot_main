from math import fabs
import rospy
import tf
import cvxpy
import numpy as np
from tbot_main.srv import positionRequest

'''
This code publishes the actual solution to the FTP and the asymptotitc solution of the FTP by considering the turtlebots 0 to n.
Additionally, a follower bot also follows the asymptotic solution.

TODO:
Get the number of bots to follow via rosparam

'''


def real_sol(pts):
    x = cvxpy.Variable(2)

    #Making the minimisation
    toMin = 0
    for i in pts:
        toMin = toMin + cvxpy.norm(x - i)
    
    objective = cvxpy.Minimize(toMin)
    constraints = []
    prob = cvxpy.Problem(objective, constraints)
    
    try:
        prob.solve()
    except cvxpy.SolverError:
        rospy.loginfo(f"Solver failed again")
        return 0, (0, 0)
    return prob.value, x.value


def broadcast_point(p):
    follower_name="/tb3_4"
    rospy.wait_for_service(f"{follower_name}/change_tbot_pos")
    try:
        request = rospy.ServiceProxy(f"{follower_name}/change_tbot_pos", positionRequest)
        resp = request(p[0], p[1])
        return resp.r
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node("FTP_test1")
    # num = rospy.get_param("num") #not working for some reason
    num = 2
    rospy.loginfo(f"{num}")
    # num=2

    pos = np.ndarray((num, 2))      #List of positions of the bots
    present = np.array([0, 0])      #present position of the asympt sol
    alpha = 1

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    last_time=rospy.Time.now()

    rate = rospy.Rate(10.0)
    numoreff=0
    while not rospy.is_shutdown():
        #Getting the new positions; if any of the positions are not found, this loop is ignored
        ignore=False
        for i in range(num):
            try:
                (trans, _) = listener.lookupTransform("world",
                f"tb{i}",
                rospy.Time(0))
                pos[i, 0], pos[i, 1] = trans[0], trans[1]
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as err:
                rospy.loginfo(f"Huh this is weird, {err}, {numoreff}")
                numoreff+=1
                ignore=True
                continue        
        if(ignore):
            rate.sleep()
            continue

        #Solution using cvxpy
        (_, sol_with_cvxpy) = real_sol(pos)
        br.sendTransform((sol_with_cvxpy[0], sol_with_cvxpy[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(), "cvpy_solution", "world")
      
        #Asymp solution
        present_time = rospy.Time.now()
        tdif=(present_time - last_time).to_sec()
        present = present + alpha * (np.mean(pos, 0) - present) * tdif
        last_time = present_time

        br.sendTransform((present[0], present[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(), "asymptotic_solution", "world")

        broadcast_point(present)
        rate.sleep()

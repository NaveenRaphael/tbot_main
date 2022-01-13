import rospy
import tf
import cvxpy
import numpy as np
from tbot_main.srv import positionRequest
'''
To Do:
1) Get the points of all the bots using tf
2) Get the actual solution using cvxpy
3) Get the Asymptotic solution using the algorithm
    -> Publish this in both tf and to the follower
'''


def real_sol(pts):
    x = cvxpy.Variable(2)
    toMin = 0
    # print(pts)
    for i in pts:
        toMin = toMin + cvxpy.norm(x - i)
    # print(toMin)
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
    return
    rospy.wait_for_service("??")
    try:
        request = rospy.ServiceProxy('??', positionRequest)
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

    pos = np.ndarray((num, 2))
    present = np.array([0, 0])
    alpha = 1

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    last_time = rospy.Time.now()

    rate = rospy.Rate(10.0)
    numoreff=0
    while not rospy.is_shutdown():
        # print("what")
        for i in range(num):
            try:
                (trans, _) = listener.lookupTransform(f"tb{i}", "world",
                                                      rospy.Time(0))
                # print(trans)
                pos[i, 0], pos[i, 1] = trans[0], trans[1]
                # rospy.loginfo_once(f"Position updated: {pos}")
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as err:
                rospy.loginfo(f"Huh this is weird, {err}, {numoreff}")
                numoreff+=1
                continue
            # print(pos)

        #Solution using cvxpy
        (_, sol_with_cvxpy) = real_sol(pos)
        # rospy.loginfo_once(f"Reached here!")
        br.sendTransform((sol_with_cvxpy[0], sol_with_cvxpy[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(), "cvpy_solution", "world")
        # rospy.loginfo_once(f"Reached here! boradcasting data!")

        #Asymp solution
        present_time = rospy.Time.now()
        present = present + alpha * (np.mean(pos, 0) - present) * (
            present_time - last_time).to_sec()
        last_time = present_time

        br.sendTransform((present[0], present[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(), "asymptotic_solution", "world")

        broadcast_point(present)
        rate.sleep()

#! /usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_euler  # Utilidad para convertir Euler a cuaterniones

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    # Suscripciones y publicaciones
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/mavros/setpoint_attitude/cmd_vel", TwistStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20)  # Frecuencia de publicación
    
    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # Definir posición inicial con orientación
    pose = PoseStamped()

    # Orientación: 180° en el eje Z (Yaw)
    #quaternion = quaternion_from_euler(0, 0, 3.14159)  # Roll=0, Pitch=0, Yaw=π
    #pose.pose.orientation.x = quaternion[0]
    #pose.pose.orientation.y = quaternion[1]
    #pose.pose.orientation.z = quaternion[2]
    #pose.pose.orientation.w = quaternion[3]

    # Definir velocidades iniciales
    velocity = TwistStamped()

    # Configuración inicial del modo y armado
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    start_time = rospy.Time.now()  # Inicio del temporizador

    while not rospy.is_shutdown():
        # Cambio al modo OFFBOARD
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()

        # Armado del vehículo
        elif not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if arming_client.call(arm_cmd).success:
                rospy.loginfo("Vehicle armed")
            last_req = rospy.Time.now()

        # Control de posición o velocidad según el tiempo
        elapsed_time = rospy.Time.now() - start_time
        if elapsed_time.to_sec() < 30.0:
            rospy.loginfo_throttle(5, f"Control de posición activo: {elapsed_time.to_sec():.2f} segundos")
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 2  # Altura deseada
            local_pos_pub.publish(pose)
        else:
            rospy.loginfo_throttle(5, f"Control de velocidad activo: {elapsed_time.to_sec():.2f} segundos")
            velocity.twist.linear.x = 0.05  # Velocidad en x
            velocity.twist.linear.y = 0.0  # Velocidad en y
            velocity.twist.linear.z = 0.08  # Velocidad en z
            velocity.twist.angular.x = 0.0  # Rotación en x
            velocity.twist.angular.y = 0.0  # Rotación en y
            velocity.twist.angular.z = 0.02  # Velocidad angular en z
            cmd_vel_pub.publish(velocity)

        rate.sleep()

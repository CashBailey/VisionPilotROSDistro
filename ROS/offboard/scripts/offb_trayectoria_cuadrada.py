#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def publish_velocity_with_fixed_alt(pub, vx, vy, z_fixed, yaw_rate):
    target = PositionTarget()
    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

    # Configuración para velocidades horizontales y altura fija
    target.type_mask = (
        PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |  # Ignorar posición X, Y
        PositionTarget.IGNORE_VZ |                             # Ignorar velocidad en Z
        PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
        PositionTarget.IGNORE_YAW_RATE                        # Ignorar velocidad angular de YAW
    )
    target.velocity.x = vx
    target.velocity.y = vy
    target.position.z = z_fixed  # Altura fija
    target.yaw = yaw_rate

    pub.publish(target)


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    altura_fija = 1.0  # Altura fija (metros)
    velocidad_base = 0.5     # Velocidad lineal en X y Y (m/s)
    duracion = 15.0      # Duración de cada movimiento (s)

    movimientos = [
        (velocidad_base, 0),    # Adelante en X
        (0, velocidad_base),    # Derecha en Y
        (-velocidad_base, 0),   # Atrás en X
        (0, -velocidad_base)    # Izquierda en Y
    ]

    while not rospy.is_shutdown():
        # Cambiar a modo OFFBOARD si no está activo
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()

        # Armar el dron si no está armado
        elif not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if arming_client.call(arm_cmd).success:
                rospy.loginfo("Vehicle armed")
            last_req = rospy.Time.now()

        # Publicar trayectorias cuadradas con disminución de velocidad
        for vx, vy in movimientos:
            rospy.loginfo(f"Moviendo: vx={vx}, vy={vy}")

            # Gradualmente reducir la velocidad conforme se acerca al final
            for t in range(int(duracion * 20)):
                factor = 1 - (t / (duracion * 20))  # Factor de reducción lineal
                rospy.loginfo(f"el factor es: {factor}")
                vx_actual = vx * factor
                vy_actual = vy * factor

                publish_velocity_with_fixed_alt(raw_pub, vx_actual, vy_actual, altura_fija, 0)
                rate.sleep()

        rate.sleep()

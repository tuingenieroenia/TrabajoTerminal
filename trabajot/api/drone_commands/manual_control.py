from pymavlink import mavutil

def mover_dron(drone, direccion, intensidad=0.5):
    """
    Mueve el dron en la dirección deseada con una intensidad limitada.
    :param direccion: 'adelante', 'atras', 'izquierda', 'derecha', 'subir', 'bajar', 'girar_izq', 'girar_der'
    """
    print(f"🕹️ Movimiento manual: {direccion}")

    if not drone:
        print("❌ No hay conexión con el dron.")
        return
    
    # Velocidades iniciales
    vx, vy, vz = 0, 0, 0

    # Valores base
    vel = 1.0 * intensidad  # m/s, ajustable
    vel_z = 0.5 * intensidad  # subir o bajar
    yaw_rate = 10 * intensidad  # grados/s

    if direccion == "adelante":
        vx, vy, vz = vel, 0, 0
    elif direccion == "atras":
        vx, vy, vz = -vel, 0, 0
    elif direccion == "izquierda":
        vx, vy, vz = 0, -vel, 0
    elif direccion == "derecha":
        vx, vy, vz = 0, vel, 0
    elif direccion == "subir":
        vx, vy, vz = 0, 0, -vel_z
    elif direccion == "bajar":
        vx, vy, vz = 0, 0, vel_z
    elif direccion == "girar_izq":
        vx, vy, vz = 0, 0, 0
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            -yaw_rate,  # Giro izquierda
            0, 1, 1, 0, 0, 0
        )
        return
    elif direccion == "girar_der":
        vx, vy, vz = 0, 0, 0
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            yaw_rate,  # Giro derecha
            0, 1, 1, 0, 0, 0
        )
        return
    else:
        print("⚠️ Dirección no reconocida, revisar.")
        return
    
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE * 0 |  # usar velocidad
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE * 0 |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE * 0 |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    drone.mav.set_position_target_local_ned_send(
        0,  # Tiempo
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0,  # Pos (ignorada)
        vx, vy, vz,  # Velocidades
        0, 0, 0,  # Aceleraciones (ignoradas)
        0, 0
    )

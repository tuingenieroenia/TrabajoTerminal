import asyncio
import json
from pymavlink import mavutil
import time
import math

def calcular_yaw_entre_puntos(lat1, lon1, lat2, lon2):
    """
    Calcula el ángulo de orientación (yaw) desde el punto (lat1, lon1)
    hacia el punto (lat2, lon2).
    """
    dlon = math.radians(lon2 - lon1)
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)

    x = math.sin(dlon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)

    initial_bearing = math.atan2(x, y)
    bearing = (math.degrees(initial_bearing) + 360) % 360
    return bearing


def obtener_ruta(folio):
    from api.models import RutasVuelo
    try:
        ruta = RutasVuelo.objects.get(folio=folio)
        coordenadas = ruta.coordenadas
        puntos = []

        for coord in coordenadas:
            lat = coord.get("lat")
            lon = coord.get("lon")
            print(f"mandamos {lat},{lon}")
            if lat is not None and lon is not None:
                puntos.append((lat, lon))

        return puntos
    except RutasVuelo.DoesNotExist:
        print("❌ Ruta no encontrada en la base de datos.")
        return []
    
def establecer_home(drone, lat, lon, alt=5):
    """
    Establece manualmente el punto de inicio (HOME) del dron en coordenadas lat/lon y altitud deseada.
    """
    print(f"📍 Estableciendo punto HOME en: {lat}, {lon}, alt: {alt}")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,          # Confirmation
        1,          # Use current location: 0 = usar coord personalizadas, 1 = usar actuales
        lat * 1e7,  # Param2: Lat (solo si Param1 es 0)
        lon * 1e7,  # Param3: Lon (solo si Param1 es 0)
        alt,        # Param4: Altitud
        0, 0, 0     # No usados
    )
    print("✅ Punto HOME establecido.")

def validar_estado_para_mision(drone):
    print("✅ Validando estado del dron antes de misión...")

    # 1. Verificar señal GPS
    gps = drone.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
    if not gps or gps.fix_type < 3:
        return {"ok": False, "mensaje": "GPS sin señal 3D (fix_type < 3)"}

    # 2. Verificar batería
    estado = drone.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
    if not estado or (estado.battery_remaining is not None and estado.battery_remaining < 20):
        return {"ok": False, "mensaje": "Batería insuficiente (<20%)"}

    # Puedes agregar más validaciones si lo deseas

    return {"ok": True}
    
async def enviar_mision(drone, waypoints, altitud=5):
    if not drone or not waypoints:
        print("❌ Conexión al dron o waypoints no válidos.")
        return

    print("📋 Iniciando flujo de misión automática...")

    # 1. Validar estado del dron
    validacion = validar_estado_para_mision(drone)
    if not validacion["ok"]:
        print(f"❌ Cancelado: {validacion['mensaje']}")
        return

    # 2. Establecer HOME
    gps_msg = drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if gps_msg:
        lat = gps_msg.lat / 1e7
        lon = gps_msg.lon / 1e7
        alt = gps_msg.relative_alt / 1000
        establecer_home(drone, lat, lon, alt)
    else:
        print("❌ No se pudo obtener posición para HOME.")
        return

    # 3. Cambiar a modo GUIDED
    print("🔁 Cambiando a modo GUIDED...")
    drone.mav.set_mode_send(
        drone.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED
    )
    await asyncio.sleep(2)

    # 4. Armar el dron
    print("🛡️ Armando el dron...")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    await asyncio.sleep(2)

    # 5. Limpiar misión anterior
    print("🧹 Limpiando misiones anteriores...")
    drone.mav.mission_clear_all_send(
        drone.target_system,
        drone.target_component
    )
    await asyncio.sleep(1)

    # 6. Preparar misión
    print("📦 Preparando waypoints...")
    yaw_inicial = calcular_yaw_entre_puntos(*waypoints[0], *waypoints[1])
    drone.waypoint_list = []
    for i, (lat, lon) in enumerate(waypoints):
        yaw = yaw_inicial if i == 0 else 0 
        wp = drone.mav.mission_item_int_encode(
            drone.target_system,
            drone.target_component,
            i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            1 if i == 0 else 0,
            1,
            0, 0, yaw, 0,
            int(lat * 1e7),
            int(lon * 1e7),
            altitud
        )
        drone.waypoint_list.append(wp)
        print(f"✅ WP{i}: ({lat}, {lon})")
    # 🛬 Agregar waypoint de aterrizaje
    ultimo_lat, ultimo_lon = waypoints[-1]
    wp_land = drone.mav.mission_item_int_encode(
        drone.target_system,
        drone.target_component,
        len(waypoints),  # último índice
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,  # current
        1,  # autocontinue
        0, 0, 0, 0,
        int(ultimo_lat * 1e7),
        int(ultimo_lon * 1e7),
        0  # Altitud objetivo para aterrizaje
    )
    drone.waypoint_list.append(wp_land)
    print(f"✅ Waypoint final de aterrizaje añadido en ({ultimo_lat}, {ultimo_lon})")

    # 7. Enviar handshake inicial
    print("📨 Solicitando lista de misiones al dron (MISSION_REQUEST_LIST)...")
    drone.mav.mission_request_list_send(drone.target_system, drone.target_component)
    await asyncio.sleep(1)

    # 8. Enviar número de waypoints
    print(f"📤 Enviando número total de waypoints: {len(drone.waypoint_list)}")
    drone.mav.mission_count_send(
        drone.target_system,
        drone.target_component,
        len(drone.waypoint_list)
    )

    # 9. Enviar los waypoints según solicitud
    enviados = 0
    timeout = time.time() + 30
    print("⏳ Esperando solicitudes de misión...")
    while enviados < len(drone.waypoint_list) and time.time() < timeout:
        msg = drone.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=2)
        if msg:
            seq = msg.seq
            print(f"📍 Enviando waypoint {seq + 1}/{len(drone.waypoint_list)}...")

            if msg.get_type() == 'MISSION_REQUEST_INT':
                drone.mav.send(drone.waypoint_list[seq])
            elif msg.get_type() == 'MISSION_REQUEST':
                wp = drone.waypoint_list[seq]
                # Convertir a MISSION_ITEM si es necesario (esto depende del firmware, pero en general puede aceptarlo igual)
                drone.mav.send(wp)  # normalmente acepta el mismo mensaje
            enviados += 1
        else:
            print("⏳ Aún sin solicitud de waypoint...")

    # 10. Confirmación final
    ack = drone.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if not ack:
        print("❌ No se recibió MISSION_ACK.")
        return
    print("✅ Misión cargada con éxito.")

    # 11. Despegue
    print(f"🚀 Despegando a {altitud} metros...")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, yaw_inicial, 0, 0, altitud
    )
    await asyncio.sleep(6)

    # 12. Cambiar a AUTO
    print("🔄 Cambiando a modo AUTO...")
    drone.mav.set_mode_send(
        drone.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        3  # AUTO
    )
    await asyncio.sleep(2)

    # 13. Iniciar misión
    print("▶️ Enviando MISSION_START...")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    print("🟢 Misión iniciada correctamente.")

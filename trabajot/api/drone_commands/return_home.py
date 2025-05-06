from pymavlink import mavutil
import time

def establecer_parametro(drone, nombre, valor):
    """
    Configura un parámetro en el dron.
    """
    print(f"🔧 Configurando {nombre} a {valor}...")
    try:
        drone.mav.param_set_send(
            drone.target_system,
            drone.target_component,
            nombre.encode(),
            float(valor),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        print(f"✅ {nombre} configurado correctamente.")
    except Exception as e:
        print(f"❌ Error al configurar {nombre}: {e}")

def regresar_home_seguro(drone):
    """
    Ejecuta una secuencia segura para retornar al punto de inicio (RTL).
    """
    try:
        print("🛑 Preparando regreso a casa...")

        # Configurar altitud de retorno y altitud final
        establecer_parametro(drone, "RTL_ALT", 600)          # 6 metros en centímetros
        establecer_parametro(drone, "RTL_ALT_FINAL", 0)      # 0 para aterrizar al llegar

        # Enviar comando RTL directamente
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print("🏠 Comando RTL enviado.")

        return True

    except Exception as e:
        print(f"❌ Error en regreso a casa seguro: {e}")
        return False
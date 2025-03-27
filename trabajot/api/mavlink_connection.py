from pymavlink import mavutil

# Definir la variable global correctamente
drone_connection = None

def get_drone_connection():
    """Retorna la conexión global al dron o la crea si no existe."""
    global drone_connection  # ← Asegura que use la variable global

    if drone_connection is None:
        try:
            print("🔄 Estableciendo conexión con el dron...")
            # Selecciona el tipo de conexión (SITL o real)
            #drone_connection = mavutil.mavlink_connection("COM9", baud=57600)  # Para dron real
            drone_connection = mavutil.mavlink_connection("tcp:127.0.0.1:5762")  # Para SITL
            # Esperar hasta recibir el heartbeat
            drone_connection.wait_heartbeat()
            print("✅ Conectado al dron")
        except Exception as e:
            print(f"❌ Error conectando al dron: {e}")
            drone_connection = None

    return drone_connection
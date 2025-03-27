import asyncio
import json
from channels.generic.websocket import AsyncWebsocketConsumer
from pymavlink import mavutil
from api.mavlink_connection import get_drone_connection

class TelemetriaConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        """Maneja la conexión WebSocket e inicia la transmisión de telemetría."""
        await self.accept()
        
        self.drone = get_drone_connection()  # Usa la conexión global

        if self.drone is None:
            print("❌ No se pudo conectar al dron.")
            await self.send(json.dumps({"error": "No se pudo conectar al dron"}))
            await self.close()
            return

        self.running = True
        self.telemetry_task = asyncio.create_task(self.enviar_telemetria())

    async def receive(self, text_data):
        """Recibe comandos desde el frontend y los ejecuta en el dron."""
        try:
            data = json.loads(text_data)
            command = data.get("command")

            if command == "test_motores":
                self.test_motores()
            elif command == "armar":
                self.armar_dron()
            elif command == "despegar":
                self.despegar()
            elif command == "regresar_home":
                self.regresar_home()
            elif command == "aterrizar":
                self.aterrizar()

            await self.send(json.dumps({"status": "Comando ejecutado", "command": command}))

        except Exception as e:
            print(f"❌ Error al procesar comando: {e}")
            await self.send(json.dumps({"error": f"Error al ejecutar comando: {str(e)}"}))

    def test_motores(self):
        """Realiza un testeo de motores sin despegar el dron."""
        if self.drone:
            print("⚙️ Testeando motores...")
            try:
                self.drone.mav.command_long_send(
                    self.drone.target_system,   
                    self.drone.target_component,  
                    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,  # Comando de prueba de motores
                    0,    # Confirmation
                    0,    # Motor index (0 = todos los motores)
                    2,    # Tipo de prueba (2 = porcentaje de aceleración)
                    15,   # Throttle (%) (ejemplo: 50% de potencia)
                    5,    # Duración en segundos
                    0,    # Param5 (No usado, poner 0)
                    0,    # Param6 (No usado, poner 0)
                    0     # Param7 (No usado, poner 0)
                )
                print("✅ Comando de prueba de motores enviado correctamente.")
            except Exception as e:
                print(f"❌ Error al procesar comando: {e}")

    def armar_dron(self):
        """Arma el dron."""
        if self.drone:
            print("🔍 Verificando estado del dron antes de armar...")

            try:
                # Solicita el estado actual del dron (HEARTBEAT)
                self.drone.mav.command_long_send(
                    self.drone.target_system,  # Target System ID
                    self.drone.target_component,  # Target Component ID
                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Comando para solicitar un mensaje
                    0,  # Confirmation
                    mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,  # ID del mensaje a solicitar
                    0, 0, 0, 0, 0, 0  # Parámetros vacíos (requeridos en MAVLink v2)
                )

                # Espera la respuesta con un timeout
                estado = self.drone.recv_match(type="HEARTBEAT", blocking=True, timeout=2)

                if estado:
                    armed = (estado.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    print(f"🔎 Estado actual: {'ARMADO' if armed else 'DESARMADO'}")

                print("🔧 Intentando armar motores...")
                self.drone.mav.command_long_send(
                    self.drone.target_system,
                    self.drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Comando para armar
                    0,  # Confirmation
                    1,  # Param1: 1 para ARMAR, 0 para DESARMAR
                    0, 0, 0, 0, 0, 0  # Parámetros vacíos
                )
                
                print("✅ Comando de armado enviado.")

            except Exception as e:
                print(f"❌ Error al procesar comando de armado: {e}")

    def set_modo_guided(self):
        """Cambia el modo de vuelo a GUIDED."""
        print("🔄 Configurando modo de vuelo GUIDED...")

        try:
            self.drone.mav.command_long_send(
                self.drone.target_system,  # Target System ID
                self.drone.target_component,  # Target Component ID
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # Comando para cambiar el modo de vuelo
                0,  # Confirmation
                1,  # Param1: Base mode (MAV_MODE_FLAG_CUSTOM_MODE no se usa aquí)
                4,  # Param2: Modo GUIDED (4 en ArduPilot)
                0, 0, 0, 0, 0  # Parámetros vacíos
            )
            asyncio.sleep(2)  # Esperar que el cambio de modo se aplique

            print("✅ Modo GUIDED activado.")
        except Exception as e:
            print(f"❌ Error al cambiar a modo GUIDED: {e}")

    def set_altitud_maxima(self, altitud_maxima=1):
        """Configura la altitud máxima permitida para el dron."""
        print(f"🔒 Configurando altitud máxima a {altitud_maxima} metros...")

        try:
            self.drone.mav.param_set_send(
                self.drone.target_system,
                self.drone.target_component,
                b"FENCE_ALT_MAX",  # Parámetro de altitud máxima
                float(altitud_maxima),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            print("✅ Altitud máxima configurada.")
        except Exception as e:
            print(f"❌ Error al configurar la altitud máxima: {e}")

    def mantener_en_posicion(self):
        """Mantiene el dron en su posición después del despegue."""
        print("🛑 Manteniendo dron en su posición...")

        try:
            gps_status = self.drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
            if gps_status:
                latitud = gps_status.lat / 1e7
                longitud = gps_status.lon / 1e7
                altitud = gps_status.relative_alt / 1000

                # Enviar comando para mantenerlo en esa posición
                self.drone.mav.set_position_target_global_int_send(
                    0,  # Tiempo de transmisión (0 para inmediato)
                    self.drone.target_system,
                    self.drone.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    int(0b0000111111111000),  # Solo afecta x, y, z
                    int(latitud * 1e7),  # Latitud
                    int(longitud * 1e7),  # Longitud
                    altitud,  # Altitud actual
                    0, 0, 0,  # Velocidades en x, y, z (0 para mantener en su sitio)
                    0, 0, 0,  # Aceleraciones en x, y, z
                    0,  # Yaw
                    0  # Yaw rate
                )

                print("✅ Dron fijado en su posición.")

        except Exception as e:
            print(f"❌ Error al fijar el dron en su posición: {e}")

    def esperar_armado(self):
        """Espera hasta que el dron esté armado."""
        print("⏳ Esperando que el dron se arme...")

        for _ in range(10):  # Intentar durante 10 ciclos (5 segundos)
            estado = self.drone.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if estado:
                armed = (estado.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                if armed:
                    print("✅ Dron armado correctamente.")
                    return True
            asyncio.sleep(0.5)

        print("⚠️ Tiempo de espera agotado. El dron no se armó.")
        return False
    
    def verificar_restricciones(self):
        """Solicita los mensajes de estado para ver si hay restricciones activas."""
        print("🔎 Verificando posibles restricciones...")

        estado = self.drone.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
        gps = self.drone.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)

        if estado:
            if estado.errors_count1 > 0:
                print("⚠️ Hay errores en el sistema. Revisa en Mission Planner.")
            else:
                print("✅ No hay errores críticos reportados.")

        if gps:
            if gps.fix_type < 3:
                print("⚠️ Señal GPS insuficiente (Fix Type:", gps.fix_type, "). Se necesita 3D Fix.")
            else:
                print("✅ GPS con señal suficiente para el vuelo.")

    def despegue_controlado(self, altura_objetivo=1):
        """Despega progresivamente sin usar MAV_CMD_NAV_TAKEOFF para evitar arranques bruscos."""
        if self.drone:
            print(f"🚀 Iniciando despegue controlado a {altura_objetivo} metros...")

            try:
                # ✅ Verificar si el dron está armado
                estado = self.drone.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                armed = estado and (estado.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                if not armed:
                    print("⚠️ El dron no está armado. Armando automáticamente...")
                    self.armar_dron()
                    asyncio.sleep(5)  # Esperar a que termine de armarse

                # ✅ Verificar señal GPS antes de despegar
                gps_status = self.drone.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
                if not gps_status or gps_status.fix_type < 3:
                    print("⚠️ Señal GPS insuficiente para despegar.")
                    return

                # ✅ Configurar modo GUIDED antes de despegar
                self.set_modo_guided()

                # ✅ Aumento progresivo del throttle para evitar arranque brusco
                print("⚙️ Aumentando potencia de motores gradualmente...")
                throttle = 1100  # PWM inicial (necesario para iniciar motores)
                max_throttle = 1350  # Ajustar según el peso del dron

                while throttle < max_throttle:
                    for motor in range(1, 5):  # Asumiendo 4 motores
                        self.drone.mav.command_long_send(
                            self.drone.target_system,
                            self.drone.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0,
                            motor,  # Canal del motor
                            throttle,  # PWM incrementado progresivamente
                            0, 0, 0, 0, 0
                        )
                    throttle += 25  # Incremento gradual (ajustable)
                    asyncio.sleep(0.5)  # Esperar antes del siguiente incremento

                print("✅ Potencia establecida, verificando altitud...")

                # ✅ Monitorear la altitud en tiempo real para evitar sobre aceleración
                while True:
                    estado = self.drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
                    if estado and estado.relative_alt >= (altura_objetivo * 1000):  # Factor de conversión de altitud
                        print(f"✅ Altura objetivo alcanzada: {altura_objetivo}m")
                        break
                    asyncio.sleep(0.5)

                print("🛑 Estabilizando en altura...")

            except Exception as e:
                print(f"❌ Error durante el despegue controlado: {e}")

    def despegar(self, altura=1):
        """Ejecuta el despegue controlado con ajuste progresivo de motores"""
        print(f"🚀 Iniciando secuencia de despegue con control de motores...")
        self.despegue_controlado(altura)  # 🔹 Llama a la función de despegue progresivo

    def aterrizar(self):
        """Secuencia completa de aterrizaje."""
        if self.drone:
            print("🛬 Iniciando aterrizaje...")

            try:
                # Configurar modo LAND
                print("🔄 Configurando modo LAND...")
                self.drone.mav.command_long_send(
                    self.drone.target_system,
                    self.drone.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                print("✅ Comando de aterrizaje enviado.")

                # Esperar a que el dron aterrice completamente
                while True:
                    estado = self.drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
                    if estado and estado.relative_alt < 10:  # Si la altitud es menor a 1 metro
                        print("🛑 Dron aterrizado completamente.")
                        break
                    asyncio.sleep(1)

                # Desarmar después del aterrizaje
                print("🔧 Desarmando motores...")
                self.drone.mav.command_long_send(
                    self.drone.target_system,
                    self.drone.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                print("✅ Dron desarmado.")

                # Notificar al frontend que aterrizó
                self.send(json.dumps({"status": "aterrizado"}))

            except Exception as e:
                print(f"❌ Error durante el proceso de aterrizaje: {e}")

    def regresar_home(self):
        """Regresa el dron al punto de origen."""
        if self.drone:
            print("🏠 Regresando a Home...")
            self.drone.mav.command_long_send(
                self.drone.target_system,
                self.drone.target_component,
                20,  # MAV_CMD_NAV_RETURN_TO_LAUNCH
                0, 0, 0, 0, 0, 0, 0
            )

    async def enviar_telemetria(self):
        """Envía datos de telemetría en tiempo real."""
        try:
            while self.running:
                if self.drone is None:
                    print("❌ Conexión perdida con el dron. Deteniendo telemetría.")
                    await self.close()
                    break

                msg = await asyncio.to_thread(self.drone.recv_match, type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'BATTERY_STATUS', 'SYS_STATUS', 'GPS_RAW_INT'], blocking=True, timeout=1)
                if msg:
                    #print(f"📡 Mensaje recibido: {msg}")  # LOG para verificar si llegan datos

                    data = {
                        "tipo": msg.get_type(),
                        "pitch": getattr(msg, "pitch", None),
                        "roll": getattr(msg, "roll", None),
                        "yaw": getattr(msg, "yaw", None),
                        "lat": getattr(msg, "lat", None) / 1e7 if msg.get_type() == "GPS_RAW_INT" else None,
                        "lon": getattr(msg, "lon", None) / 1e7 if msg.get_type() == "GPS_RAW_INT" else None,
                        "alt": getattr(msg, "alt", None) / 1000 if msg.get_type() == "GPS_RAW_INT" else None,
                        "alt_agl": getattr(msg, "relative_alt", None) / 1000 if msg.get_type() == "GLOBAL_POSITION_INT" else None,
                        "vx": getattr(msg, "vx", 0),
                        "vy": getattr(msg, "vy", 0),
                        "vz": getattr(msg, "vz", 0),
                        "bateria": getattr(msg, "battery_remaining", 0) if msg.get_type() in ["SYS_STATUS", "BATTERY_STATUS"] else None,
                        "voltaje": getattr(msg, "voltage_battery", 0) / 1000 if getattr(msg, "voltage_battery", None) else None,
                        "modo": self.get_flight_mode(msg),
                        "satellites": getattr(msg, "satellites_visible", None) if msg.get_type() == "GPS_RAW_INT" else None,
                        "gps_fix": getattr(msg, "fix_type", None) if msg.get_type() == "GPS_RAW_INT" else None,
                    }

                    await self.send(json.dumps(data))  # Envía la telemetría al frontend
                    #print(f"✅ Enviando telemetría: {data}")  # LOG para confirmar envío de datos

                else:
                    print("⚠ No se recibieron datos de telemetría en este ciclo.")  # LOG si no hay datos

                await asyncio.sleep(0.5)

        except asyncio.CancelledError:
            print("🛑 Tarea de telemetría cancelada.")
        except Exception as e:
            print(f"❌ Error en la telemetría: {e}")

    def get_flight_mode(self, msg):
        """Devuelve el modo de vuelo actual si está disponible."""
        if msg.get_type() == "HEARTBEAT":
            flight_modes = {
                0: "Manual",
                1: "Circle",
                2: "Stabilize",
                3: "Training",
                4: "Acro",
                5: "FBWA",
                6: "FBWB",
                7: "Cruise",
                8: "Autotune",
                10: "Auto",
                11: "RTL",
                12: "Loiter",
                15: "Guided",
                16: "Initialising",
            }
            return flight_modes.get(msg.custom_mode, "En sitio")
        return None

    async def disconnect(self, close_code):
        """Cierra la conexión de manera ordenada."""
        print(f"🔴 WebSocket desconectado: {close_code}")
        self.running = False
        if hasattr(self, "telemetry_task"):
            self.telemetry_task.cancel()
            try:
                await self.telemetry_task
            except asyncio.CancelledError:
                pass
        if hasattr(self, "drone"):
            self.drone.close()

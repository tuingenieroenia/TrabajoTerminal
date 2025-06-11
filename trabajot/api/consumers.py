import asyncio
import json
import math
from channels.generic.websocket import AsyncWebsocketConsumer
from pymavlink import mavutil
from api.mavlink_connection import get_drone_connection, websocket_clients
from api.drone_commands.manual_control import mover_dron
from api.drone_commands.mission_manager import enviar_mision, obtener_ruta, establecer_home, validar_estado_para_mision
from api.drone_commands.return_home import cambiar_a_loiter

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

        websocket_clients.add(self)  # ✅ Añadir cliente a lista global
        self.movimiento_activo = None
        self.task_movimiento = None
        self.running = True
        self.mision_en_curso = False

        if len(websocket_clients) == 1:
            self.telemetry_task = asyncio.create_task(self.enviar_telemetria())
        #self.telemetry_task = asyncio.create_task(self.enviar_telemetria())
        await self.enviar_estado_inicial()

    async def enviar_estado_inicial(self):
        """Envía el estado actual del dron al cliente recién conectado."""
        try:
            print("📤 Enviando estado inicial al nuevo cliente...")

            # Último modo
            modo = self.ultimo_modo if hasattr(self, "ultimo_modo") else "Desconocido"

            # Posición actual
            posicion = self.drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
            alt_agl = posicion.relative_alt / 1000 if posicion else None

            # Nivel de batería
            bateria = self.drone.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
            nivel_bateria = bateria.battery_remaining if bateria else None

            estado = {
                "evento": "estado_inicial",
                "modo": modo,
                "alt_agl": alt_agl,
                "bateria": nivel_bateria,
                "status": "en_mision" if self.mision_en_curso else "en_estacion_base"
            }

            await self.send(json.dumps(estado))
            print(f"✅ Estado inicial enviado: {estado}")

        except Exception as e:
            print(f"❌ Error al enviar estado inicial: {e}")

    async def receive(self, text_data):
        """Recibe comandos desde el frontend y los ejecuta en el dron."""
        try:
            data = json.loads(text_data)
            command = data.get("command")

            if command == "test_motores":
                self.test_motores()
            elif command == "armar":
                self.armar_dron()
                await asyncio.sleep(5)
                self.set_modo_guided()
            elif command == "despegar":
                self.despegar()
            elif command == "regresar_home":
                try:
                    success = cambiar_a_loiter(self.drone)
                    if success:
                        self.mision_en_curso = False  # 💥 Detener estado de misión
                        await self.send(text_data=json.dumps({
                            "evento": "estado",
                            "valor": "En espera (LOITER)",
                            "mensaje": "✅ Dron cambiado a modo LOITER."
                        }))
                    else:
                        await self.send(text_data=json.dumps({
                            "evento": "error",
                            "valor": "No se pudo cambiar a LOITER",
                            "mensaje": "❌ El dron no respondió al cambio de modo."
                        }))
                except Exception as e:
                    await self.send(text_data=json.dumps({
                        "evento": "error",
                        "valor": "Error inesperado",
                        "mensaje": f"❌ Ocurrió un error al cambiar a LOITER: {e}"
                    }))
            elif command == "aterrizar":
                self.aterrizar()
            elif command in ["adelante", "atras", "izquierda", "derecha", "subir", "bajar", "girar_izq", "girar_der"]:
                self.iniciar_movimiento_continuo(command)
            elif command == "detener":
                self.detener_movimiento()
            elif command == "iniciar_mision":
                folio_ruta = data.get("folio_ruta", 1)
                optimizar = data.get("optimizar", False)
                print(f"🧭 Iniciando misión con folio {folio_ruta} | optimizar={optimizar}")
                waypoints = await asyncio.to_thread(obtener_ruta, folio_ruta, optimizar)
                if waypoints:
                    self.total_waypoints = len(waypoints)
                    self.mision_en_curso = True
                    await asyncio.gather(
                        enviar_mision(self.drone, waypoints, 5)
                    )
                else:
                    await self.broadcast_to_clients({"error": f"Error al ejecutar comando: {str(e)}"})

            await self.broadcast_to_clients({"status": "Comando ejecutado", "command": command})

        except Exception as e:
            print(f"❌ Error al procesar comando: {e}")
            await self.send(json.dumps({"error": f"Error al ejecutar comando: {str(e)}"}))

    def iniciar_movimiento_continuo(self, direccion):
        print(f"🔁 Iniciando movimiento continuo: {direccion}")
        self.movimiento_activo = direccion
        if self.task_movimiento and not self.task_movimiento.done():
            self.task_movimiento.cancel()

        self.task_movimiento = asyncio.create_task(self._loop_movimiento())

    async def _loop_movimiento(self):
        try:
            while self.movimiento_activo:
                mover_dron(self.drone, self.movimiento_activo)
                await asyncio.sleep(0.2)  # 200 ms entre comandos
        except asyncio.CancelledError:
            print("🛑 Movimiento continuo cancelado.")

    def detener_movimiento(self):
        print("⛔ Deteniendo movimiento")
        self.movimiento_activo = None

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

    def despegue_controlado(self, altura_objetivo=3):
        """Despega usando MAV_CMD_NAV_TAKEOFF (funciona en SITL)."""
        if self.drone:
            print(f"🚀 Ejecutando despegue SITL a {altura_objetivo} metros...")

            try:
                # Asegurarse de que está armado
                estado = self.drone.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                armed = estado and (estado.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if not armed:
                    print("⚠️ El dron no está armado. Armando automáticamente...")
                    self.armar_dron()
                    asyncio.sleep(5)

                # Cambiar a modo GUIDED
                self.set_modo_guided()
                asyncio.sleep(2)

                # Enviar comando de despegue
                self.drone.mav.command_long_send(
                    self.drone.target_system,
                    self.drone.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0, 0, 0, 0, 0, 0,  # Lat, Lon opcionales
                    altura_objetivo  # Altitud objetivo
                )

                print("🛫 Comando de despegue enviado, esperando alcanzar altitud...")

                # Esperar a alcanzar la altitud
                while True:
                    estado = self.drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
                    if estado and estado.relative_alt >= (altura_objetivo * 1000):  # en mm
                        print(f"✅ Altura alcanzada: {estado.relative_alt / 1000}m")
                        break
                    asyncio.sleep(0.5)

            except Exception as e:
                print(f"❌ Error durante el despegue: {e}")

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
        # Variables de umbrales
        BAT_LOW_WARNING = 25  # %
        BAT_LOW_CRITICAL = 15  # %
        GPS_FIX_MIN = 3
        SAT_MIN = 6
        """Envía datos de telemetría en tiempo real."""
        try:
            self.ultimo_modo = "Desconocido"
            self.wp_final_alcanzado = False

            while self.running:
                if self.drone is None:
                    print("❌ Conexión perdida con el dron. Deteniendo telemetría.")
                    await self.close()
                    break

                msg = await asyncio.to_thread(self.drone.recv_match, 
                                              type=['MISSION_ITEM_REACHED',
                                                    'MISSION_CURRENT', 
                                                    'ATTITUDE', 
                                                    'GLOBAL_POSITION_INT', 
                                                    'VFR_HUD', 
                                                    'BATTERY_STATUS', 
                                                    'SYS_STATUS',
                                                    'HEARTBEAT', 
                                                    'GPS_RAW_INT'],
                                              blocking=True, 
                                              timeout=1)
                if msg:
                    #print(f"📡 Mensaje recibido: {msg}")  # LOG para verificar si llegan datos
                    tipo = msg.get_type()

                    #data de misiones para actualizar status en el frontend
                    
                    # 1. Waypoint alcanzado
                    if tipo == 'MISSION_ITEM_REACHED':
                        print(f"📍 Waypoint alcanzado: {msg.seq}")
                        await self.broadcast_to_clients({
                            "evento": "waypoint_alcanzado",
                            "wp": msg.seq
                        })

                        if hasattr(self, 'total_waypoints'):
                            if msg.seq == self.total_waypoints - 1:  # El anterior al LAND
                                print("🛬 Último waypoint alcanzado. Iniciando secuencia de aterrizaje...")
                                self.wp_final_alcanzado = True
                                await self.broadcast_to_clients({"status": "aterrizando"})
                            elif msg.seq >= self.total_waypoints:
                                print("🏁 Misión finalizada tras último waypoint.")
                                self.mision_en_curso = False
                                self.wp_final_alcanzado = False
                                await self.broadcast_to_clients({"status": "mision_finalizada"})

                    # 2. Detectar aterrizaje real
                    if self.mision_en_curso and tipo == "GLOBAL_POSITION_INT":
                        altitud = getattr(msg, "relative_alt", 99999) / 1000
                        # Detectar despegue (ej. si altitud sube por encima de 0.5 y aún no se ha notificado)
                        if self.total_waypoints > 0 and self.ultimo_wp_actual == 0 and altitud > 0.5:
                            print("🛫 Despegue detectado...")
                            await self.broadcast_to_clients({
                                "status": "despegando",
                                "evento": "despegando"
                            })
                        # Detectar aterrizaje tras misión
                        if self.wp_final_alcanzado and altitud < 1:
                            print("🏁 Dron aterrizó completamente tras misión.")
                            self.mision_en_curso = False
                            self.wp_final_alcanzado = False
                            await self.broadcast_to_clients({
                                "status": "mision_finalizada",
                                "evento": "fin_mision"
                            })
                    
                    # 3. Waypoint actual (log)
                    if tipo == 'MISSION_CURRENT':
                        #print(f"🔄 Waypoint actual: {msg.seq}")
                        if hasattr(self, "ultimo_wp_actual") and self.ultimo_wp_actual == 0 and msg.seq == 1:
                            print("🚀 El dron comenzó su misión tras el despegue.")
                            await self.broadcast_to_clients({
                                "evento": "iniciando_mision",
                                "status": "iniciando"
                            })
                        self.ultimo_wp_actual = msg.seq

                    if tipo == "HEARTBEAT":
                        modo_actual = self.get_flight_mode(msg)
                        if modo_actual:
                            #if modo_actual != self.ultimo_modo:
                                #print(f"🧭 Modo actual detectado: {modo_actual}")
                            self.ultimo_modo = modo_actual

                    #if tipo == "ATTITUDE":
                        #print(f"🎯 Yaw recibido: {msg.yaw}")

                    #telemetria general
                    data = {
                        "tipo": tipo,
                        "pitch": getattr(msg, "pitch", None),
                        "roll": getattr(msg, "roll", None),
                        "yaw": math.degrees(getattr(msg, "yaw", 0)) % 360 if msg.get_type() == "ATTITUDE" else None,
                        "lat": getattr(msg, "lat", None) / 1e7 if msg.get_type() == "GPS_RAW_INT" else None,
                        "lon": getattr(msg, "lon", None) / 1e7 if msg.get_type() == "GPS_RAW_INT" else None,
                        "alt": getattr(msg, "alt", None) / 1000 if msg.get_type() == "GPS_RAW_INT" else None,
                        "alt_agl": getattr(msg, "relative_alt", None) / 1000 if msg.get_type() == "GLOBAL_POSITION_INT" else None,
                        "vx": getattr(msg, "vx", 0),
                        "vy": getattr(msg, "vy", 0),
                        "vz": getattr(msg, "vz", 0),
                        "bateria": getattr(msg, "battery_remaining", 0) if msg.get_type() in ["SYS_STATUS", "BATTERY_STATUS"] else None,
                        "voltaje": getattr(msg, "voltage_battery", 0) / 1000 if getattr(msg, "voltage_battery", None) else None,
                        "modo": self.ultimo_modo,
                        "satellites": getattr(msg, "satellites_visible", None) if msg.get_type() == "GPS_RAW_INT" else None,
                        "gps_fix": getattr(msg, "fix_type", None) if msg.get_type() == "GPS_RAW_INT" else None,
                    }

                    # 👉 Determinar tipo_operacion desde backend
                    tipo_operacion = "Desconocido"
                    if self.mision_en_curso:
                        if self.wp_final_alcanzado:
                            tipo_operacion = "Aterrizando..."
                        elif getattr(self, "ultimo_wp_actual", 0) == 0:
                            tipo_operacion = "Despegando..."
                        else:
                            tipo_operacion = "Automática"
                    elif self.ultimo_modo in ["Guided", "Stabilize", "Loiter", "Manual", "Training"]:
                        tipo_operacion = "Manual"
                    elif self.ultimo_modo == "Auto":
                        tipo_operacion = "Automática"
                    elif self.ultimo_modo == "Land":
                        tipo_operacion = "Aterrizando..."
                    elif self.ultimo_modo == "RTL":
                        tipo_operacion = "Retornando"
                    else:
                        tipo_operacion = "En estación base"

                    data["tipo_operacion"] = tipo_operacion

                     # 👇 Lógica de seguridad basada en batería
                    if msg.get_type() in ["SYS_STATUS", "BATTERY_STATUS"]:
                        battery = getattr(msg, "battery_remaining", None)
                        if battery is not None:
                            if battery < BAT_LOW_CRITICAL:
                                print("🔴 Nivel de batería crítico. Ejecutando RTL.")
                                self.regresar_home()
                                await self.broadcast_to_clients({"alerta": "Batería crítica. Regresando a casa automáticamente."})
                            elif battery < BAT_LOW_WARNING:
                                print("🟡 Nivel de batería bajo. Solicitando confirmación al usuario.")
                                await self.broadcast_to_clients({"alerta": "Batería baja. ¿Deseas regresar a casa?", "accion_requerida": "confirmar_rtl"})

                     # 👇 Lógica de seguridad basada en señal GPS
                    if msg.get_type() == "GPS_RAW_INT":
                        gps_fix = getattr(msg, "fix_type", 0)
                        satellites = getattr(msg, "satellites_visible", 0)
                        if gps_fix < GPS_FIX_MIN or satellites < SAT_MIN:
                            print("🟡 Señal GPS débil.")
                            await self.broadcast_to_clients({
                                "alerta": "Señal GPS débil. ¿Deseas continuar la misión?",
                                "accion_requerida": "confirmar_continuar"
                            })


                    await self.broadcast_to_clients(data)  # Envía la telemetría al frontend
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
                9: "Land",
                10: "Auto",
                11: "RTL",
                12: "Loiter",
                15: "Guided",
                16: "Initialising",
        }
        if msg.get_type() == "HEARTBEAT" and hasattr(msg, "custom_mode"):
            return flight_modes.get(msg.custom_mode, f"Modo {msg.custom_mode}")
        return None

    async def broadcast_to_clients(self, data):
        for client in list(websocket_clients):
            try:
                await client.send(json.dumps(data))
            except:
                print("❌ Error al enviar a un cliente. Eliminando.")
                websocket_clients.discard(client)

    async def disconnect(self, close_code):
        """Cierra la conexión de manera ordenada."""
        print(f"🔴 WebSocket desconectado: {close_code}")
        websocket_clients.discard(self)

        if not websocket_clients and hasattr(self, "telemetry_task"):
            self.running = False
            self.telemetry_task.cancel()
            try:
                await self.telemetry_task
            except asyncio.CancelledError:
                pass

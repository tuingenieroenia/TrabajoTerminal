import asyncio
import json
from channels.generic.websocket import AsyncWebsocketConsumer
from pymavlink import mavutil

class TelemetriaConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        """Maneja la conexión WebSocket e inicia la conexión con el dron."""
        await self.accept()
        try:
            # Conectar con el dron
            print("🔄 Intentando conectar al dron...")
            self.drone = mavutil.mavlink_connection("udp:127.0.0.1:14550")

            # Intentar recibir el heartbeat sin bloquear el sistema
            for _ in range(5):  # Intentos limitados
                heartbeat = await asyncio.to_thread(self.drone.recv_match, type="HEARTBEAT", blocking=False)
                if heartbeat:
                    print("✅ Conectado al dron")
                    break
                await asyncio.sleep(1)  # Espera 1 segundo antes de intentar de nuevo
            else:
                print("❌ No se recibió heartbeat del dron. Verifica SITL o Mission Planner.")
                await self.close()
                return

            # Iniciar la transmisión de telemetría
            self.running = True
            self.telemetry_task = asyncio.create_task(self.enviar_telemetria())

        except Exception as e:
            print(f"❌ Error conectando con el dron: {e}")
            await self.close()

    async def enviar_telemetria(self):
        """Envía datos de telemetría en tiempo real."""
        try:
            while self.running:
                msg = await asyncio.to_thread(self.drone.recv_match, type=['ATTITUDE', 'GLOBAL_POSITION_INT'], blocking=True, timeout=1)
                if msg:
                    data = {
                        "tipo": msg.get_type(),
                        "pitch": getattr(msg, "pitch", None),
                        "roll": getattr(msg, "roll", None),
                        "yaw": getattr(msg, "yaw", None),
                        "lat": getattr(msg, "lat", None) / 1e7 if getattr(msg, "lat", None) else None,
                        "lon": getattr(msg, "lon", None) / 1e7 if getattr(msg, "lon", None) else None,
                        "alt": getattr(msg, "alt", None) / 1000 if getattr(msg, "alt", None) else None,
                    }
                    if self.scope["type"] == "websocket":
                        await self.send(json.dumps(data))

                await asyncio.sleep(0.5)  # Ajusta la frecuencia de actualización

        except asyncio.CancelledError:
            print("🛑 Tarea de telemetría cancelada.")
        except Exception as e:
            print(f"❌ Error en la telemetría: {e}")

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

from pymavlink import mavutil
import time

def cambiar_a_loiter(drone):
    print("📡 Forzando cambio a LOITER vía RC_OVERRIDE (modo extendido)...")

    try:
        # NAV_FORCE_TERMINATION por seguridad
        drone.mav.param_set_send(
            drone.target_system,
            drone.target_component,
            b"NAV_FORCE_TERMINATION",
            1,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        time.sleep(1)

        # Repetir RC_OVERRIDE para asegurar persistencia
        print("🎮 Enviando override canal 5 (PWM 1300) múltiples veces...")
        for _ in range(15):
            drone.mav.rc_channels_override_send(
                drone.target_system,
                drone.target_component,
                0, 0, 0, 0,
                1300,  # Canal 5: modo LOITER
                0, 0, 0
            )
            time.sleep(0.2)

        # Confirmar modo
        loiter_mode = drone.mode_mapping().get("LOITER")
        for _ in range(6):
            hb = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb and getattr(hb, 'custom_mode', -1) == loiter_mode:
                print("✅ Confirmado: modo actual es LOITER")
                break
            time.sleep(1)
        else:
            print("⚠️ No se confirmó el cambio a LOITER por override")
            return False

        # Apagar override
        print("🚫 Desactivando RC_OVERRIDE...")
        drone.mav.rc_channels_override_send(
            drone.target_system,
            drone.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        return True

    except Exception as e:
        print(f"❌ Error durante el override extendido: {e}")
        return False
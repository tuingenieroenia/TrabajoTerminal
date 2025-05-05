from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from django.utils import timezone
from api.models import RutasVuelo
from api.serializers import RutasVueloSerializer
import json
from api.drone_commands.mission_manager import obtener_ruta

# Coordenadas del punto "Home"
HOME_COORDENADAS = {"lat": 19.512379, "lon": -99.127972, "alt": 5, "cmd": "HOME"}

@api_view(['GET'])
def listar_rutas(request):
    rutas = RutasVuelo.objects.all()
    serializer = RutasVueloSerializer(rutas, many=True)
    return Response(serializer.data)

@api_view(['GET'])
def obtener_detalle_ruta(request, folio):
    try:
        # Ruta original (sin optimizar)
        original = obtener_ruta(folio, optimizar=False)
        optimizada = obtener_ruta(folio, optimizar=True)

        original_json = [{"lat": lat, "lon": lon} for lat, lon in original]
        optimizada_json = [{"lat": lat, "lon": lon} for lat, lon in optimizada]

        return Response({
            "original": original_json,
            "optimizada": optimizada_json
        })

    except Exception as e:
        return Response({"error": f"No se pudo obtener la ruta: {str(e)}"}, status=500)

@api_view(['POST'])
def guardar_ruta(request):
    data = request.data
    folio_dron = data.get('folio_dron')
    folio_usuario = data.get('folio_usuario')
    coordenadas = data.get('coordenadas')

    if not (folio_dron and folio_usuario and coordenadas):
        return Response({"error": "Faltan datos"}, status=400)
    
    # Asegurar que la altitud de cada punto sea 5 metros si no está definida
    for punto in coordenadas:
        if "alt" not in punto:
            punto["alt"] = 5  # Altitud por defecto

    # Agregar "Home" como el primer y último punto de la ruta
    ruta_completa = [HOME_COORDENADAS] + coordenadas + [HOME_COORDENADAS]

    nueva_ruta = RutasVuelo.objects.create(
        folio_dron_id=folio_dron,
        folio_usuario_id=folio_usuario,
        fecha=timezone.now(),
        coordenadas=ruta_completa
    )

    return Response({"message": "Ruta guardada correctamente", "folio": nueva_ruta.folio, "coordenadas": ruta_completa}, status=201)

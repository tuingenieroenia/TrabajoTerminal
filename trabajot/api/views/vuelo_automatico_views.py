from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from django.utils import timezone
from ..models import RutasVuelo
from ..serializers import RutasVueloSerializer
import json

@api_view(['GET'])
def listar_rutas(request):
    rutas = RutasVuelo.objects.all()
    serializer = RutasVueloSerializer(rutas, many=True)
    return Response(serializer.data)

@api_view(['POST'])
def guardar_ruta(request):
    data = request.data
    folio_dron = data.get('folio_dron')
    folio_usuario = data.get('folio_usuario')
    coordenadas = data.get('coordenadas')

    if not (folio_dron and folio_usuario and coordenadas):
        return Response({"error": "Faltan datos"}, status=400)

    nueva_ruta = RutasVuelo.objects.create(
        folio_dron_id=folio_dron,
        folio_usuario_id=folio_usuario,
        fecha=timezone.now(),
        coordenadas=coordenadas
    )

    return Response({"message": "Ruta guardada correctamente", "folio": nueva_ruta.folio}, status=201)

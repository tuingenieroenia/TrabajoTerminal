from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import json
from ..models import Dron
from ..serializers import DronSerializer

@api_view(['POST'])
def registrar_dron(request):
    serializer = DronSerializer(data=request.data)
    if serializer.is_valid():
        serializer.save()
        return Response({"message": "Dron registrado correctamente", "data": serializer.data}, status=status.HTTP_201_CREATED)
    return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)

@api_view(['PUT'])
def editar_dron(request, dron_id):
    try:
        dron = Dron.objects.get(folio=dron_id)
    except Dron.DoesNotExist:
        return Response({"error": "Dron no encontrado"}, status=status.HTTP_404_NOT_FOUND)

    serializer = DronSerializer(dron, data=request.data, partial=True)
    if serializer.is_valid():
        serializer.save()
        return Response({"message": "Dron actualizado correctamente", "data": serializer.data})
    return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)

@api_view(['DELETE'])
def eliminar_dron(request, dron_id):
    try:
        dron = Dron.objects.get(folio=dron_id)
        dron.delete()
        return Response({"message": "Dron eliminado correctamente"}, status=status.HTTP_204_NO_CONTENT)
    except Dron.DoesNotExist:
        return Response({"error": "Dron no encontrado"}, status=status.HTTP_404_NOT_FOUND)

@api_view(['GET'])
def listar_drones(request):
    drones = Dron.objects.all()
    for dron in drones:
        if isinstance(dron.telemetria, str):
            dron.telemetria = json.loads(dron.telemetria)  # Convertir a dict

    serializer = DronSerializer(drones, many=True)
    return Response(serializer.data)

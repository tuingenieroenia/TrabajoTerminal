from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
from ..models import Notificaciones, EstadoConfirmacion
from ..serializers import NotificacionesSerializer

class NotificacionesListCreateAPIView(APIView):
    def get(self, request):
        notificaciones = Notificaciones.objects.all().order_by('-fecha')
        serializer = NotificacionesSerializer(notificaciones, many=True)
        return Response(serializer.data, status=status.HTTP_200_OK)

    def post(self, request):
        serializer = NotificacionesSerializer(data=request.data)
        if serializer.is_valid():
            serializer.save()
            return Response(serializer.data, status=status.HTTP_201_CREATED)
        return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)
    
class ActualizarEstadoConfirmacionAPIView(APIView):
    def patch(self, request, folio):
        try:
            notificacion = Notificaciones.objects.get(folio=folio)
        except Notificaciones.DoesNotExist:
            return Response({'error': 'Notificación no encontrada'}, status=status.HTTP_404_NOT_FOUND)

        nuevo_estado_id = request.data.get('estado_confirmacion')
        if not nuevo_estado_id:
            return Response({'error': 'Falta el campo estado_confirmacion'}, status=status.HTTP_400_BAD_REQUEST)

        try:
            nuevo_estado = EstadoConfirmacion.objects.get(folio=nuevo_estado_id)
        except EstadoConfirmacion.DoesNotExist:
            return Response({'error': 'Estado de confirmación no válido'}, status=status.HTTP_400_BAD_REQUEST)

        notificacion.estado_confirmacion = nuevo_estado
        notificacion.save()

        return Response({'mensaje': 'Estado actualizado correctamente'}, status=status.HTTP_200_OK)
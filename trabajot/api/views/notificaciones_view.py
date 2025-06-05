from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
from rest_framework.permissions import IsAuthenticated
from django.shortcuts import get_object_or_404
from ..models import Notificaciones
from ..serializers import NotificacionesSerializer

class NotificacionesListCreateAPIView(APIView):

    def get(self, request):
        notificaciones = Notificaciones.objects.all().order_by('-fecha')[:50]
        serializer = NotificacionesSerializer(notificaciones, many=True)
        return Response(serializer.data, status=status.HTTP_200_OK)


    def post(self, request):
        """Crear una nueva notificación usando NotificationSerializer."""
        serializer = NotificacionesSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data, status=status.HTTP_201_CREATED)


class NotificacionEstadoUpdateAPIView(APIView):

    def patch(self, request, folio):
        """Actualizar solo el estado_confirmacion de una notificación existente."""
        notificacion = get_object_or_404(Notificaciones, folio=folio)

        # Esperamos un campo 'estado_confirmacion_id' en el body
        nuevo_estado_id = request.data.get('estado_confirmacion_id')
        if nuevo_estado_id is None:
            return Response({'error': 'Falta el campo estado_confirmacion_id'}, status=status.HTTP_400_BAD_REQUEST)

        # Usamos el serializer para la actualización parcial
        serializer = NotificacionesSerializer(
            notificacion,
            data={'estado_confirmacion_id': nuevo_estado_id},
            partial=True
        )
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data, status=status.HTTP_200_OK)

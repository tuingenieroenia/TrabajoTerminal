from rest_framework import viewsets, status
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework.permissions import AllowAny, IsAuthenticated
from django.shortcuts import get_object_or_404
from ..models import Usuario
from ..serializers import UsuarioSerializer

class UsuarioViewSet(viewsets.ModelViewSet):
    queryset = Usuario.objects.all()
    serializer_class = UsuarioSerializer

class UsuarioCreateAPIView(APIView):
    permission_classes = [AllowAny]

    def post(self, request):
        serializer = UsuarioSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)

        correo = serializer.validated_data['correo']
        if Usuario.objects.filter(correo=correo).exists():
            return Response(
                {'error': 'Este correo ya está registrado.'},
                status=status.HTTP_400_BAD_REQUEST
            )

        usuario = serializer.save()
        return Response(
            UsuarioSerializer(usuario).data,
            status=status.HTTP_201_CREATED
        )


class UsuarioRetrieveByEmailAPIView(APIView):

    def get(self, request, correo):
        # Busca por correo, no por folio
        usuario = get_object_or_404(Usuario, correo=correo)
        data = UsuarioSerializer(usuario).data
        return Response(data, status=status.HTTP_200_OK)


class UsuarioUpdateByEmailAPIView(APIView):

    def put(self, request, correo):
        # 1) Obtenemos el usuario por correo
        usuario = get_object_or_404(Usuario, correo=correo)

        # 2) Sólo aceptamos nombre y apellidos
        data = {}
        if 'nombre' in request.data:
            data['nombre'] = request.data['nombre']
        if 'apellidos' in request.data:
            data['apellidos'] = request.data['apellidos']

        # 3) Serializamos y validamos
        serializer = UsuarioSerializer(usuario, data=data, partial=False)
        serializer.is_valid(raise_exception=True)

        # 4) Guardamos y devolvemos
        serializer.save()
        return Response(serializer.data, status=status.HTTP_200_OK)
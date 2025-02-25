from rest_framework import viewsets, status
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework.permissions import AllowAny
from django.contrib.auth.hashers import make_password
from ..models import Usuario
from ..serializers import UsuarioSerializer

class UsuarioViewSet(viewsets.ModelViewSet):
    queryset = Usuario.objects.all()
    serializer_class = UsuarioSerializer

class RegisterView(APIView):
    permission_classes = [AllowAny]

    def post(self, request):
        data = request.data
        if Usuario.objects.filter(correo=data['correo']).exists():
            return Response({'error': 'Este correo ya está registrado.'}, status=status.HTTP_400_BAD_REQUEST)

        usuario = Usuario.objects.create(
            nombre=data['nombre'],
            apellidos=data['apellidos'],
            correo=data['correo'],
            contrasena=make_password(data['contrasena'])
        )

        return Response({'message': 'Usuario registrado con éxito'}, status=status.HTTP_201_CREATED)

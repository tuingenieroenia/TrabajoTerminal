from rest_framework import serializers
from .models import Usuario, Dron, RutasVuelo

class UsuarioSerializer(serializers.ModelSerializer):
    class Meta:
        model = Usuario
        fields = ['nombre', 'apellidos', 'correo', 'contrasena']
        extra_kwargs = {'contrasena': {'write_only': True}}  # No devolver la contraseña en respuestas

class DronSerializer(serializers.ModelSerializer):
    class Meta:
        model = Dron
        fields = '__all__'

class RutasVueloSerializer(serializers.ModelSerializer):
    class Meta:
        model = RutasVuelo
        fields = '__all__'

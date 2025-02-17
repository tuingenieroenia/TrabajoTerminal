from rest_framework import serializers
from .models import Usuario, Dron

class UsuarioSerializer(serializers.ModelSerializer):
    class Meta:
        model = Usuario
        fields = '__all__'

class DronSerializer(serializers.ModelSerializer):
    class Meta:
        model = Dron
        fields = '__all__'

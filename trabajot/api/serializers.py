from rest_framework import serializers
from .models import Usuario, Dron, RutasVuelo, Notificaciones, TipoAlerta, EstadoConfirmacion

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

class TiposAlertasSerializer(serializers.ModelSerializer):
    class Meta:
        model = TipoAlerta
        fields = ['folio', 'descripcion']
        
class EstadosConfirmacionSerializer(serializers.ModelSerializer):
    class Meta:
        model = EstadoConfirmacion
        fields = ['folio', 'estado']
        
class NotificacionesSerializer(serializers.ModelSerializer):
    tipo = TiposAlertasSerializer(read_only=True)
    estado_confirmacion = EstadosConfirmacionSerializer(read_only=True)
    folio_dron = DronSerializer(read_only=True)

    class Meta:
        model = Notificaciones
        fields = '__all__'
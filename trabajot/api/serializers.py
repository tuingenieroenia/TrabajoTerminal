from rest_framework import serializers
from .models import Usuario, Dron, RutasVuelo, Notificaciones, TipoAlerta, EstadoConfirmacion

class UsuarioSerializer(serializers.ModelSerializer):
    correo = serializers.EmailField(read_only=True)
    class Meta:
        model = Usuario
        fields = ['nombre', 'apellidos', 'correo']
        
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
    # READ-ONLY nested serializers for convenient output
    tipo = TiposAlertasSerializer(read_only=True)
    estado_confirmacion = EstadosConfirmacionSerializer(read_only=True)
    folio_dron = DronSerializer(read_only=True)

    # WRITE-ONLY PK fields to accept integer IDs
    tipo_id = serializers.PrimaryKeyRelatedField(
        source='tipo', queryset=TipoAlerta.objects.all(), write_only=True
    )
    estado_confirmacion_id = serializers.PrimaryKeyRelatedField(
        source='estado_confirmacion', queryset=EstadoConfirmacion.objects.all(), write_only=True
    )
    folio_dron_id = serializers.PrimaryKeyRelatedField(
        source='folio_dron', queryset=Dron.objects.all(), write_only=True
    )

    # Frame como URLField para validación
    frame = serializers.URLField()

    class Meta:
        model = Notificaciones
        fields = [
            'folio', 'fecha', 'longitud', 'latitud',
            'tipo', 'tipo_id',
            'estado_confirmacion', 'estado_confirmacion_id',
            'folio_dron', 'folio_dron_id',
            'frame',
        ]
        read_only_fields = ['folio']
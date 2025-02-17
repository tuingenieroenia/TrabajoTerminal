from django.db import models

class Usuario(models.Model):
    folio = models.AutoField(primary_key=True)
    nombre = models.CharField(max_length=255)
    apellidos = models.CharField(max_length=255)
    correo = models.EmailField(unique=True)
    contrasena = models.CharField(max_length=255)
    class Meta:
        db_table = 'users'  # Asegura que use la tabla existente

class Dron(models.Model):
    controladora_de_vuelo = models.CharField(max_length=255)
    frame = models.CharField(max_length=255)
    helices = models.IntegerField()
    motores = models.IntegerField()
    transmisores = models.CharField(max_length=255)
    estado = models.CharField(max_length=255)
    telemetria = models.JSONField()
    class Meta:
        db_table = 'dron'

class TipoAlerta(models.Model):
    folio = models.AutoField(primary_key=True)
    descripcion = models.CharField(max_length=255)

    class Meta:
        db_table = 'tipo_alerta'

class EstadoConfirmacion(models.Model):
    folio = models.AutoField(primary_key=True)
    estado = models.CharField(max_length=255)

    class Meta:
        db_table = 'estado_confirmacion'

class RutasVuelo(models.Model):
    folio = models.AutoField(primary_key=True)
    folio_dron = models.ForeignKey(Dron, on_delete=models.CASCADE, db_column='folio_dron')
    folio_usuario = models.ForeignKey(Usuario, on_delete=models.CASCADE, db_column='folio_usuario')
    fecha = models.DateTimeField()
    coordenadas = models.JSONField()  # Ajustado para que sea compatible con PostgreSQL

    class Meta:
        db_table = 'rutas_vuelo'

class Notificaciones(models.Model):
    folio = models.AutoField(primary_key=True)
    fecha = models.DateTimeField()
    longitud = models.FloatField()
    latitud = models.FloatField()
    tipo = models.ForeignKey(TipoAlerta, on_delete=models.CASCADE, db_column='tipo')
    estado_confirmacion = models.ForeignKey(EstadoConfirmacion, on_delete=models.CASCADE, db_column='estado_confirmacion')
    folio_dron = models.ForeignKey(Dron, on_delete=models.CASCADE, db_column='folio_dron')

    class Meta:
        db_table = 'notificaciones'

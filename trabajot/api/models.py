from django.db import models

class Usuario(models.Model):
    nombre = models.CharField(max_length=255)
    apellidos = models.CharField(max_length=255)
    correo = models.EmailField(unique=True)
    contrasena = models.CharField(max_length=255)

class Dron(models.Model):
    controladora_de_vuelo = models.CharField(max_length=255)
    frame = models.CharField(max_length=255)
    helices = models.IntegerField()
    motores = models.IntegerField()
    transmisores = models.CharField(max_length=255)
    estado = models.CharField(max_length=255)
    telemetria = models.JSONField()

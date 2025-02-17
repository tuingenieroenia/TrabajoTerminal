from django.shortcuts import render
from rest_framework import viewsets
from .models import Usuario, Dron
from .serializers import UsuarioSerializer, DronSerializer

class UsuarioViewSet(viewsets.ModelViewSet):
    queryset = Usuario.objects.all()
    serializer_class = UsuarioSerializer

class DronViewSet(viewsets.ModelViewSet):
    queryset = Dron.objects.all()
    serializer_class = DronSerializer


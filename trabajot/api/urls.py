from django.urls import path, include
from rest_framework.routers import DefaultRouter
from .views import UsuarioViewSet, DronViewSet

router = DefaultRouter()
router.register(r'usuarios', UsuarioViewSet)
router.register(r'drones', DronViewSet)

urlpatterns = [
    path('', include(router.urls)),
]

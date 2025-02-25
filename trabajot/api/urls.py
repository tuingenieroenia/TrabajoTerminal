from django.urls import path, include
from rest_framework.routers import DefaultRouter
from rest_framework_simplejwt.views import TokenObtainPairView, TokenRefreshView
from .views import usuarios_views, drones_views, vuelo_manual_views, vuelo_automatico_views

router = DefaultRouter()
router.register(r'usuarios', usuarios_views.UsuarioViewSet)

urlpatterns = [
    # Autenticación
    path('', include(router.urls)),
    path('auth/login/', TokenObtainPairView.as_view(), name='token_obtain_pair'),
    path('auth/refresh/', TokenRefreshView.as_view(), name='token_refresh'),
    path('auth/register/', usuarios_views.RegisterView.as_view(), name='register'),

    # CRUD de drones
    path('drones/registrar/', drones_views.registrar_dron, name='registrar_dron'),
    path('drones/editar/<int:dron_id>/', drones_views.editar_dron, name='editar_dron'),
    path('drones/eliminar/<int:dron_id>/', drones_views.eliminar_dron, name='eliminar_dron'),
    path('drones/', drones_views.listar_drones, name='listar_drones'),

    # Vuelo manual (teclas WASD)
    path('vuelo/manual/', vuelo_manual_views.mover_dron, name='mover_dron'),

    # Vuelo automático (rutas predefinidas)
    path('rutas/', vuelo_automatico_views.listar_rutas, name='listar_rutas'),
    path('rutas/guardar/', vuelo_automatico_views.guardar_ruta, name='guardar_ruta'),
]

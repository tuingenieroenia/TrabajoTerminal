from django.urls import path
from api.consumers import TelemetriaConsumer

websocket_urlpatterns = [
    path("ws/telemetria/", TelemetriaConsumer.as_asgi()),
]
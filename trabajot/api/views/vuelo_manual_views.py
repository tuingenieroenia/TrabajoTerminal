from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status

@api_view(['POST'])
def mover_dron(request):
    comando = request.data.get('comando')

    if not comando:
        return Response({"error": "Comando no recibido"}, status=400)

    # Aquí se enviaría el comando al dron usando MAVLink
    print(f"Comando recibido: {comando}")  # Simulación

    return Response({"message": f"Comando {comando} enviado al dron"}, status=200)

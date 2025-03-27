from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
from ..models import TipoAlerta
from ..serializers import TiposAlertasSerializer

# 🔹 Vista para TipoAlerta
class TipoAlertaListCreateAPIView(APIView):
    def get(self, request):
        alertas = TipoAlerta.objects.all()
        serializer = TiposAlertasSerializer(alertas, many=True)
        return Response(serializer.data, status=status.HTTP_200_OK)

    def post(self, request):
        serializer = TiposAlertasSerializer(data=request.data)
        if serializer.is_valid():
            serializer.save()
            return Response(serializer.data, status=status.HTTP_201_CREATED)
        return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)